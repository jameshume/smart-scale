# This is code is a modification of code from Initial State Technologies, Inc.
# There wasn't a copyright header when I forked their code but it is released
# under LGPL. No warrenty, liability etc for my changes.
#
# Requires
#    sudo apt-get install bluetooth libbluetooth-dev
#    sudo pip install pybluez
#
# Ported to Python 3. TODO: Make work on both versions.
import time
import bluetooth
import subprocess
import threading
import codecs
import collections

class EventProcessor(object):
    def __init__(self):
        self._done = False
        self._measurement_avail = False

    def mass(self, event):
        assert(False)

    def get_weight(self):
        assert(False)

    def measurement_available(self):
        return self._measurement_avail

    def is_done(self):
        return self._done

    def signal_done(self):
        self._done = True

#
# Simple wrapper that can make any EventProcessor subclass thread safe
class ThreadSafeEventProcessor:
    def __init__(self, event_processor):
        self._event_processor = event_processor
        self._lock = threading.Lock()

    def mass(self, event):
        with self._lock:
            self._event_processor.mass(event)

    def get_weight(self):
        with self._lock:
            return self._event_processor.get_weight()

    def measurement_available(self):
        with self._lock:
            return self._event_processor.measurement_available()

    def is_done(self):
        with self._lock:
            return self._event_processor.is_done()

    def signal_done(self):
        with self._lock:
            self._event_processor.signal_done()

   
#
# Take snapshots of weights by gathering n samples
class SnapShotWeightProcessor(EventProcessor):
    def __init__(self, samples_per_avg):
        super(SnapShotWeightProcessor, self).__init__()

        assert(samples_per_avg) >= 1
        self._samples_per_avg = samples_per_avg
        self._samples_count = 0
        self._samples = [0] * samples_per_avg
        self._weight_kg = 0

    def mass(self, event):
        self._samples[self._samples_count] = event.totalWeight
        self._samples_count += 1
        if self._samples_count == self._samples_per_avg:
             self._weight_kg = collections.Counter(round(num, 1) for num in self._samples).most_common(1)[0][0]
             self._samples_count = 0
             self._measurement_avail = True

    def get_weight(self):
        self._measurement_avail = False
        return self._weight_kg


       
class BoardEvent:
    def __init__(self, topLeft, topRight, bottomLeft, bottomRight, buttonPressed, buttonReleased):
        self.topLeft = topLeft
        self.topRight = topRight
        self.bottomLeft = bottomLeft
        self.bottomRight = bottomRight
        self.buttonPressed = buttonPressed
        self.buttonReleased = buttonReleased
        self.totalWeight = topLeft + topRight + bottomLeft + bottomRight

class Wiiboard:
    # Wiiboard Parameters
    CONTINUOUS_REPORTING = "04"  # Easier as string with leading zero
    COMMAND_LIGHT = 11
    COMMAND_REPORTING = 12
    COMMAND_REQUEST_STATUS = 15
    COMMAND_REGISTER = 16
    COMMAND_READ_REGISTER = 17
    INPUT_STATUS = 0x20
    INPUT_READ_DATA = 0x21
    EXTENSION_8BYTES = 0x32
    EXTENSION_8BYTES_STR = 32
    BUTTON_DOWN_MASK = 8
    TOP_RIGHT = 0
    BOTTOM_RIGHT = 1
    TOP_LEFT = 2
    BOTTOM_LEFT = 3
    BLUETOOTH_NAME = "Nintendo RVL-WBC-01"

    def __init__(self, processor):
        # Sockets and status
        self.receivesocket = None
        self.controlsocket = None

        assert(isinstance(processor, EventProcessor))
        self.processor = processor
        self.calibration = []
        self.calibrationRequested = False
        self.LED = False
        self.address = None
        self.buttonDown = False
        for i in range(3):
            self.calibration.append([])
            for j in range(4):
                self.calibration[i].append(10000)  # high dummy value so events with it don't register

        self.status = "Disconnected"
        self.lastEvent = BoardEvent(0, 0, 0, 0, False, False)

        try:
            self.receivesocket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
            self.controlsocket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        except ValueError:
            raise Exception("Error: Bluetooth not found")

    def isConnected(self):
        return self.status == "Connected"

    # Connect to the Wiiboard at bluetooth address <address>
    def connect(self, address):
        if address is None:
            print("Non existant address")
            return
        self.receivesocket.connect((address, 0x13))
        self.controlsocket.connect((address, 0x11))
        if self.receivesocket and self.controlsocket:
            print("Connected to Wiiboard at address " + address)
            self.status = "Connected"
            self.address = address
            self.calibrate()
            useExt = ["00", Wiiboard.COMMAND_REGISTER, "04", "A4", "00", "40", "00"]
            self.send(useExt)
            self.setReportingType()
            print("Wiiboard connected")
        else:
            print("Could not connect to Wiiboard at address " + address)

    def _receive_one_reading(self):
        while self.status == "Connected" and not self.processor.is_done():
            data = self.receivesocket.recv(25)
            datahex = codecs.encode(data, 'hex')

            #intype = int(data.encode("hex")[2:4])
            intype = data[1]
            if intype == Wiiboard.INPUT_STATUS:
                # TODO: Status input received. It just tells us battery life really
                self.setReportingType()
            elif intype == Wiiboard.INPUT_READ_DATA:
                if self.calibrationRequested:
                    #packetLength = (int(str(data[4]).encode("hex"), 16) / 16 + 1)
                    packetLength = int(data[4] / 16 + 1)
                    self.parseCalibrationResponse(data[7:(7 + packetLength)])

                    if packetLength < 16:
                        self.calibrationRequested = False
            elif intype == Wiiboard.EXTENSION_8BYTES:
                bev = self.createBoardEvent(data[2:12])
                return bev
            else:
                print("ACK to data write received")

    def receive_one_shot(self):    
        while self.status == "Connected" and not self.processor.is_done() and not self.processor.measurement_available():
            self.processor.mass(self._receive_one_reading())
        
        if self.processor.measurement_available():
            return self.processor.get_weight()
        else:
            return None

    def receive(self):
        while self.status == "Connected" and not self.processor.is_done():
            self.processor.mass(self._receive_one_reading())

    def disconnect(self):
        if self.status == "Connected":
            self.status = "Disconnecting"
            while self.status == "Disconnecting":
                self.wait(100)
        try:
            self.receivesocket.close()
        except:
            pass
        try:
            self.controlsocket.close()
        except:
            pass
        print("WiiBoard disconnected")

    # Try to discover a Wiiboard
    def discover(self):
        print("Press the red sync button on the board now")
        address = None
        bluetoothdevices = bluetooth.discover_devices(duration=6, lookup_names=True)
        for bluetoothdevice in bluetoothdevices:
            if bluetoothdevice[1] == Wiiboard.BLUETOOTH_NAME:
                address = bluetoothdevice[0]
                print("Found Wiiboard at address " + address)
        if address is None:
            print("No Wiiboards discovered.")
        return address

    def createBoardEvent(self, bytes):
        buttonBytes = bytes[0:2]
        bytes = bytes[2:12]
        buttonPressed = False
        buttonReleased = False

        #state = (int(buttonBytes[0].encode("hex"), 16) << 8) | int(buttonBytes[1].encode("hex"), 16)
        state = (buttonBytes[0] << 8) | buttonBytes[1]
        if state == Wiiboard.BUTTON_DOWN_MASK:
            buttonPressed = True
            if not self.buttonDown:
                print("Button pressed")
                self.buttonDown = True

        if not buttonPressed:
            if self.lastEvent.buttonPressed:
                buttonReleased = True
                self.buttonDown = False
                print("Button released")

        #rawTR = (int(bytes[0].encode("hex"), 16) << 8) + int(bytes[1].encode("hex"), 16)
        #rawBR = (int(bytes[2].encode("hex"), 16) << 8) + int(bytes[3].encode("hex"), 16)
        #rawTL = (int(bytes[4].encode("hex"), 16) << 8) + int(bytes[5].encode("hex"), 16)
        #rawBL = (int(bytes[6].encode("hex"), 16) << 8) + int(bytes[7].encode("hex"), 16)
        rawTR = (bytes[0] << 8) + bytes[1]
        rawBR = (bytes[2] << 8) + bytes[3]
        rawTL = (bytes[4] << 8) + bytes[5]
        rawBL = (bytes[6] << 8) + bytes[7]

        topLeft = self.calcMass(rawTL, Wiiboard.TOP_LEFT)
        topRight = self.calcMass(rawTR, Wiiboard.TOP_RIGHT)
        bottomLeft = self.calcMass(rawBL, Wiiboard.BOTTOM_LEFT)
        bottomRight = self.calcMass(rawBR, Wiiboard.BOTTOM_RIGHT)
        boardEvent = BoardEvent(topLeft, topRight, bottomLeft, bottomRight, buttonPressed, buttonReleased)
        return boardEvent

    def calcMass(self, raw, pos):
        val = 0.0
        #calibration[0] is calibration values for 0kg
        #calibration[1] is calibration values for 17kg
        #calibration[2] is calibration values for 34kg
        if raw < self.calibration[0][pos]:
            return val
        elif raw < self.calibration[1][pos]:
            val = 17 * ((raw - self.calibration[0][pos]) / float((self.calibration[1][pos] - self.calibration[0][pos])))
        elif raw > self.calibration[1][pos]:
            val = 17 + 17 * ((raw - self.calibration[1][pos]) / float((self.calibration[2][pos] - self.calibration[1][pos])))

        return val

    def getEvent(self):
        return self.lastEvent

    def getLED(self):
        return self.LED

    def parseCalibrationResponse(self, bytes):
        index = 0
        if len(bytes) == 16:
            for i in range(2):
                for j in range(4):
                    #self.calibration[i][j] = (int(bytes[index].encode("hex"), 16) << 8) + int(bytes[index + 1].encode("hex"), 16)
                    self.calibration[i][j] = (bytes[index] << 8) + bytes[index + 1]
                    index += 2
        elif len(bytes) < 16:
            for i in range(4):
                #self.calibration[2][i] = (int(bytes[index].encode("hex"), 16) << 8) + int(bytes[index + 1].encode("hex"), 16)
                self.calibration[2][i] = (bytes[index] << 8) + bytes[index + 1]
                index += 2

    # Send <data> to the Wiiboard
    # <data> should be an array of strings, each string representing a single hex byte
    def send(self, data):
        if self.status != "Connected":
            return
        data[0] = "52"

        senddata = b''
        for byte in data:
            byte = str(byte)
            senddata += codecs.decode(byte, "hex")

        self.controlsocket.send(senddata)

    #Turns the power button LED on if light is True, off if False
    #The board must be connected in order to set the light
    def setLight(self, light):
        if light:
            val = "10"
        else:
            val = "00"

        message = ["00", Wiiboard.COMMAND_LIGHT, val]
        self.send(message)
        self.LED = light

    def calibrate(self):
        message = ["00", Wiiboard.COMMAND_READ_REGISTER, "04", "A4", "00", "24", "00", "18"]
        self.send(message)
        self.calibrationRequested = True

    def setReportingType(self):
        bytearr = ["00", Wiiboard.COMMAND_REPORTING, Wiiboard.CONTINUOUS_REPORTING, Wiiboard.EXTENSION_8BYTES_STR]
        self.send(bytearr)

    def wait(self, millis):
        time.sleep(millis / 1000.0)

def test():
    print("RUNNING TESTS...")

    processor = SnapShotWeightProcessor(20)
    board = Wiiboard(processor)
    
    print("Discovering board...")
    address = board.discover()

    try:
        # Disconnect already-connected devices.
        # This is basically Linux black magic just to get the thing to work.
        subprocess.check_output(["bluez-test-input", "disconnect", address], stderr=subprocess.STDOUT)
        subprocess.check_output(["bluez-test-input", "disconnect", address], stderr=subprocess.STDOUT)
    except:
        pass

    print("Trying to connect...")
    board.connect(address)  # The wii board must be in sync mode at this time
    board.wait(200)
    for count in range(200):
        print(board.receive_one_shot())

if __name__ == "__main__":
    test()
