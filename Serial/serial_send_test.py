# Importing libraries
import serial
import serial.tools.list_ports
import time

device_1_ID = "USB VID:PID=2E8A:F00A SER=E66164084356B639 LOCATION=1-1"
device_2_ID = "USB VID:PID=2E8A:F00A SER=E661640843278B39 LOCATION=1-1"
device_3_ID = "USB VID:PID=2E8A:F00A SER=E661640843323431 LOCATION=1-2.2.1.4.1"
device_4_ID = "USB VID:PID=2E8A:F00A SER=E661640843315C31 LOCATION=1-1"

class mcu_serial():
    '''Object for sending an receiving data over a serial connection using a custom protocol'''

    # Attributes
    allow_reconnection_timeout = False # Default is false

    # ------------------------ Setup Functions ------------------------

    def __init__(self, deviceID, datarate=115200):
        '''Initialiser'''

        # Setup
        self.serial_number  = self.get_serial_number(deviceID)
        self.datarate       = datarate
        self.timeout        = 0
        self.device         = ""
        self.startCharacter = "!"
        self.connect()

        # Basic variables
        self.data_received = None
        self.code_received = None

        # Motor variables

        # Stores motor speeds after a read
        self.motor_speed   = 0

    def get_serial_number(self, hwid):
        '''Gets the serial number for the deviceID/hwid. Handles errors'''

        # Setting up function variables
        serial_number = ""

        # Tries to split the hwid into the serial number
        try:        
            serial_number = (hwid.split("SER=")[1]).split(" ")[0]

        # if an error occurs
        except (IndexError, SyntaxError):
            print("Invalid hwid / deviceID")
        
        # Returns the serial number
        return serial_number

    def get_port(self):
        '''Gets the port of a device with know serial number. Error handling not needed'''
        
        # Steps through connected devices
        for port, desc, hwid in serial.tools.list_ports.comports():

            # Return the port if the serial number matches
            if (self.get_serial_number(hwid) == self.serial_number):
                return port
        
        # If no connected devices match the serial number
        print("No device with matching serial number")
        return None

    def is_open(self):
        '''Checks if the port is open. Doesn't handle errors'''

        # Setup function code
        code = b'102'

        # Gets the port status
        open = self.device.is_open

        # If port is open
        if open:
            code = b'202'
        
        # If port is closed
        else:
            code = b'402'

        return code


    def connect(self):
        '''Attempts to connect to the MCU once. Handles errors'''
        
        # Setup function code
        code = b'101'
        
        # Tries to connect
        try:
            # Gets the port
            self.port = self.get_port()

            # Sets up the connection
            self.device = serial.Serial(self.port, self.datarate, timeout = self.timeout)

            # Check if the port has been opened
            if self.is_open() == b'202':
                print("Connected")
                code = b'200'
            else:
                print("Failed to start connection")
                code = b'400'

        # If no device found
        except serial.serialutil.SerialException as error:
            print(error, "Failed to start connection, no device found")
            code = b'401'

        return code

mcu1 = mcu_serial(device_4_ID)

#mcu1.device.write(bytes("LEDW11", 'utf-8'))
