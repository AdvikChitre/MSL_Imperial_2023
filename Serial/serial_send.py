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

    def reconnect(self):
        '''Attempts to reconnect repeatedly until connection made or timeout. Error handling not needed'''

        # Setup function code
        code = b'105'

        # Setup function variables
        connected    = False
        attempts     = 0
        max_attempts = 10
        
        # Tries to connect if not connected. Stops after the max number of attempts if timeout is allowed
        while not connected and ((attempts < max_attempts) or not self.allow_reconnection_timeout):
                        
            # Print the attempt number
            print("Attempt", attempts + 1)

            # Attempt Connection
            connection_code = self.connect()

            # The connection failed
            if connection_code in (b'400', b'401'):
                connected = False
                code = b'405'

                # Only wait if the connection failed
                time.sleep(1)

            # The connection succeeded
            else:
                connected = True
                code = b'205'

            # Increment the number of attempts
            attempts += 1
        
        return code
    
    def check_connection(self):
        '''Checks the status of the conncetion, and reconnects if disconnected. Handles errors'''

        # Setup function code
        code = b'103'

        # Setup function variables
        connected = False

        # Try and get the port's status
        try:
            connected = (self.is_open() == b'202')

            # Interpret port status
            if connected:
                print("Good connection, port is open")
                code = b'203'
            else:
                print("Port is closed, attempting reconnection...")
                self.reconnect()
                code = b'403'

        # If no device connected
        except serial.serialutil.SerialException:
            connected = False
            print("No device connected, attempting reconnection...")
            code = b'404'
            
            # Attempts to reconnect
            if self.reconnect() == b'205':
                code = b'203'

        return code

    # ------------------------ Basic Functions ------------------------

    def reset_input_buffer(self):
        '''Resets the input buffer. Handles errors'''

        # Setup function code
        code = b'106'

        # Setup function variables
        done = False

        # Loop to clear buffer
        while not done:
            
            # Tries to clear input buffer
            try:
                self.device.reset_input_buffer()
                code = b'206'
                done = True
            
            # Fails to clear input buffer
            except (serial.serialutil.PortNotOpenError, AttributeError):
                code = b'406'
                print("Failed to clear input buffer, attempting reconnection")
                self.reconnect()
            
            return code

    def send_byte(self, byte):
        '''Sends a single byte, accepts a string/char/int, does not check for confirmation of reciept. Handles errors'''

        # Setup function code
        code = b'110'

        # Tries to send byte
#        print("sent", byte)
        try:
            self.device.write(bytes(str(byte)[0], 'utf-8'))
            code = b'210'
        
        # If an error is raised
        except serial.serialutil.SerialException:
            print("Failed to send byte, attempting reconnection")
            code = b'410'

            # Attempts to reconnect
            self.reconnect()
            
        return code

    def read_data(self, length=1):
        '''Reads a number of bytes equal to the length input, stored in .data_received. Handles errors'''

        # Setup function code
        code = b'120'
        
        # Tries to receive data
        
        try:
            self.data_received = self.device.read(length)
            code = b'220'
        
        # Fails to recieve data
        except serial.serialutil.SerialException:
            print("Failed to read data, attempting reconnection")
            code = b'420'

            # Attempts to reconnect
            self.reconnect()

#        print("read", self.data_received)
        return code

    def read_code(self):
        '''Reads 3 bytes (should be a status code), stored in .code_received. Handles errors'''

        # Setup function code
        code = b'125'

        # Tries to receive code
        try:
            self.code_received = self.device.read(3)
            code = b'225'
            
            
        
        # Fails to recieve code
        except serial.serialutil.SerialException:
            print("Failed to read code, attempting reconnection")
            code = b'425'

            # Attempts to reconnect
            self.reconnect()

#        print("read", self.code_received)
        
        return code

    def send_message(self, message):
        '''Sends a message string and recieves a confirmation. Error handling not needed'''
        
        # TODO add a break to the while loop after a certain number of tries. same as in reconnect? Might not be needed since this function calls reconnect

        # Setup function code
        code = b'111'

        # Setup function variables
        done = False

        # Flush the input buffers so we can receive data
#        self.reset_input_buffer()

        # Main loop to send message. Tries until a success
        while not done:

            # Setup loop variables
            error = False



            # Send the start of message charracter
            self.send_byte(self.startCharacter)

            # Send the message byte by byte
            for i in message:

                # Send and check if sent properly
                if self.send_byte(i) != b'210':
                    error = True
                    break
                

            # Proceed to confirmation stage if no error in sending
            if not error:
#                self.read_data(1);

                # Receive confirmation
                self.read_code()

                # If success received
                if self.code_received == b'220':
                    print("Data sent successully")
                    code = b'211'
                    done = True

                # If fail received
                elif self.code_received == b'420':
                    print("Data sent but error occured")
                    code = b'412'
                
                # No confirmation
                else:
#                    print("Data sent but no confirmation recieved")
                    code = b'411'
            
        return code
            
    # ------------------------ User Functions ------------------------

    def led(self, status=2):
        '''Writes status (0, 1, 2 for off, on, or toggle) to built in LED'''

        # Setup function code
        code = b'130'
        
        # Setup function variables
        opcode = 'LED'
        readwrite = 'W'
        length = '1'
        
        # Converts the status into a string
        data = str(status)

        # If input is invalid end function and return failure
        if data not in ("0", "1", "2"):
            print("Invalid argument")
            code = b'430'
            return code

        # Create the message
        message = opcode+readwrite+length+data

        # Send the message and check sent correctly
        if (self.send_message(message) == b'211'):


            # Check function executed
            self.read_code()
#            print(self.code_received)

            if self.code_received == b'230':
                print("LED Write succeeded, status =", status)
                code = b'230'
            
            # If function not executed
            else:
                print("LED Write failed")
                code = b'430'
            
        # If not sent correctly
        else:
            print("LED Write failed")
            code = b'430'

#        self.read_data(10)
#        print(self.data_received)

        return code

    def motor(self, motor=1, speed=0, readwrite="w", actualtarget="a"):
        '''Interacts with an individual motor, if a speed is read, it is stored in .motor_speed'''

        # TODO error handling, input validation

        # Setup function code
        code = b'140'
        
        # Setup function variables
        opcode    = "M" + str(motor)[0] + str(actualtarget)[0].upper()
        readwrite = str(readwrite)[0].upper()
        length    = "5"

        # If writing to motor
        if readwrite == "W":

            # Format the data
            if (speed >= 0):
                data = "+" + str(round(speed, 2))
            else:
                data = str(round(speed, 2))

            # Sets the length of the data
            length = str(len(data))

            # Create the message
            message = opcode+readwrite+length+data

            # Send the message and check sent correctly
            if (self.send_message(message) == b'211'):

                # Check function executed
                self.read_code()
                if self.code_received == b'241':
                    print("Motor write succeeded, speed =", speed)
                    code = b'241'

                # If function not executed
                else:
                    print("Motor write failed")
                    code = b'441'

            # If not sent correctly
            else:
                print("Motor write failed")
                code = b'441'

        # If reading from motor
        elif readwrite == "R":

            # Create the message
            message = opcode+readwrite+length

            # Send the message and check sent correctly
            if (self.send_message(message) == b'211'):
                
                # Read data and check recieved correctly
                if self.read_data(int(length)) == b'220':

                    # Convert data to float and store in .motor_speed
                    self.motor_speed = float(self.data_received)
                    print("Motor read succeeded")
                    code = b'245'
                
                # If not received correctly
                else:
                    print("Motor Read failed")
                    code = b'445'

            # If not sent correctly
            else:
                print("Motor Read failed")
                code = b'445'
        
        # Invalid value of readwrite
        else:
            print("Invalid argument: readwrite")
            code = b'440'

        return code





mcu1 = mcu_serial(device_4_ID)

""" while True:
    start = time.time()
    mcu1.led()
    time.sleep(1)
    print(round((time.time() - start), 5)) """



""" for i in range(0,200,1):
    mcu1.motor(1, i/100)



for i in range(200,0,-1):
    mcu1.motor(1, i/100)


mcu1.motor(1, 0)

mcu1.motor(1, 1) """
