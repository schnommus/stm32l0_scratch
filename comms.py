import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/tty.usbserial-A400ftbl',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()


# Raw Report Descriptor - Type 2 (Mouse) and Length 5 bytes
raw_report = "\xFD\x05\x02"


# Send value from -128 to 128 for mouse positions
# NOTE: In y-direction, NEGATIVE IS UP

# NOTE: BUTTONS LATCH
buttons = "\x00";
x_move = "\x00";
y_move = "\x00";
wheel = "\x00";

#to_send = hex((raw_report << 32) + (buttons << 24) + (x_move << 16) + (y_move << 8) + wheel);

move = "\xFD\x05\x02\x00\x00\xBD\x00"
scroll = "\xFD\x05\x02\x00\x00\x00\xFD"
test_string_push = "\xFD\x05\x02\x01\x00\x00\x00"
test_string_release = "\xFD\x05\x02\x00\x00\x00\x00"


ser.write(test_string_push)
ser.write(test_string_release) 
#ser.write(test_string_push)
#ser.write(test_string_release)