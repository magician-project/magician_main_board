import serial  # pip3 install pyserial
import time
import threading

# Flag to stop the main loop
stop_flag = False

def check_for_quit():
    global stop_flag
    while True:
        user_input = input()
        if user_input.strip().lower() == 'q':
            stop_flag = True
            break


print("Press 'q' and Enter to quit.\n")

# Open the serial port
serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1)
time.sleep(1)  # Give time for Arduino to reset

# Send 'i' to start Arduino output
serialPort.write(bytearray('i', 'ascii'))
serialPort.flush()

# Start a background thread to listen for 'q' input
threading.Thread(target=check_for_quit, daemon=True).start()


# Main loop
while not stop_flag:
    if serialPort.in_waiting > 0:
        serialPortByte = serialPort.read(1)
        try:
            char = serialPortByte.decode("ascii")
            print(char, end="")
        except UnicodeDecodeError:
            pass

# Send 'f' to stop Arduino output
serialPort.write(bytearray('f', 'ascii'))
serialPort.flush()
print("\nStopped.")

