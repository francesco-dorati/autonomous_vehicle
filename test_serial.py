import serial
import time

# Configuration
SERIAL_PORT = '/dev/ttyAMA0'  # Update this to the correct port
BAUD_RATE = 9600

def open_serial_port(port, baud_rate):
    """Open and return the serial port."""
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

def main():
    # Open the serial port
    ser = open_serial_port(SERIAL_PORT, BAUD_RATE)
    if not ser:
        return

    try:
        while True:
            # Read from Arduino
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').rstrip()
                print(f"Arduino: {data}")

            # Send data to Arduino
            user_input = input("You: ")
            if user_input.lower() == 'exit':
                break
            ser.write(user_input.encode('utf-8'))
            
    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        if ser:
            ser.close()
            print("Serial port closed.")

if __name__ == '__main__':
    main()
