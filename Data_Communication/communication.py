import time
from pymavlink import mavutil

# Replace with your Pixhawk's connection details
# For USB: connection_string = 'usb:'
# For UART: connection_string = '/dev/serial0' or '/dev/ttyAMA0'
connection_string = '/dev/ttyS0'
baud_rate = 57600  # Match Pixhawk's baud rate (default: 57600)

def connect_to_pixhawk(connection_string, baud_rate):
    print("Connecting to Pixhawk...")
    try:
        master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        master.wait_heartbeat()
        print("Connection established! Heartbeat received.")
        return master
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def send_data_to_pixhawk(master):
    try:
        print("Sending test message to Pixhawk...")
        master.mav.ping_send(
            int(time.time() * 1e6),  # Timestamp in microseconds
            0, 0, 0  # Ping ID, Target System, Target Component
        )
        print("Test message sent.")
    except Exception as e:
        print(f"Error sending data: {e}")

def receive_data_from_pixhawk(master):
    try:
        print("Waiting for messages from Pixhawk...")
        while True:
            message = master.recv_match(blocking=False)
            if message:
                print(f"Received: {message}")
                return
            time.sleep(0.1)  # Avoid busy-waiting
    except Exception as e:
        print(f"Error receiving data: {e}")

def main():
    master = connect_to_pixhawk(connection_string, baud_rate)
    if not master:
        return

    send_data_to_pixhawk(master)
    receive_data_from_pixhawk(master)

if __name__ == "__main__":
    main()
