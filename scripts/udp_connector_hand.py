import socket
import struct
import serial
import threading
import time

class UdpReceiver:
    def __init__(self):
        self.host = "0.0.0.0"
        self.port = 9999
        self.left_hand_serial_port = "/dev/left_hand"
        self.right_hand_serial_port = "/dev/right_hand"

        self.ser_left = serial.Serial(self.left_hand_serial_port, baudrate=57600, timeout=0.1)
        self.ser_right = serial.Serial(self.right_hand_serial_port, baudrate=57600, timeout=0.1)

        self.lock = threading.Lock()  # Lock to synchronize access to serial ports

    def receive_udp_bytes(self, host="0.0.0.0", port=9997):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind((host, port))
        udp_socket.setblocking(False)

        print(f"Listening for UDP packets on {host}:{port}...")

        while True:
            try:
                data, addr = udp_socket.recvfrom(11)
                # time.sleep(0.01)
                
                # Discard any previously stacked UDP packets
                while True:
                    try:
                        data, addr = udp_socket.recvfrom(11)
                    except socket.error:
                        break
                    print("Discarding stacked UDP packet...")

                received_numbers = struct.unpack('B' * 10, data[1:11])
                left_finger_cmd = [255] + list(received_numbers[:5])
                right_finger_cmd = [255] + list(received_numbers[5:])

                with self.lock:
                    self.send_to_serial_port(left_finger_cmd, self.ser_left)
                    self.send_to_serial_port(right_finger_cmd, self.ser_right)

            except socket.error:
                pass  # No data available, continue to the next iteration

    def send_to_serial_port(self, numbers, serial_port):
        try:
            data = struct.pack('B' * len(numbers), *numbers)
            serial_port.write(data)
            print(numbers)
        except serial.SerialException as e:
            print(f"Serial port error: {e}")

def main(args=None):
    receiver = UdpReceiver()

    # Create separate threads for UDP receiving and serial sending
    udp_thread = threading.Thread(target=receiver.receive_udp_bytes)
    udp_thread.start()

    # Do other things in the main thread if needed

    # Wait for the UDP thread to finish (this won't happen in this example)
    udp_thread.join()

    # Close serial ports when done
    receiver.ser_left.close()
    receiver.ser_right.close()

if __name__ == "__main__":
    main()
