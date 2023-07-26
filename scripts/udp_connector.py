#!/bin/python3
import rclpy
from rclpy.node import Node
import time

from moteus_msgs.msg import MoteusStateArray, MoteusState, MoteusCommand, MoteusCommandArray
from rclpy.executors import MultiThreadedExecutor
import threading

import socket
import struct

class UdpSender(Node):
    def __init__(self):
        super().__init__('udp_sender')
        self.subscription = self.create_subscription(
            MoteusStateArray,
            'state',
            self.states_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.host = "192.168.0.32"
        self.port = 9998

    def states_callback(self, msg):
        for state in msg.moteus_states:
            self.send_state_udp(self.host, self.port, state)

    def send_state_udp(self, host: list, port: int, state: MoteusState):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        num_list = ([state.id,
                    0,0,
                    int(state.rezero_state),
                    (state.position),
                    (state.i2c_position),
                    (state.velocity),
                    (state.torque),
                    (state.q_current),
                    (state.d_current),
                    (state.voltage),
                    (state.temperature)])
        packed_data = struct.pack('!BBBB' + 'f'*8, *num_list)
        udp_socket.sendto(packed_data, (host, port))
        udp_socket.close()

class UdpReceiver(Node):
    def __init__(self):
        super().__init__('udp_reviever')
        self.publisher_ = self.create_publisher(
            MoteusCommandArray,
            'command',
            10)
        self.host = "0.0.0.0"
        self.port = 9999

        self.motor_ids = [1]

    def receive_udp_bytes(self, host="0.0.0.0", port=9999):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind((host, port))

        
        
        print(f"Listening for UDP packets on {host}:{port}...")
        all_received = False
        while rclpy.ok() and not all_received:
            command = MoteusCommand()
            command_array = MoteusCommandArray()
            
            receive_state = {}
            for id in self.motor_ids:
                receive_state[id] = False
            
            try:
                data, addr = udp_socket.recvfrom(32)

                command.id, = struct.unpack('B', data[:1])
                command.position, command.velocity, command.maximum_torque, command.acceleration = struct.unpack('!' + 'f'*4, data[1:])
            except:
                print("UDP receive error!")

            # print(command.id)
            # print(command.position, command.velocity, command.maximum_torque, command.stop_position)
            
            command_array.moteus_commands.append(command)
            self.publisher_.publish(command_array)
            # time.sleep(1)
        
        
    def pub_command(self, command: MoteusCommandArray):
        self.publisher_.publish(command)

    def receive_loop(self):
        self.receive_udp_bytes(self.host , self.port)
        self.pub_command
    
def main(args=None):
    rclpy.init(args=args)
    sender = UdpSender()
    receiver = UdpReceiver()

    executor = MultiThreadedExecutor()
    executor.add_node(sender)

    receiver_thread = threading.Thread(target=receiver.receive_udp_bytes)
    receiver_thread.start()
    executor.spin()
    #   receiver.receive_udp_bytes()
    
    receiver_thread.join()
    executor.shutdown()
    sender.destroy_node()
    receiver.destroy_node()
        
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
