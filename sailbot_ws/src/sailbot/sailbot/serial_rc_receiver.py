#!/usr/bin/env python3
import serial
import json
import time
import binascii

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SerialChannels:
    def __init__(self, CHANNELS):
        self.channels = []
        self.numChannels = CHANNELS - 1
        self.frame_error = 0
        self.failsafe = 0

class SerialRCReceiver(Node):

    def __init__(self):
        super().__init__('serial_rc_receiver')
        self.serial = serial.Serial() #I think this is safe to remove
        self.publisher_ = self.create_publisher(String, 'serial_rc', 10)
        timer_period = 0.01  # seconds, ALSO might need to change this, not certain how fast the rc decoding runs
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #RC Variables
        self.BUFFER_LENGTH = 25
        self.CHANNELS = 18
        self.MAX_READ_ATTEMPTS = 32
        self.buffer = []
        self.bufferOffset = 0
        self.JetsonSerial = serial.Serial(
            port="/dev/ttyTHS1", # RX terminal THS1 (Port 10 on J41 Header)
            baudrate=95000,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
        )
        self.serialData = SerialChannels(self.CHANNELS)
        time.sleep(1)#let port initialize


    def timer_callback(self):
        #get inputs
        self.receive()
        msg = String()
        msg.data = json.dumps(self.readLineToJson())
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def readLineToJson(self):
        if self.serialData.channels:
            return {"state1": self.serialData.channels[0], 
                    "ballast": self.serialData.channels[1], 
                    "rudder": self.serialData.channels[2], 
                    "manual": self.serialData.channels[4], 
                    "state2": self.serialData.channels[5]}
        else:#just returning 0 for now but not sure what should actually be sent when no data available
            return {"state1": 0,
                    "ballast": 892,
                    "rudder": 934,
                    "manual": 1130,
                    "state2": 992}


    def receive(self):
        data = 0
        counter = 0

        while self.JetsonSerial.in_waiting and (self.bufferOffset < self.BUFFER_LENGTH) and (counter < self.MAX_READ_ATTEMPTS):
            data = int(self.JetsonSerial.read().hex(), 16)
            if self.bufferOffset == 0 and data != 0x0f:
                data = 0
                continue
            self.buffer.append(data & 0xff)
            self.bufferOffset += 1

        if (self.bufferOffset == self.BUFFER_LENGTH):
            self.JetsonSerial.reset_input_buffer()

            if self.decodeSBUS():
                if self.serialData.failsafe:
                    self.serialData.channels = []
                    print("Failsafe failed")
                elif self.serialData.frame_error:
                    self.serialData.channels = []
                    print("Frame error")
                elif self.serialData.failsafe or self.serialData.frame_error:
                    print("Successful decode")

        self.buffer.clear()
        self.bufferOffset = 0

    def decodeSBUS(self):
        if (self.buffer[0] != 0x0f):
            print("Incorrect start bit")
            return False
        if (self.buffer[self.BUFFER_LENGTH - 1] != 0x00):
            print("Incorrect stop bit")
            return False
            # print("good bit")
            # print(buffer)
        self.serialData.channels = []
        dataChannels = self.serialData.channels

        dataChannels.append((self.buffer[1] | self.buffer[2] << 8) & 0x07FF)  # Channel 0
        dataChannels.append((self.buffer[2] >> 3 | self.buffer[3] << 5) & 0x07FF)  # Channel 1
        dataChannels.append((self.buffer[3] >> 6 | self.buffer[4] << 2 | self.buffer[5] << 10) & 0x07FF)  # Channel 2
        dataChannels.append((self.buffer[5] >> 1 | self.buffer[6] << 7) & 0x07FF)  # Channel 3
        dataChannels.append((self.buffer[6] >> 4 | self.buffer[7] << 4) & 0x07FF)  # Channel 4
        dataChannels.append((self.buffer[7] >> 7 | self.buffer[8] << 1 | self.buffer[9] << 9) & 0x07FF)  # Channel 5
        dataChannels.append((self.buffer[9] >> 2 | self.buffer[10] << 6) & 0x07FF)  # Channel 6
        dataChannels.append((self.buffer[10] >> 5 | self.buffer[11] << 3) & 0x07FF)  # Channel 7
        dataChannels.append((self.buffer[12] | self.buffer[13] << 8) & 0x07FF)  # Channel 8
        dataChannels.append((self.buffer[13] >> 3 | self.buffer[14] << 5) & 0x07FF)  # Channel 9
        dataChannels.append((self.buffer[14] >> 6 | self.buffer[15] << 2 | self.buffer[16] << 10) & 0x07FF)  # Channel 10
        dataChannels.append((self.buffer[16] >> 1 | self.buffer[17] << 7) & 0x07FF)  # Channel 11
        dataChannels.append((self.buffer[17] >> 4 | self.buffer[18] << 4) & 0x07FF)  # Channel 12
        dataChannels.append((self.buffer[18] >> 7 | self.buffer[19] << 1 | self.buffer[20] << 9) & 0x07FF)  # Channel 13
        dataChannels.append((self.buffer[20] >> 2 | self.buffer[21] << 6) & 0x07FF)  # Channel 14
        dataChannels.append((self.buffer[21] >> 5 | self.buffer[22] << 3) & 0x07FF)  # Channel 15

        self.serialData.frame_error = (self.buffer[23] & (1 << 2)) != 0
        self.serialData.failsafe = (self.buffer[23] & (1 << 3)) != 0

        return True


def main(args=None):
    rclpy.init(args=args)

    serial_rc_receiver = SerialRCReceiver()

    rclpy.spin(serial_rc_receiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_rc_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
