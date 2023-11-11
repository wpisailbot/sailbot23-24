#!/usr/bin/env python3
import json
import rclpy
import RPi.GPIO as GPIO
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from std_msgs.msg import String, Empty

import smbus2 as smbus
import math
import time


# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __MODE2              = 0x01
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD


  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)
	
  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
	  
  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result
	
  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)
    self.write(self.__MODE2, 0x04)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))
	  
  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))
    
  def setRotationAngle(self, channel, Angle): 
    if(Angle >= 0 and Angle <= 180):
        temp = Angle * (2000 / 180) + 501
        self.setServoPulse(channel, temp)
    else:
        print("Angle out of range")
    
  def exit_PCA9685(self):
    self.write(self.__MODE2, 0x00)

class PWMController(LifecycleNode):

    def __init__(self):
        super().__init__('pwm_controller')

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")
        self.subscription = self.create_subscription(
            String,
            'pwm_control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer_pub = self.create_lifecycle_publisher(Empty, '/heartbeat/pwm_controller', 1)
        self.timer = self.create_timer(0.5, self.heartbeat_timer_callback)
        up = False
        while not up:
          try:
            self.pwm = PCA9685()
            up = True
          except:
            self.get_logger().info("PWM controller error! Is PWM HAT connected?")
            time.sleep(1)
        self.pwm.setPWMFreq(50)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.timer_pub)
        self.destroy_subscription(self.subscription)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Error caught!")
        return super().on_error(state)
    
    #end callbacks
        
    def heartbeat_timer_callback(self):
      #self.get_logger().info("Publishing timer")
      self.timer_pub.publish(Empty())
       

    def listener_callback(self, msg):
        #self.get_logger().info('PWM command received: "%s"' % msg.data)
        self.execute_pwm(msg)

    def execute_pwm(self, msg):
        jmsg = json.loads(str(msg.data))
        #print("channel: " + str(jmsg['channel']) + ", angle: " + str(jmsg['angle']))
        self.pwm.setRotationAngle(int(jmsg['channel']), int(jmsg['angle']))


def main(args=None):
    rclpy.init(args=args)
    pwm_controller = PWMController()

    rclpy.spin(pwm_controller)

    # exit pwm
    pwm_controller.pwm.exit_PCA9685()
    GPIO.cleanup()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pwm_controller.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

