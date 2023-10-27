#!/usr/bin/env python3
from time import time
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Float32, Int8, Int16
import sailbot.autonomous.p2p as p2p
from collections import deque


class ControlSystem(Node):  # Gathers data from some nodes and distributes it to others

    def __init__(self):
        super().__init__('control_system')
        # Create subcription to serial_rc topic
        self.serial_rc_subscription = self.create_subscription(
            String,
            'serial_rc',
            self.serial_rc_listener_callback,
            10)
        self.serial_rc_subscription
        
        # Create subscription to airmar_data
        self.airmar_data_subscription = self.create_subscription(
            String,
            'airmar_data',
            self.airmar_data_listener_callback,
            10)
        self.airmar_data_subscription
        
        # Create subscription to tt_telemetry
        self.trim_tab_telemetry_subscription = self.create_subscription(
            Float32,
            'tt_telemetry',
            self.trim_tab_telemetry_listener_callback,
            10)
        self.trim_tab_telemetry_subscription
        
        # Create publisher to pwm_control
        self.pwm_control_publisher_ = self.create_publisher(String, 'pwm_control', 10)

        # Create publisher to trim_tab_control
        self.trim_tab_control_publisher_ = self.create_publisher(Int8, 'tt_control', 10)
        self.trim_tab_angle_publisher_ = self.create_publisher(Int16, 'tt_angle', 10)

        # Create publisher to ballast_algorithnm_debug
        self.ballast_algorithnm_debug_publisher_ = self.create_publisher(String, 'ballast_algorithnm_debug', 10)

        # Create instance vars for subscribed topics to update
        self.serial_rc = {}
        self.airmar_data = {}
        self.trim_tab_status = {}

        # Create instance var for keeping queue of wind data
        self.lastWinds = []
        self.p2p_alg = None

        # Create instance var for keeping queue of roll data
        self.omega = deque(maxlen=4)
        self.alpha = deque(maxlen=3)
        self.lastRollAngle = deque(maxlen=4)
        # self.p2p_alg = None
        

    def serial_rc_listener_callback(self, msg):
        #self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.serial_rc[i] = msg_dict[i]
        
    def airmar_data_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.airmar_data[i] = msg_dict[i]
        
    def trim_tab_telemetry_listener_callback(self, msg):
        #self.get_logger().info('Received msg: "%s"' % msg.data)

        try:
            self.trim_tab_status['wind_dir'] = msg.data
        except Exception as e:
            self.get_logger().error(str(e))

    def update_winds(self, relative_wind):
        # Check we have new wind
        if len(self.lastWinds) != 0 and relative_wind == self.lastWinds[len(self.lastWinds) -1]:
            return       
        # First add wind to running list
        self.lastWinds.append(float(relative_wind))
        if len(self.lastWinds) > 10:
            self.lastWinds.pop(0)
        # Now find best trim tab state
        smooth_angle = self.median(self.lastWinds)
        return smooth_angle

    def find_trim_tab_state(self, relative_wind):           #five states of trim
        smooth_angle = self.update_winds(relative_wind)
        msg = Int8()
        if 45.0 <= smooth_angle < 135:
            # Max lift port
            msg.data = (0)
        elif 135 <= smooth_angle < 180:
            # Max drag port
            msg.data = (2)
        elif 180 <= smooth_angle < 225:
            # Max drag starboard
            msg.data = (3)
        elif 225 <= smooth_angle < 315:
            # Max lift starboard
            msg.data = (1)
        else:
            # In irons, min lift
            msg.data = (4)

        
        self.trim_tab_control_publisher_.publish(msg)
            
    def make_json_string(self, json_msg):
        json_str = json.dumps(json_msg)
        message = String()
        message.data = json_str
        return message
           
    def median(self, lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n//2-1:n//2+1])/2.0, s[n//2])[n % 2] if n else None

    def ballast_algorithm(self):
        # Check wind angle, then check current tilt of boat, then adjust ballast accordingly
        if len(self.lastWinds) == 0:
            return
        self.lastRollAngle.append(self.airmar_data["roll"])
        smooth_angle = self.median(self.lastWinds)
        ballast_angle = 0
        #print("roll:" + self.airmar_data["roll"])
        delta = self.airmar_data["roll"] - self.lastRollAngle[-1]
        

        timeDifference = .5     #hypothetically - see main
        omega_n = delta/timeDifference
        self.omega.append(omega_n)
        alpha_n = self.omega[-1]/timeDifference
        self.alpha.append(alpha_n)
        #-- Logging ----------------
        self.ballast_algorithnm_debug_publisher_.publish("omega: " + str(omega_n) + " -- " + "alpha / acceleration: " + str(alpha_n) + "\n")
        #Account for a heavy tilt
        
         #-----------
         # Starboard tack
        if 0 < smooth_angle <= 180: 
            # Go for 20 degrees
            if float(self.airmar_data["roll"]) > -12:           #change to roll acc.
                #ballast_angle = 110
                ballast_angle = omega_n * 2
            elif float(self.airmar_data["roll"]) < -20:         #change to roll acc.
                ballast_angle = 80
                #-----------
        # Port tack
        elif 180 < smooth_angle < 360:  
            if float(self.airmar_data["roll"]) < 12:
                ballast_angle = 80
            elif float(self.airmar_data["roll"]) > 20:
                ballast_angle = 110

        ballast_json = {"channel": "12", "angle": ballast_angle}
        self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))

           
def main(args=None):
    rclpy.init(args=args)

    control_system = ControlSystem()
    
    while rclpy.ok():
        
        rclpy.spin_once(control_system, timeout_sec=.5)
        # Now we have new vals from subscribers in:
        # control_system.serial_rc
        # control_system.airmar_data
        # control_system.trim_tab_status

        # Need to publish new values to both control topics based on new values
        # control_system.pwm_control_publisher_.publish()       <----- i think both of these are notes from last year and have since been implemented
        # control_system.trim_tab_control_publisher_.publish()  <----- i think both of these are notes from last year and have since been implemented

        #TODO ^^implement
        
        if len(control_system.serial_rc) < 2:
            pass # Don't have rc values
        elif float(control_system.serial_rc["state2"]) > 600:  # in RC
            if float(control_system.serial_rc["state1"]) < 400:
                # Manual
                manual_angle = int((float(control_system.serial_rc["manual"]) / 2000) * 100) + 65
                state_msg = Int8()
                state_msg.data = 5
                angle_msg = Int16()
                angle_msg.data = manual_angle
                control_system.trim_tab_control_publisher_.publish(state_msg)
                control_system.trim_tab_angle_publisher_.publish(angle_msg)
            elif "wind-angle-relative" in control_system.airmar_data:
                # print(control_system.airmar_data["wind-angle-relative"])
                try:
                    control_system.find_trim_tab_state(control_system.airmar_data["apparentWind"]["direction"])
                except Exception as e:
                    control_system.get_logger().error(str(e))
            else:
                print("No wind angle values")
            if float(control_system.serial_rc["state1"]) < 800:
                ballast_angle = 0
                if control_system.serial_rc["ballast"] > 1200:
                    ballast_angle = 110
                elif control_system.serial_rc["ballast"] < 800:
                    ballast_angle = 80
                ballast_json = {"channel" : "12", "angle" : ballast_angle}
                control_system.pwm_control_publisher_.publish(control_system.make_json_string(ballast_json))
            else:
                control_system.ballast_algorithm()
            rudder_angle = (float(control_system.serial_rc["rudder"]) / 2000 * 90) + 25
            rudder_json = {"channel": "8", "angle": rudder_angle}
            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))
        elif float(control_system.serial_rc["state2"]) < 600:
            destinations = [(42.277055,-71.799924),(42.276692,-71.799912)] 
            if 'Latitude' in control_system.airmar_data and 'Longitude' in control_system.airmar_data:
                try:
                    if control_system.p2p_alg is None:  # Instantiate new
                        control_system.p2p_alg = p2p.P2P((float(control_system.airmar_data['Latitude']), float(control_system.airmar_data['Longitude'])), destinations[0])
                    wind = control_system.update_winds(control_system.airmar_data["apparentWind"]["direction"])
                    action = control_system.p2p_alg.getAction(wind, float(control_system.airmar_data["magnetic-sensor-heading"]), float(control_system.airmar_data["track-degrees-true"]))
                    control_system.get_logger().error(str(control_system.p2p_alg.getdistance()))
                    control_system.get_logger().error(str(action))
                    if action['status'] == 'DONE':
                        if control_system.p2p_alg.dest == destinations[0]:
                            control_system.p2p_alg = p2p.P2P((control_system.airmar_data['Latitude'], control_system.airmar_data['Longitude']), destinations[1])
                        else:
                            control_system.p2p_alg = p2p.P2P((control_system.airmar_data['Latitude'], control_system.airmar_data['Longitude']), destinations[0])
                    else:  # We have a non-done action (either trim tab or rudders)
                        if 'tt-state' in action:
                            control_system.trim_tab_control_publisher_.publish(int(action['tt-state']))
                        elif 'rudder-angle' in action:
                            rudder_json = {"channel": "8", "angle": int(action['rudder-angle'])}
                            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))
                        control_system.ballast_algorithm()
                except Exception as e:
                    control_system.get_logger().error(str(e))
            else:
                control_system.get_logger().error("No latitude and longitude data")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
