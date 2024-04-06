#!/usr/bin/env python3
import serial
import json
import time
import sys
from typing import Optional
import subprocess
import traceback

import rclpy
from rclpy.node import Node

from rclpy.timer import Timer
from rclpy.subscription import Subscription

from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import Wind
import signal

class AirmarReader(Node): #translates airmar data into json and publishes on 'airmar_data' ROS2 topic

    def __init__(self):
        super().__init__('airmar_reader')

        self.publisher_ = self.create_publisher(String, 'airmar_data', 10)
        self.rot_publisher = self.create_publisher(Float64, 'airmar_data/rate_of_turn', 10)
        self.navsat_publisher = self.create_publisher(NavSatFix, 'airmar_data/lat_long', 10)
        self.track_degrees_true_publisher = self.create_publisher(Float64, 'airmar_data/track_degrees_true', 10)
        self.track_degrees_magnetic_publisher = self.create_publisher(Float64, 'airmar_data/track_degrees_magnetic', 10)
        self.speed_knots_publisher = self.create_publisher(Float64, 'airmar_data/speed_knots', 10)
        self.speed_kmh_publisher = self.create_publisher(Float64, 'airmar_data/speed_kmh', 10)
        self.heading_publisher = self.create_publisher(Float64, 'airmar_data/heading', 10)
        self.true_wind_publisher = self.create_publisher(Float64, 'airmar_data/true_wind', 10)
        self.apparent_wind_publisher = self.create_publisher(Float64, 'airmar_data/apparent_wind', 10)
        self.roll_publisher = self.create_publisher(Float64, 'airmar_data/roll', 10)
        self.pitch_publisher = self.create_publisher(Float64, 'airmar_data/pitch', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.ser = serial.Serial('/dev/serial/by-id/usb-Maretron_USB100__NMEA_2000_USB_Gateway__1163885-if00')

        

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.readLineToJson())
        if msg.data == {}:
            return
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def publishIfValid(self, value, publisher, type: type):
        #self.get_logger().info("Checking validity: ")
        #self.get_logger().info(str(type))
        #self.get_logger().info(str(value))

        if type == Float64:
            msg = Float64()
            try:
                msg.data = float(value)
                self.get_logger().info(str(msg.data))
                publisher.publish(msg)
            except Exception as e:
                #self.get_logger().error(traceback.format_exc())
                return
        elif type == Wind:
            msg = Wind()
            try:
                msg.direction = float(value[0])
                msg.speed = float(value[1])
                publisher.publish(msg)

            except:
                return
        elif type == NavSatFix:
            msg = NavSatFix()
            try:
                msg.latitude = float(value[0])
                msg.longitude = float(value[1])
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                publisher.publish(msg)
                self.get_logger().info("Published latlong!")
            except:
                return
        else:
            self.get_logger().info("publishIfValid: unimplemented type")
        

    def readLineToJson(self):

        try:
            line = self.ser.readline().decode()
            #self.get_logger().info(line)
            tag = line.split(',',1)[0]
            type_code = tag[-3:]
            args = line.split(',')
            args[len(args) - 1] = args[len(args) - 1].split('*')[0] #get rid of checksum

            if(type_code == 'ROT'): #rate of turn degrees per minute. negative is to port
                self.publishIfValid(args[1], self.rot_publisher, Float64)
                return {"rate-of-turn":args[1]}
            elif(type_code == 'GLL'):
                self.get_logger().info("Got GPS data: ")
                self.get_logger().info(line)
                #convert from degree decimal minutes to decimal degrees
                #dd = d + m/60
                #lat = math.floor(float(args[1]) / 100) + (float(args[1]) % 100)/60.0
                #lon = math.floor(float(args[3]) / 100) + (float(args[3]) % 100)/60.0
                lat_raw = args[1]
                lon_raw = args[3]
                if(lat_raw=="" or lon_raw==""):
                    return {}
                lat = float(lat_raw[:2]) + float(lat_raw[2:])/60.0
                lon = float(lon_raw[:3]) + float(lon_raw[3:])/60.0
                if(args[2] == 'S'):
                    lat *= -1
                if(args[4] == 'W'):
                    lon *= -1
                self.publishIfValid([lat, lon], self.navsat_publisher, NavSatFix)
                self.get_logger().info("Publishing latlong")

                return {"Latitude":lat,
                        "Latitude-direction":args[2],
                        "Longitude":lon,
                        "Longitude-direction":args[4]}
            elif(type_code == 'VTG'):
                self.publishIfValid(args[1], self.track_degrees_true_publisher, Float64)
                self.publishIfValid(args[3], self.track_degrees_magnetic_publisher, Float64)
                self.publishIfValid(args[5], self.speed_knots_publisher, Float64)
                self.publishIfValid(args[7], self.speed_kmh_publisher, Float64)
                return {"track-degrees-true":args[1],
                        "track-degrees-magnetic":args[3],
                        "speed-knots":args[5],
                        "speed-kmh":args[7]}
            elif(type_code == 'XDR'):
                ret = {}
                if(args[4] == "ENV_OUTSIDE_T"): #in celcius
                    ret["outside-temp"] = args[2]
                elif(args[4] == "ENV_ATMOS_P"): #in ???
                    ret["atmospheric-pressure"] = args[2]
                if(len(args) > 5):
                    if(args[8] == "ENV_OUTSIDE_T"): #in celcius
                        ret["outside-temp"] = args[6]
                    elif(args[8] == "ENV_ATMOS_P"): #in ???
                        ret["atmospheric-pressure"] = args[6]

                return ret

            elif(type_code == 'HDG'):
                self.get_logger().info("Publishing heading")
                self.publishIfValid(args[1], self.heading_publisher, Float64)
                return {"currentHeading":args[1], #degrees
                        "magnetic-deviation":args[2], #degrees
                        "magnetic-deviation-direction":args[3],
                        "magnetic-variation":args[4], #degrees
                        "magnetic-variation-direction":args[5]}

            elif(type_code == 'VHW'): #water speed and direction (speed not showing up)
                return {}  #not sure we need
            elif(type_code == 'GGA'): #GPS position, quality, elevation, # of satilites
                return {} #not sure needed, repeated gps from GLL
            elif(type_code == 'DTM'): #unreferenced in documentation except as datnum reference -- unusable
                return {}
            elif(type_code == 'GSV'): #GPS satilites in view
                return {} #no need for this data
            elif(type_code == 'GSA'): # GPS Dilution of Precision
                return {} #not sure if needed
            elif(type_code == 'GRS'): #"The GRS message is used to support the Receiver Autonomous Integrity Monitoring (RAIM)." -- unneeded
                return {}
            elif(type_code == 'MWD'):
                self.get_logger().info("Got true wind!")
                self.publishIfValid([args[5], args[1]], self.true_wind_publisher, Wind)
                return {"trueWind":
                    {"speed": args[5],      #in knots
                     #"speed": args[7]      for reporting in m/s
                    "direction": args[1]   #in deg
                    }
                }
            elif(type_code == 'MWV'):
                self.publishIfValid([args[5], args[1]], self.true_wind_publisher, Wind)
                self.get_logger().info("Got apparent wind")
                return {"apparentWind":
                    {"speed": args[3],       #in knots 
                    "direction": args[1]   #in deg
                    }
                }
            elif(type_code == 'ZDA'): #date & time
                return {} # unneeded
            elif(type_code == 'OUT'): #real key is 'PMAROUT', shortened to OUT, since all others are 3 letters
                #"PGN is translated to a Maretron proprietary NMEA 0183 sentence " -- used for pitch and roll
                self.publishIfValid(args[3], self.pitch_publisher, Float64)
                self.publishIfValid(args[2], self.roll_publisher, Float64)
                return { "pitchroll":
                        {"roll":args[2],
                        "pitch":args[3]}
                        }
            else:
                pass
                #raise ValueError("Unknown NEMA code: " + type_code)
        except Exception as e:
            print(e)
            return({})


def main(args=None):

    rclpy.init(args=args)
    airmar_reader = AirmarReader()

    # Hack to reset Maretron since it's broken and gets stuck
    # command = ['echo', 'sailbot', '|', 'sudo', '-S', 'usbreset', '1576:03b1']
    # result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # if result.returncode == 0:
    #     airmar_reader.get_logger().info("Command executed successfully.")
    # else:
    #     airmar_reader.get_logger().error("Command failed.")
    #     airmar_reader.get_logger().info(result.stderr.decode())

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(airmar_reader)

    def signal_handler(sig, frame):
        airmar_reader.get_logger().info('You pressed Ctrl+C! Closing serial connection...')
        airmar_reader.ser.close()
        executor.shutdown()
        airmar_reader.destroy_node()
        sys.exit(0)
    #signal.signal(signal.SIGINT, signal_handler)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        airmar_reader.get_logger().fatal(f'Unhandled exception: {e}')
    finally:
        # Shutdown and cleanup the node
        airmar_reader.ser.close()
        executor.shutdown()
        airmar_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
