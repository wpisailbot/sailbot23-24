#!/usr/bin/env python3
import serial
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import Wind


class AirmarReader(Node): #translates airmar data into json and publishes on 'airmar_data' ROS2 topic

    def __init__(self):
        super().__init__('airmar_reader')
        self.ser = serial.Serial('/dev/serial/by-id/usb-Maretron_USB100__NMEA_2000_USB_Gateway__1170079-if00')
        self.publisher_ = self.create_publisher(String, 'airmar_data', 10)
        self.rot_publisher = self.create_publisher(Float64, 'airmar_data/rate_of_turn', 10)
        self.navsat_publisher = self.create_publisher(NavSatFix, 'airmar_data/lat_long', 10)
        self.track_degrees_true_publisher = self.create_publisher(Float64, 'airmar_data/track_degrees_true', 10)
        self.track_degrees_magnetic_publisher = self.create_publisher(Float64, 'airmar_data/track_degrees_magnetic', 10)
        self.speek_knots_publisher = self.create_publisher(Float64, 'airmar_data/speed_knots', 10)
        self.speek_kmh_publisher = self.create_publisher(Float64, 'airmar_data/speed_kmh', 10)
        self.heading_publisher = self.create_publisher(Float64, 'airmar_data/heading', 10)
        self.true_wind_publisher = self.create_publisher(Float64, 'airmar_data/true_wind', 10)
        self.apparent_wind_publisher = self.create_publisher(Float64, 'airmar_data/apparent_wind', 10)
        self.roll_publisher = self.create_publisher(Float64, 'airmar_data/roll', 10)
        self.pitch_publisher = self.create_publisher(Float64, 'airmar_data/pitch', 10)


        #timer_period = 0.01  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.readLineToJson())
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def readLineToJson(self):

        try:
            line = self.ser.readline().decode()
            tag = line.split(',',1)[0]
            type_code = tag[-3:]
            args = line.split(',')
            args[len(args) - 1] = args[len(args) - 1].split('*')[0] #get rid of checksum

            if(type_code == 'ROT'): #rate of turn degrees per minute. negative is to port
                rot_msg = Float64()
                rot_msg.data = args[1]
                self.rot_publisher.publish(rot_msg)
                return {"rate-of-turn":args[1]}
            elif(type_code == 'GLL'):
                #convert from degree decimal minutes to decimal degrees
                #dd = d + m/60
                #lat = math.floor(float(args[1]) / 100) + (float(args[1]) % 100)/60.0
                #lon = math.floor(float(args[3]) / 100) + (float(args[3]) % 100)/60.0
                lat_raw = args[1]
                lat = float(lat_raw[:2]) + float(lat_raw[2:])/60.0
                lon_raw = args[3]
                lon = float(lon_raw[:3]) + float(lon_raw[3:])/60.0
                if(args[2] == 'S'):
                    lat *= -1
                if(args[4] == 'W'):
                    lon *= -1
                nav_sat_msg = NavSatFix()
                nav_sat_msg.latitude = lat
                nav_sat_msg.longitude = lon
                nav_sat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.navsat_publisher.publish(nav_sat_msg)
                self.get_logger().info("Publishing latlong")

                return {"Latitude":lat,
                        "Latitude-direction":args[2],
                        "Longitude":lon,
                        "Longitude-direction":args[4]}
            elif(type_code == 'VTG'):
                track_true_msg = Float64()
                track_true_msg.data = args[1]
                self.track_degrees_true_publisher.publish(track_true_msg)
                track_mag_msg = Float64()
                track_mag_msg.data = args[3]
                self.track_degrees_magnetic_publisher.publish(track_mag_msg)
                speed_knots_msg = Float64()
                speed_knots_msg = args[5]
                self.speek_knots_publisher.publish(speed_knots_msg)
                speed_kmh_msg = Float64()
                speed_kmh_msg = args[7]
                self.speek_kmh_publisher.publish(speed_kmh_msg)
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
                heading_msg = Float64()
                heading_msg.data = args[1]
                self.heading_publisher.publish(heading_msg)
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
                true_wind_msg = Wind()
                true_wind_msg.speed.data = args[5]
                true_wind_msg.direction.data = args[1]
                self.true_wind_publisher.publish(true_wind_msg)
                return {"trueWind":
                    {"speed": args[5],      #in knots
                     #"speed": args[7]      for reporting in m/s
                    "direction": args[1]   #in deg
                    }
                }
            elif(type_code == 'MWV'):
                apparent_wind_msg = Wind()
                apparent_wind_msg.speed.data = args[3]
                apparent_wind_msg.direction.data = args[1]
                self.apparent_wind_publisher.publish(apparent_wind_msg)
                return {"apparentWind":
                    {"speed": args[3],       #in knots 
                    "direction": args[1]   #in deg
                    }
                }
            elif(type_code == 'ZDA'): #date & time
                return {} # unneeded
            elif(type_code == 'OUT'): #real key is 'PMAROUT', shortened to OUT, since all others are 3 letters
                #"PGN is translated to a Maretron proprietary NMEA 0183 sentence " -- used for pitch and roll
                pitch_msg = Float64()
                pitch_msg.data = args[3]
                self.pitch_publisher.publish(pitch_msg)
                roll_msg = Float64()
                roll_msg.data = args[2]
                self.roll_publisher.publish(roll_msg)
                return { "pitchroll":
                        {"roll":args[2],
                        "pitch":args[3]}
                        }
            else:
                raise ValueError("Unknown NEMA code: " + type_code)
        except Exception as e:
            print(e)
            return({})


def main(args=None):
    rclpy.init(args=args)

    airmar_reader = AirmarReader()
    while( rclpy.ok() ):
        rclpy.spin_once(airmar_reader, timeout_sec=.001)
        airmar_reader.timer_callback()
    #rclpy.spin(airmar_reader)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airmar_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
