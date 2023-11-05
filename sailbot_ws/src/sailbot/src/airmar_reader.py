#!/usr/bin/env python3
import serial
import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class AirmarReader(Node): #translates airmar data into json and publishes on 'airmar_data' ROS2 topic

    def __init__(self):
        super().__init__('airmar_reader')
        self.ser = serial.Serial('/dev/serial/by-id/usb-Maretron_USB100__NMEA_2000_USB_Gateway__1170079-if00')
        self.publisher_ = self.create_publisher(String, 'airmar_data', 10)
        #timer_period = 0.01  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.readLineToJson())
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def readLineToJson(self):

        try:
            line = self.ser.readline().decode()
            tag = line.split(',',1)[0]
            type_code = tag[-3:]
            args = line.split(',')
            args[len(args) - 1] = args[len(args) - 1].split('*')[0] #get rid of checksum

            if(type_code == 'ROT'): #rate of turn degrees per minute. negative is to port
                return {"rate-of-turn":args[1]}
            elif(type_code == 'GLL'):
                #convert from degree decimal minutes to decimal degrees
                #dd = d + m/60
                #lat = math.floor(float(args[1]) / 100) + (float(args[1]) % 100)/60.0
                #lon = math.floor(float(args[3]) / 100) + (float(args[3]) % 100)/60.0
                lat_raw = args[1]3
                lat = float(lat_raw[:2]) + float(lat_raw[2:])/60.0
                lon_raw = args[3]
                lon = float(lon_raw[:3]) + float(lon_raw[3:])/60.0
                if(args[2] == 'S'):
                    lat *= -1
                if(args[4] == 'W'):
                    lon *= -1

                return {"Latitude":lat,
                        "Latitude-direction":args[2],
                        "Longitude":lon,
                        "Longitude-direction":args[4]}
            elif(type_code == 'VTG'):
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
                return {"trueWind":
                    {"speed": args[5],      #in knots
                     #"speed": args[7]      for reporting in m/s
                    "direction": args[1]   #in deg
                    }
                }
            elif(type_code == 'MWV'):
                return {"apparentWind":
                    {"speed": args[3],       #in knots 
                    "direction": args[1]   #in deg
                    }
                }
            elif(type_code == 'ZDA'): #date & time
                return {} # unneeded
            elif(type_code == 'OUT'): #real key is 'PMAROUT', shortened to OUT, since all others are 3 letters
                #"PGN is translated to a Maretron proprietary NMEA 0183 sentence " -- used for pitch and roll
                return { "pitchroll":
                        {"roll":args[2],
                        "pitch":args[3]}
                        }
            else:
                raise ValueError("Unknown NEMA code: " + type_code)
        except Exception as e:
            print(e);
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
