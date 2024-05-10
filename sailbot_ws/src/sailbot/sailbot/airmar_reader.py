#!/usr/bin/env python3
import serial
import json
import time
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription

from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import Wind
import signal
import logging
import traceback


class AirmarReader(LifecycleNode): #translates airmar data into json and publishes on 'airmar_data' ROS2 topic

    def __init__(self):
        super().__init__('airmar_reader')
        self.publisher_: Optional[Publisher]
        self.rot_publisher: Optional[Publisher]
        self.navsat_publisher: Optional[Publisher]
        self.track_degrees_true_publisher: Optional[Publisher]
        self.track_degrees_magnetic_publisher: Optional[Publisher]
        self.speed_knots_publisher: Optional[Publisher]
        self.speed_kmh_publisher: Optional[Publisher]
        self.heading_publisher: Optional[Publisher]
        self.true_wind_publisher: Optional[Publisher]
        self.apparent_wind_publisher: Optional[Publisher]
        self.roll_publisher: Optional[Publisher]
        self.pitch_publisher: Optional[Publisher]


    #lifecycle node callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        try:
            port = '/dev/serial/by-id/usb-Maretron_USB100__NMEA_2000_USB_Gateway__1163885-if00'
            # try:
            #     # Attempt to open and immediately close the port
            #     ser = serial.Serial(port)
            #     ser.close()
            # except serial.SerialException as e:
            #     print(f"Failed to reset port: {e}")
            self.ser = serial.Serial(port)
        except:
            return TransitionCallbackReturn.FAILURE
        self.publisher_ = self.create_lifecycle_publisher(String, 'airmar_data', 10)
        self.rot_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/rate_of_turn', 10)
        self.navsat_publisher = self.create_lifecycle_publisher(NavSatFix, 'airmar_data/lat_long', 10)
        self.track_degrees_true_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/track_degrees_true', 10)
        self.track_degrees_magnetic_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/track_degrees_magnetic', 10)
        self.speed_knots_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/speed_knots', 10)
        self.speed_kmh_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/speed_kmh', 10)
        self.heading_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/heading', 10)
        self.true_wind_publisher = self.create_lifecycle_publisher(Wind, 'airmar_data/true_wind', 10)
        self.apparent_wind_publisher = self.create_lifecycle_publisher(Wind, 'airmar_data/apparent_wind', 10)
        self.roll_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/roll', 10)
        self.pitch_publisher = self.create_lifecycle_publisher(Float64, 'airmar_data/pitch', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_lifecycle_publisher(self.publisher_)
        self.destroy_lifecycle_publisher(self.rot_publisher)
        self.destroy_lifecycle_publisher(self.navsat_publisher)
        self.destroy_lifecycle_publisher(self.track_degrees_true_publisher)
        self.destroy_lifecycle_publisher(self.track_degrees_magnetic_publisher)
        self.destroy_lifecycle_publisher(self.speed_knots_publisher)
        self.destroy_lifecycle_publisher(self.speed_kmh_publisher)
        self.destroy_lifecycle_publisher(self.heading_publisher)
        self.destroy_lifecycle_publisher(self.true_wind_publisher)
        self.destroy_lifecycle_publisher(self.apparent_wind_publisher)
        self.destroy_lifecycle_publisher(self.roll_publisher)
        self.destroy_lifecycle_publisher(self.pitch_publisher)
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.readLineToJson())
        if msg.data == {}:
            return
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def publishIfValid(self, value, publisher, type: type):
        """
        Validates and publishes the given value using the specified publisher based on the data type. This function is designed
        to ensure that only valid data is published to ROS topics.

        :param value: The value to be published. Depending on the type, this can be a single value or a tuple of values.
        :param publisher: The ROS publisher object used to publish the data.
        :param type: The data type class which defines how to interpret `value`. This can be one of several expected ROS message types
                    such as `Float64`, `Wind`, or `NavSatFix`.

        :return: None. This function does not return a value but instead publishes data to a ROS topic if the data is valid.

        Function behavior includes:
        - Checking the specified `type` and constructing a corresponding ROS message object.
        - Attempting to cast `value` to the appropriate type(s) required by the ROS message.
        - If casting is successful and the value is valid, the data is published using the provided `publisher`.
        - If an error occurs during casting or validation, the function catches the exception and refrains from publishing,
        optionally logging the error or ignoring invalid data.

        This function supports multiple ROS data types and can be extended to include more types as needed.
        """
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
                msg.speed = float(value[0])
                msg.direction = float(value[1])
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
        """
        Reads a line from a serial input, decodes it, and interprets various NMEA sentences into structured JSON-like dictionaries
        based on the sentence type. The function handles various NMEA sentence codes by parsing and publishing appropriate data
        while logging relevant information.

        :return: A dictionary representing the parsed data from the NMEA sentence. This dictionary's structure varies depending
                on the type of NMEA sentence processed. The function returns an empty dictionary for certain NMEA sentence types
                that are deemed unnecessary or redundant.

        This function primarily processes data for navigation and environmental monitoring, converting raw NMEA sentence inputs
        into more structured data forms. It handles a variety of NMEA sentence types including GPS position data (`GLL`),
        rate of turn (`ROT`), speed and heading information (`VTG`), environmental data like temperature and atmospheric pressure (`XDR`),
        and wind data (`MWV`, `MWD`). It also includes error handling to manage exceptions during the read and parse operations.

        Function behavior includes:
        - Reading and decoding a line from the serial port.
        - Splitting the line based on commas to differentiate data fields and removing any checksums.
        - Identifying the sentence type through a code in the tag and processing accordingly.
        - Publishing valid data to appropriate ROS topics using helper functions.
        - Logging significant actions and data points for debugging purposes.
        - Returning structured data as a dictionary where applicable, or an empty dictionary for unsupported or unnecessary data types.

        This function assumes the availability of a serial port connection, currently to a Maretron USB100.
        """
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
                self.publishIfValid([args[3], args[1]], self.apparent_wind_publisher, Wind)
                self.get_logger().info(f"Got apparent wind: {args[3]}, {args[1]}")
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
    airmar_reader.get_logger().set_level(logging.WARNING)

    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(airmar_reader)

    # def signal_handler(sig, frame):
    #     airmar_reader.get_logger().info('You pressed Ctrl+C! Closing serial connection...')
    #     airmar_reader.ser.close()
    # signal.signal(signal.SIGINT, signal_handler)

    try:
        # Spin the node to execute callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        trace = traceback.format_exc()
        airmar_reader.get_logger().fatal(f'Unhandled exception: {e}\n{trace}')
    finally:
        # Shutdown and cleanup the node
        airmar_reader.ser.close()
        executor.shutdown()
        airmar_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
