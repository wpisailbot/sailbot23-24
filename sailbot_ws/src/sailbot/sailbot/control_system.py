import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Float32, Int8, Int16, Int32
from collections import deque
import numpy as np
import math
from collections import deque # TODO: replace the queue for waypoints with a more traditional python queue


class ControlSystem(Node):  # Gathers data from some nodes and distributes it to others

    def __init__(self):
        super().__init__('control_system')
        # Create subscription to serial_rc topic
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

        self.ballast_adc_subscription = self.create_subscription(
            Float32,
            'ballast_adc_vals',
            self.ballast_adc_listener_callback,
            10)
        self.ballast_adc_subscription

        self.heading_adc_subscription = self.create_subscription(
            Float32,
            'heading_adc_publisher',
            self.heading_adc_listener_callback,
            10)
        self.heading_adc_subscription

        self.cv_subscription = self.create_subscription(
            Int32,
            'neon_orange_detected',
            self.cv_listener_callback,
            10)
        self.cv_subscription

        # Create publisher to pwm_control
        self.pwm_control_publisher_ = self.create_publisher(String, 'pwm_control', 10)

        # Create publisher to trim_tab_control
        self.trim_tab_control_publisher_ = self.create_publisher(Int8, 'tt_control', 10)
        self.trim_tab_angle_publisher_ = self.create_publisher(Int16, 'tt_angle', 10)

        # Create publisher to ballast_algorithm_debug
        self.ballast_algorithm_debug_publisher_ = self.create_publisher(String, 'ballast_algorithm_debug', 10)

        # Create instance vars for subscribed topics to update
        self.serial_rc = {}
        self.airmar_data = {}
        self.trim_tab_status = {}
        self.ballast_adc_value = 0.5
        self.heading_adc_value = 0
        self.cv = 0

        # Create instance var for keeping queue of wind data
        self.lastWinds = []

        # Create instance var for keeping queue of roll data
        self.omega = deque(maxlen=4)
        self.alpha = deque(maxlen=3)
        self.lastRollAngle = deque(maxlen=4)

        # Create global variables for autonomous algorithm
        self.minSailingAngle = np.pi / 4  # minimum sailing angle for the boat
        self.dp_min = -np.cos(self.minSailingAngle)  # minimum dot product for the boat's sailing angle
        """
        # max cross-track error, in meters, that we will allow for the boat when going directly 
        # upwind (from centerline! if you want the boat to wander from -10 to +10m off course, 
        # put in 10!) 
        """
        self.max_cte = 20

        # autonomous global variables, position and velocity of objects
        self.boat = np.array([0, 0])  # boat position
        self.wind = np.array([1, 0])  # wind in x and y velocity (m/s)
        self.windDir = self.wind / np.linalg.norm(self.wind)  # normalize the wind vector

        # # need to feed new values
        # # test for
        # self.goal = np.array([4227.43991, 7180.57987])  # goal position
        self.accuracyCounter = 0
        self.goals_queue = deque([
            np.array([4284.5627, -7097.7018]),  # initial goal position
            np.array([4284.5618, -7097.6987]),  # additional goal positions
            np.array([4284.5141, -7097.6801]),
            np.array([4284.5144, -7097.6596]),
            np.array([4284.5810, -7097.6211]),
            np.array([4284.5923, -7097.6386]),
            np.array([4284.5547, -7097.6768]),
            np.array([4284.5618, -7097.6987]),
            np.array([4284.5627, -7097.7018]),
            np.array([4284.5677, -7097.7100]),
            # add more goal positions as needed
        ])
        self.goal = self.goals_queue.popleft()  # get the first goal position

        # information stored on the boat's current heading
        self.onAB = False  # whether the boat is currently trying to sail a course on either side of the no-go zone
        self.onA = False
        self.onB = False

        # information stored for the ballast alg
        # Replace angle with encoder positions or do conversions after the fact
        self.prevErrors = [0, 0, 0, 0, 0]
        self.errorSum = 0
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.1
        self.activeKp = 0.5
        self.port_roll = None  # need info on how to track
        self.adc = 0.48

    def checkDesiredRoll(self):
        # Check wind angle, then check current tilt of boat, then adjust ballast accordingly
        self.lastRollAngle.append(self.airmar_data["pitchroll"]["roll"])
        smooth_angle = self.median(self.lastWinds)
        if 0 < smooth_angle <= 180:  # starboard tack
            return True
        else:
            return False  # port tack
        # self.get_logger().info("roll:" + self.airmar_data["roll"])
        # delta = self.airmar_data["roll"] - self.lastRollAngle[-1]

    # unlike the passive alg, this is designed to aim for the +/-20 degree angle at all times
    def ballast_alg_active(self):
        # NOTE: max port ADC values for ballast is 0.16; starboard is 0.79; midship is 0.48
        # True or False depending on whether we want to lean to the left/port (true) or right/starboard (
        # false)
        my_string = '-'.join(str(e) for e in self.lastWinds)
        self.get_logger().error(my_string)
        if len(self.lastWinds) == 0:
            return  # failsafe if we have received no data on wind to prevent crash
        else:
            self.port_roll = self.checkDesiredRoll()

        self.get_logger().error("within ballast alg")

        if self.port_roll:  # if we are leaning port
            roll_error = 25 + float(self.airmar_data["pitchroll"]["roll"])  # -20 is our desired goal
        else:  # if we are leaning starboard
            roll_error = -25 + float(self.airmar_data["pitchroll"]["roll"])  # 20 is our desired goal

        # make more efficient with rolling overwrite # used in integral error and derivative error
        # prevErrors[4] = prevErrors[3]
        # prevErrors[3] = prevErrors[2]
        # prevErrors[2] = prevErrors[1]
        # prevErrors[1] = prevErrors[0]
        # prevErrors[0] = adcError

        if roll_error > 1 or roll_error < -1:  # if the ballast needs to move...
            # integralAngleError = sum(prevErrors) derivativeAngleError = (adcError - prevErrors[1])*0.01 # Change
            # 0.01 to avg time between changes [or just delete derivative lol]

            error_sum = roll_error * self.activeKp  # + integralAngleError * ki #+ derivativeAngleError * kd

            if -3 < roll_error < 3:  # if the ballast is close to its goal and needs to slow down (-2/2 was chosen
                # arbitrarily)
                error_sum /= (1.0 + roll_error / 2.0)  # this will linearly decrease the speed at which the ballast
                # is moving based on how close the ballast is to its goal

            # translate the PID error into our output range; the motor accepts 60-130, with 60
            # being full tilt port and 130 being full tilt starboard; 95 is the median value
            ballast_speed = 95
            ballast_speed += (error_sum * 1.75)  # the max error_sum value can be is ~20, so this means the max
            # speed can be approximately at 60 (or 130)

            # this is to ensure the rail will not go off the edge
            if (self.ballast_adc_value > 0.25) and ballast_speed > 95:
                self.get_logger().error("trying to move")
                ballast_json = {"channel": "12", "angle": ballast_speed}  # create a json file to send to the motor
                self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))  # publish the json
                return

            # this is to ensure the rail will not go off the edge
            if (self.ballast_adc_value < 0.75) and ballast_speed < 95:
                self.get_logger().error("trying to move")
                ballast_json = {"channel": "12", "angle": ballast_speed}  # create a json file to send to the motor
                self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))  # publish the json
                return

            # ...otherwise, if we want the ballast to stay still:
            else:
                self.get_logger().error("staying in middle")
                ballast_json = {"channel": "12", "angle": 0}
                # despite being outside the range of 60-130, sending 0 stops the ballast motor for some reason
                self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))
                return
        # this should not happen
        self.get_logger().error("you should not see this")
        ballast_json = {"channel": "12", "angle": 0}
        self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))
        return

    def calc_heading(self):  # calculate the heading we should follow

        # calculate BG (boat to goal vector)
        BGraw = np.subtract(self.goal, self.boat)  # absolute distance to goal
        BG = BGraw / np.linalg.norm(BGraw)  # convert to a unit vector

        # compute the dot product of BG and windDir to see if we can sail directly towards the goal

        Dp = np.dot(BG, self.windDir)

        # self.get_logger().error("dot product of direction to goal and the wind is:")
        # self.get_logger().error(str(Dp))

        if Dp > self.dp_min:
            # if current Dp is less than Dpmin, we can sail directly at the goal. Easy-peasy!

            self.get_logger().error("trying to sail directly at the goal")

            self.onAB = False  # we are NOT sailing on the edge of the no-go zone if we sail directly at the goal
            return BG  # return desired heading

        else:
            # if we can't sail directly at the goal we will have to decide how to tack however, if we're already on
            # an upwind course that's not directly at the goal, we'd like to continue on that course unless our
            # crosstrack error is too high

            self.get_logger().error("we cannot sail directly at the goal - calculating best heading")

            # checking if our cross-track error is too high requires knowing the vectors A and B, so we'll start with
            # that: A and B are the vectors that lie on the edge of the no-go zone, so we'll just rotate the upwind
            # direction by + and - theta, where theta is the minimum sailing angle

            # rotation matrix for minimum sailing angle:

            c, s = np.cos(self.minSailingAngle), np.sin(self.minSailingAngle)

            R = np.array(((c, -s), (s, c)))  # rotation matrices
            R2 = np.array(((c, s), (-s, c)))

            # multiply the matrices by the wind - MUST be the upwind direction! (which is just -windDir)

            A = np.matmul(-self.windDir, R)
            B = np.matmul(-self.windDir, R2)

            # now that we have A and B, we can find which points more in the direction we want to go
            ADBG = np.dot(A, BG)  # dot product tells us which vector is pointing more towards the goal
            BDBG = np.dot(B, BG)

            if not self.onAB:  # if we're not on a heading A or B, but we aren't sailing directly at the goal,
                # we need to start moving on A or B.

                if ADBG > BDBG:  # return whichever heading A or B points more towards the goal
                    self.onAB = True
                    self.onA = True
                    self.onB = False
                    return A
                else:
                    self.onAB = True
                    self.onA = False
                    self.onB = True
                    return B

            else:  # if we're on a heading A or B, we only want to change heading if we've accumulated too much
                # cross-track error

                cte_threshold = np.cos(self.minSailingAngle - np.arcsin(self.max_cte / np.linalg.norm(BGraw)))

                self.get_logger().error("cte_threshold is:")
                self.get_logger().error(str(cte_threshold))

                self.get_logger().error("A dot BG is:")
                self.get_logger().error(str(ADBG))

                self.get_logger().error("B dot BG is:")
                self.get_logger().error(str(BDBG))

                if BDBG > cte_threshold:
                    self.onAB = True
                    self.onA = False
                    self.onB = True
                    return B

                if ADBG > cte_threshold:
                    self.onAB = True
                    self.onA = True
                    self.onB = False
                    return A

                # if neither of the above statements evaluate to true we should just keep following whatever path we
                # were already trying to follow

                if self.onA:
                    self.onAB = True
                    self.onA = True
                    self.onB = False
                    return A
                if self.onB:
                    self.onAB = True
                    self.onA = False
                    self.onB = True
                    return B

        # this should not ever return
        self.get_logger().error("you shouldn't be seeing this")
        return np.array(1, 1)

    def unit_circle_to_heading_gps(self, x, y):
        """Convert unit circle (x, y) coordinates to heading angle in degrees."""
        angle = -math.degrees(math.atan2(y, x)) + 90
        return angle + 360 if angle < 0 else angle

    def serial_rc_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.serial_rc[i] = msg_dict[i]

    def airmar_data_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.airmar_data[i] = msg_dict[i]

    def ballast_adc_listener_callback(self, msg):
        self.ballast_adc_value = msg.data
        self.get_logger().error('Received ballast: "%s"' % msg.data)

    def heading_adc_listener_callback(self, msg):
        self.heading_adc_value = msg.data
        self.get_logger().error('Received heading: "%s"' % msg.data)

    def cv_listener_callback(self, msg):
        self.cv = 0
        self.get_logger().error('Received cv: "%s"' % msg.data)

    def trim_tab_telemetry_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)

        try:
            self.trim_tab_status['wind_dir'] = msg.data
        except Exception as e:
            self.get_logger().error(str(e))

    def update_winds(self, relative_wind):
        # Check we have new wind
        if len(self.lastWinds) != 0 and relative_wind == self.lastWinds[len(self.lastWinds) - 1]:
            return
            # First add wind to running list
        self.lastWinds.append(float(relative_wind))
        if len(self.lastWinds) > 10:
            self.lastWinds.pop(0)
        # Now find best trim tab state
        smooth_angle = self.median(self.lastWinds)
        return smooth_angle

    def find_trim_tab_state(self, relative_wind):  # five states of trim
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
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None


    def distance_to_goal(self):
        # Convert latitude and longitude from degrees to radians

        # tempLat1 = self.goal[0]
        # tempLon1 = self.goal[1]
        # tempLat2 = self.boat[0]
        # tempLon2 = self.boat[1]
    
        lat1 = math.radians(self.goal[0]/100.0)
        lon1 = math.radians(self.goal[1]/100.0)
        lat2 = math.radians(self.boat[0]/100.0)
        lon2 = math.radians(self.boat[1]/100.0)

        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Radius of the Earth in meters
        radius = 6371 * 1000  # 6371 km

        # Calculate the distance
        distance = radius * c
        return distance

def main(args=None):
    rclpy.init(args=args)

    control_system = ControlSystem()

    while rclpy.ok():

        rclpy.spin_once(control_system, timeout_sec=2)

        if len(control_system.serial_rc) < 2:
            pass  # Don't have rc values
        elif float(control_system.serial_rc["state2"]) > 600:  # in RC
            control_system.get_logger().error("Currently in RC")

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
                try:
                    control_system.find_trim_tab_state(control_system.airmar_data["apparentWind"]["direction"])
                except Exception as e:
                    control_system.get_logger().error(str(e))
            else:
                control_system.get_logger().info("No wind angle values")
            if float(control_system.serial_rc["state1"]) < 800:
                ballast_angle = 0
                if control_system.serial_rc["ballast"] > 1200:
                    if control_system.ballast_adc_value > 0.25:
                        ballast_angle = 130
                elif control_system.serial_rc["ballast"] < 800:
                    if control_system.ballast_adc_value < 0.75:
                        ballast_angle = 60
                ballast_json = {"channel": "12", "angle": ballast_angle}
                control_system.pwm_control_publisher_.publish(control_system.make_json_string(ballast_json))
            else:
                control_system.update_winds(control_system.airmar_data["apparentWind"]["direction"])
                control_system.ballast_alg_active()
            rudder_angle = (float(control_system.serial_rc["rudder"]) / 2000 * 90) + 25
            rudder_json = {"channel": "8", "angle": rudder_angle}
            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))

        elif float(control_system.serial_rc["state2"]) < 600:
            control_system.get_logger().error("Currently in AUTONOMOUS")

            if control_system.cv == 1:
                rudder_json = {"channel": "8", "angle": 106}
                control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))
            else:

                # code to control ballast
                if float(control_system.serial_rc["state1"]) < 800:
                    ballast_angle = 0
                    if control_system.serial_rc["ballast"] > 1200:
                        if control_system.ballast_adc_value > 0.25:
                            ballast_angle = 130
                    elif control_system.serial_rc["ballast"] < 800:
                        if control_system.ballast_adc_value < 0.75:
                            ballast_angle = 60
                    ballast_json = {"channel": "12", "angle": ballast_angle}
                    control_system.pwm_control_publisher_.publish(control_system.make_json_string(ballast_json))
                else:
                    control_system.update_winds(control_system.airmar_data["apparentWind"]["direction"])
                    control_system.ballast_alg_active()

                # code to Control the  Trim Tab
                if "apparentWind" in control_system.airmar_data and "direction" in control_system.airmar_data[
                    "apparentWind"]:
                    try:
                        control_system.find_trim_tab_state(control_system.airmar_data["apparentWind"]["direction"])
                    except Exception as e:
                        control_system.get_logger().error(str(e))
                else:
                    control_system.get_logger().info("No wind angle values")

                # code to control the rudders (aka nav alg stuff)

                if "Latitude" in control_system.airmar_data and "Longitude" in control_system.airmar_data:
                    control_system.boat = np.array(
                        [float(control_system.airmar_data["Latitude"]), float(control_system.airmar_data["Longitude"])])
                    
                    distance = control_system.distance_to_goal()
               
                    if distance <= 0.01 and control_system.accuracyCounter < 7: # ~10m is the placeholder threshold distance, in units of meters
                        if control_system.goals_queue:
                            control_system.goal = control_system.goals_queue.popleft()
                            control_system.accuracyCounter += 1 # increment until we've passed 7 waypoints
                    elif distance <= 0.005: # much tighter for crossing the gate
                        if control_system.goals_queue:
                            control_system.goal = control_system.goals_queue.popleft()

                    if "apparentWind" in control_system.airmar_data and "direction" in control_system.airmar_data[
                        "apparentWind"]:
                        curr_wind_value = control_system.update_winds(
                            control_system.airmar_data["apparentWind"]["direction"])
                        # curr_heading_value = float(control_system.airmar_data["currentHeading"])
                        curr_heading_value = control_system.heading_adc_value
                        true_wind_value = (curr_wind_value + curr_heading_value) % 360
                        wind_cos = math.cos(-true_wind_value)
                        wind_sin = math.sin(-true_wind_value)

                        # control_system.get_logger().error("wind values")
                        # control_system.get_logger().error(str(wind_cos))
                        # control_system.get_logger().error(str(wind_sin))

                        control_system.wind = np.array([wind_sin, wind_cos])

                        # get the desired heading
                        desired_heading_cartesian = control_system.calc_heading()

                        # convert to polar
                        final_desired_heading = control_system.unit_circle_to_heading_gps(desired_heading_cartesian[0],
                                                                                          desired_heading_cartesian[1])
                        control_system.get_logger().error("final desired heading: " + str(final_desired_heading))
                        control_system.get_logger().error("current heading: " + str(control_system.heading_adc_value))

                        # check if we are currently offset from the desired heading
                        heading_difference = math.atan2(
                            math.sin(math.radians(final_desired_heading - control_system.heading_adc_value)),
                            math.cos(math.radians(final_desired_heading - control_system.heading_adc_value)))
                        heading_difference = math.degrees(heading_difference)

                        if abs(heading_difference) < 10:
                            rudder_json = {"channel": "8", "angle": 69}
                            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))

                        elif heading_difference > 0:
                            rudder_json = {"channel": "8", "angle": 94}
                            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))

                        # elif final_desired_heading + 5 < float(control_system.airmar_data["currentHeading"]):
                        else:
                            rudder_json = {"channel": "8", "angle": 45}
                            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))

                    else:
                        control_system.get_logger().error("No Wind Data")
                else:
                    control_system.get_logger().error("No GPS Data")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()