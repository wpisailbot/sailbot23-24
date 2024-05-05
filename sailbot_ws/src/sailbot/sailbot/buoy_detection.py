import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, NavSatFix
from geographic_msgs.msg import GeoPoint
from cv_bridge import CvBridge

from sailbot_msgs.msg import BuoyDetectionStamped

import cv2
import numpy as np
import math
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment
import pyproj
from pyproj import Transformer

from geopy.distance import geodesic

PIXEL_SIZE = 0.002 # In mm
CAMERA_RESOLUTION = (2208, 1242) # Width, Height
CAMERA_WORLD_POSITION = np.array([0, 0, -1.8]) # XYZ in meters
CAMERA_ORIENTATION = np.array([0, 0, 0]) # Orientation as pitch, yaw, roll

#Camera calibration for LEFT camera info from ZED application
CX = 1099.58
CY = 634.60

FX = 1053.2
FY = 1052.39

camera_matrix = np.array([[FX, 0, CX],
                          [0, FY, CY],
                          [0, 0, 1]])

K1, K2, P1, P2, K3 = -0.0393, 0.0085, 0.0001, 0.0004, -0.0045
distortion_coefficients = np.array([K1, K2, P1, P2, K3])

# Define geographic coordinate system (WGS84)
geographic = pyproj.CRS('EPSG:4326')  # WGS 84

# Define local ENU coordinate system based on a specific zone
local_enu = pyproj.CRS('EPSG:32619')  # UTM Zone 19N, North Hemisphere

# Create a transformer to convert from geographic to local ENU coordinates
geo_to_enu_transformer = Transformer.from_crs(geographic, local_enu, always_xy=True)

# Create a transformer to convert from local ENU back to geographic coordinates
enu_to_geo_transformer = Transformer.from_crs(local_enu, geographic, always_xy=True)

def geodetic_to_enu(lat, lon):
    """Transform from lat, lon to ENU coordinates."""
    x, y = geo_to_enu_transformer.transform(lon, lat)
    return x, y

def enu_to_geodetic(x, y):
    """Transform from ENU coordinates back to geodetic."""
    lon, lat = enu_to_geo_transformer.transform(x, y)
    return lat, lon

def create_kalman_filter(initial_x, initial_y):
    kf = KalmanFilter(dim_x=4, dim_z=2)
    dt = 1  # time step

    # State transition matrix
    kf.F = np.array([[1, dt, 0,  0],
                     [0,  1, 0,  0],
                     [0,  0, 1, dt],
                     [0,  0, 0,  1]])

    # Measurement function
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])

    # Measurement uncertainty
    kf.R = np.array([[10, 0],
                     [0, 10]])

    # Initial state covariance
    kf.P *= 1000

    # Process noise
    kf.Q = np.eye(4)

    # Initialize state
    kf.x = np.array([initial_x, 0, initial_y, 0])  # Positions and velocities

    return kf

class Track:
    id_counter = 0

    def __init__(self, initial_e, initial_n):
        self.id = Track.id_counter
        Track.id_counter += 1 
        self.kf = create_kalman_filter(initial_e, initial_n)
        self.time_since_update = 0
        self.history = [(self.kf.x[0], self.kf.x[2])]

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return (self.kf.x[0], self.kf.x[2])

    def update(self, detection_enu):
        self.kf.update(detection_enu)
        self.time_since_update = 0
        self.history.append((self.kf.x[0], self.kf.x[2]))
    
    def get_position(self):
        return (self.kf.x[0], self.kf.x[2])

def calculate_offset_position(lat, lon, heading, z_distance, x_distance):
    heading_rad = math.radians(heading)

    # Calculate the effective heading by considering the forward (Z) and rightward (X) distances
    angle_from_heading = math.atan2(x_distance, z_distance)

    # Calculate angle of the resultant vector relative to the heading direction
    total_angle = heading_rad + angle_from_heading

    resultant_distance = (z_distance**2 + x_distance**2)**0.5

    new_location = geodesic(meters=resultant_distance).destination((lat, lon), math.degrees(total_angle))
    return GeoPoint(latitude=new_location.latitude, longitude=new_location.longitude)

class BuoyDetection(Node):
    current_x_scaling_factor = 1.0
    current_y_scaling_factor = 1.0
    latitude, longitude = 42.0396766107111, -71.84585650616927
    heading = 0
    tracks = []
    def __init__(self):
        super().__init__('object_detection_node')

        self.set_parameters()
        self.get_parameters()

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            10)
        self.airmar_position_subscription = self.create_subscription(
                NavSatFix,
                '/airmar_data/lat_long',
                self.airmar_position_callback,
                10)
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            '/airmar_data/heading',
            self.airmar_heading_callback,
            10)

        self.mask_publisher = self.create_publisher(
            Image,
            'cv_mask',
            10
        )
        self.buoy_position_publisher = self.create_publisher(
            BuoyDetectionStamped,
            'buoy_position',
            10
        )
        self.bridge = CvBridge()
    
        self.get_logger().info("Setup done")

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.cv.lower_h', 5)
        self.declare_parameter('sailbot.cv.lower_s', 49)
        self.declare_parameter('sailbot.cv.lower_v', 120)
        self.declare_parameter('sailbot.cv.upper_h', 110)
        self.declare_parameter('sailbot.cv.upper_s', 255)
        self.declare_parameter('sailbot.cv.upper_v', 255)
        self.declare_parameter('sailbot.cv.buoy_circularity_threshold', 0.6)
        self.declare_parameter('sailbot.cv.buoy_diameter_meters', 0.5)




    def get_parameters(self) -> None:
        self.lower_h = self.get_parameter('sailbot.cv.lower_h').get_parameter_value().integer_value
        self.lower_s = self.get_parameter('sailbot.cv.lower_s').get_parameter_value().integer_value
        self.lower_v = self.get_parameter('sailbot.cv.lower_v').get_parameter_value().integer_value
        self.upper_h = self.get_parameter('sailbot.cv.upper_h').get_parameter_value().integer_value
        self.upper_s = self.get_parameter('sailbot.cv.upper_s').get_parameter_value().integer_value
        self.upper_v = self.get_parameter('sailbot.cv.upper_v').get_parameter_value().integer_value
        self.buoy_circularity_threshold = self.get_parameter('sailbot.cv.buoy_circularity_threshold').get_parameter_value().double_value
        self.buoy_diameter_meters = self.get_parameter('sailbot.cv.buoy_diameter_meters').get_parameter_value().double_value







    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
    
    def airmar_heading_callback(self, msg: Float64) -> None:
        self.heading = msg.data

    def publish_tracks(self) -> None:
        #self.get_logger().info(f"Num tracks: {len(self.tracks)}")
        for track in self.tracks:
            if track.time_since_update == 0:  # Publish only for tracks updated in the last frame
                enu_position = track.get_position()
                latlon = enu_to_geodetic(enu_position[0], enu_position[1])
                #self.get_logger().info(f"Publishing detection with position: {latlon}")
                detection = BuoyDetectionStamped()
                detection.position.latitude = latlon[0]
                detection.position.longitude = latlon[1]
                detection.id = track.id
                self.buoy_position_publisher.publish(detection)

    def associate_detections_to_tracks(self, tracks, detections_enu, max_distance=3):
        if len(tracks) == 0:
            return [], set(range(len(detections_enu)))
        
        if len(detections_enu) == 0:
            return [], []

        cost_matrix = np.zeros((len(tracks), len(detections_enu)), dtype=np.float32)
        for t, track in enumerate(tracks):
            predicted_enu = track.predict()
            #self.get_logger().info(f"Predicted enu: {predicted_enu}")
            for d, detection_enu in enumerate(detections_enu):
                # Now we use ENU coordinates directly here
                cost_matrix[t, d] = np.linalg.norm(np.array(predicted_enu) - np.array(detection_enu))
        
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        matches = []
        unmatched_detections = set(range(len(detections_enu)))
        unmatched_tracks = set(range(len(tracks)))

        for r, c in zip(row_ind, col_ind):
            #self.get_logger().info(f"Cost matrix: {cost_matrix[r, c]}")
            if cost_matrix[r, c] < max_distance:
                matches.append((r, c))
                unmatched_detections.remove(c)
                unmatched_tracks.remove(r)

        return matches, list(unmatched_detections)#, list(unmatched_tracks)

    def listener_callback(self, data) -> None:
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #self.get_logger().info("frame width: "+str(current_frame.shape))
        self.current_x_scaling_factor = current_frame.shape[1]/CAMERA_RESOLUTION[0]
        self.current_y_scaling_factor = current_frame.shape[0]/CAMERA_RESOLUTION[1]
        #self.get_logger().info("x scaling factor: "+str(self.current_x_scaling_factor))
        #self.get_logger().info("y scaling factor: "+str(self.current_y_scaling_factor))
        
        image = cv2.undistort(current_frame, camera_matrix, distortion_coefficients)

        contours = self.detect_orange_objects(image)
        detections = [self.calculate_object_center(contour) for contour in contours]
        #self.get_logger().info(f"detections: {detections}")

        detections_latlon = []
        for triplet in detections:
            world_coords = self.pixel_to_world(triplet[0], triplet[1], triplet[2], FX*self.current_x_scaling_factor, FY*self.current_y_scaling_factor, CX*self.current_x_scaling_factor, CY*self.current_y_scaling_factor)
            latlon = calculate_offset_position(self.latitude, self.longitude, self.heading, world_coords[2], world_coords[0])
            detections_latlon.append((latlon.latitude, latlon.longitude))
        #self.get_logger().info(f"detections_latlon: {detections_latlon}")

        detections_enu = [geodetic_to_enu(lat, lon) for lat, lon in detections_latlon]
        #self.get_logger().info(f"detections_enu: {detections_enu}")
        
        matches, unmatched_detections = self.associate_detections_to_tracks(self.tracks, detections_enu)
        
        for track_idx, detection_idx in matches:
            self.tracks[track_idx].update(detections_enu[detection_idx])

        for idx in unmatched_detections:
            enu = detections_enu[idx]
            self.tracks.append(Track(enu[0], enu[1]))

        self.publish_tracks()


    def fill_holes(self, image):
        # Find contours
        contours, hierarchy = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # Create an all white image to draw filled contours
        filled_image = np.ones(image.shape, dtype=np.uint8) * 255

        # Iterate through contours and hierarchy
        for idx, contour in enumerate(contours):
            # Check if it's an internal contour
            if hierarchy[0][idx][3] != -1:
                # It's a hole, fill it
                cv2.drawContours(filled_image, [contour], -1, (0, 0, 0), 1)
            else:
                # It's an external contour, just draw it
                cv2.drawContours(filled_image, [contour], -1, (0, 0, 0), -1)
        
        return 255-filled_image

    def detect_orange_objects(self, image):
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for orange color and create a mask
        #lower_orange = np.array([3, 205, 74])
        lower_orange = np.array([3, 120, 74])
        upper_orange = np.array([15, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        mask = self.fill_holes(mask)
        #mask = cv2.GaussianBlur(mask, (5, 5), 2)
        kernel = np.ones((2, 2), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        #edges = cv2.Canny(mask, 50, 150) 

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_area = []
        # calculate area and filter into new array
        for con in contours:
            area = cv2.contourArea(con)
            if 100 < area < 10000:
                contours_area.append(con)
        
        contours_cirles = []
        # check if contour is of circular shape
        for con in contours_area:
            perimeter = cv2.arcLength(con, True)
            area = cv2.contourArea(con)

            if perimeter == 0:
                break
            circularity = 4*math.pi*(area/(perimeter*perimeter))
            if self.buoy_circularity_threshold < circularity < 1.2:
                contours_cirles.append(con)
                (x, y), radius = cv2.minEnclosingCircle(con)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(mask_rgb, center, radius, (0, 255, 0), 2)

        img_msg = self.bridge.cv2_to_imgmsg(mask_rgb, encoding="rgb8")
        self.mask_publisher.publish(img_msg)
        #cv2.imwrite('/home/sailbot/test_image.jpg', mask_rgb)
        return contours_cirles

    def calculate_depth(self, contour):

        (x, y), radius = cv2.minEnclosingCircle(contour)
        #self.get_logger().info("radius: "+str(radius))

        #depth = (KNOWN_DIAMETER*PIXEL_SIZE*current_x_scaling_factor/2 * FOCAL_LENGTH*current_x_scaling_factor) / radius
        depth = (self.buoy_diameter_meters*FX)/(radius*2/self.current_x_scaling_factor)
        return depth

    def calculate_object_center(self, contour):
        # Calculate the center of the contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        
        cZ = self.calculate_depth(contour)
        return cX, cY, cZ

    def pixel_to_world(self, x_pixel, y_pixel, depth, f_x, f_y, c_x, c_y):
        """
        Convert pixel coordinates to world coordinates.

        Parameters:
        x_pixel, y_pixel (float): Pixel coordinates of the sphere's center.
        depth (float): Depth of the sphere from the camera.
        f_x, f_y (float): Focal length of the camera (in pixels).
        c_x, c_y (float): Optical center of the camera (in pixels).

        Returns:
        tuple: Real-world coordinates (x, y, z) of the sphere.
        """
        # Convert to normalized camera coordinates
        x_norm = (x_pixel - c_x) / f_x
        y_norm = (y_pixel - c_y) / f_y

        # Convert to world coordinates
        x_world = x_norm * depth
        y_world = y_norm * depth

        return x_world, y_world, depth
    

def main(args=None):
    rclpy.init(args=args)

    buoy_detection = BuoyDetection()

    rclpy.spin(buoy_detection)

    buoy_detection.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
