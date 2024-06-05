import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, NavSatFix
from geographic_msgs.msg import GeoPoint
from cv_bridge import CvBridge

from sailbot_msgs.msg import BuoyDetectionStamped, CVParameters, BuoyTypeInfo, AnnotatedImage

import cv2
import numpy as np
import math
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment
import pyproj
from pyproj import Transformer
from typing import List
import time
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
        self.last_update_time = time.time() 

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return (self.kf.x[0], self.kf.x[2])

    def update(self, detection_enu):
        self.kf.update(detection_enu)
        self.time_since_update = 0
        self.history.append((self.kf.x[0], self.kf.x[2]))
        self.last_update_time = time.time()
    
    def get_position(self):
        return (self.kf.x[0], self.kf.x[2])

def calculate_offset_position(lat, lon, heading, z_distance, x_distance):
    """
    Calculates a new geographic position offset by specified distances in the forward (z-axis) and rightward (x-axis) directions
    relative to a given heading from an initial latitude and longitude.

    :param lat: The latitude of the starting point in decimal degrees.
    :param lon: The longitude of the starting point in decimal degrees.
    :param heading: The current heading in degrees from the North, measured clockwise.
    :param z_distance: The distance to move forward from the starting point, in meters.
    :param x_distance: The distance to move rightward from the starting point, in meters.

    :return: A 'GeoPoint' object representing the new geographic position in decimal degrees of latitude and longitude.

    Function behavior includes:
    - Converting the heading from degrees to radians for trigonometric calculations.
    - Calculating the angle from the original heading to the resultant vector created by the specified distances.
    - Determining the total angle by combining the heading with this new angle.
    - Computing the resultant distance using the Pythagorean theorem.
    - Using the 'geodesic' method from the 'geopy' library to calculate the new position based on this resultant distance and angle.

    This function assumes that the 'geodesic' method is capable of calculating a destination point given a starting point,
    a distance, and a bearing. The 'GeoPoint' structure returned should be compatible with the needs of geographic
    information systems or other location-based services.
    """
    heading_rad = math.radians(heading)

    # Calculate the effective heading by considering the forward (Z) and rightward (X) distances
    angle_from_heading = math.atan2(x_distance, z_distance)

    # Calculate angle of the resultant vector relative to the heading direction
    total_angle = heading_rad + angle_from_heading

    resultant_distance = (z_distance**2 + x_distance**2)**0.5

    new_location = geodesic(meters=resultant_distance).destination((lat, lon), math.degrees(total_angle))
    return GeoPoint(latitude=new_location.latitude, longitude=new_location.longitude)

class BuoyDetection(Node):
    """
    A ROS2 node responsible for detecting buoys using image data from a camera. This node subscribes to image streams,
    processes the images to detect objects based on color and shape criteria, and publishes the locations of detected buoys.

    :ivar current_x_scaling_factor: Scaling factor to adjust the pixel coordinates based on the image width.
    :ivar current_y_scaling_factor: Scaling factor to adjust the pixel coordinates based on the image height.
    :ivar latitude: Current latitude of the node.
    :ivar longitude: Current longitude of the node.
    :ivar heading: Current heading of the vehicle in degrees.
    :ivar tracks: List of current tracking objects representing detected buoys.

    **Methods**:

    - **publish_tracks**: Publishes the current tracked positions of detected buoys.
    - **associate_detections_to_tracks**: Associates new detections with existing tracks using a cost matrix based on Euclidean distance.
    - **listener_callback**: Processes each incoming image, detects buoys, and manages tracks.
    - **fill_holes**: Fills holes within binary images to create solid objects, improving reliability of object detection.
    - **detect_colored_objects**: Detects colored objects in the image by applying a color threshold and shape analysis.
    - **calculate_depth**: Finds the depth of the object by sampling the depth texture from the ZED camera.
    - **calculate_object_center**: Calculates the center of detected objects in pixel coordinates.
    - **pixel_to_world**: Converts pixel coordinates to world coordinates using intrinsic camera parameters.

    **Usage**:

    - The node must be managed by state_manager

    **Notes**:

    - The ZED2 ros node must be running in order for this node to receive frames.
    
    """
    current_x_scaling_factor = 1.0
    current_y_scaling_factor = 1.0
    latitude, longitude = 42.84456, -70.97622
    heading = 0
    tracks: List[Track] = []
    buoy_types: List[BuoyTypeInfo] = []
    depth_image = None

    def __init__(self):
        super().__init__('object_detection_node')

        orangeType = BuoyTypeInfo()
        orangeType.buoy_diameter = 0.5
        orangeType.name = "orange"
        orangeType.hsv_bounds.lower_h = 0
        orangeType.hsv_bounds.lower_s = 98
        orangeType.hsv_bounds.lower_v = 52
        orangeType.hsv_bounds.upper_h = 113
        orangeType.hsv_bounds.upper_s = 255
        orangeType.hsv_bounds.upper_v = 255
        self.buoy_types.append(orangeType)

        greenType = BuoyTypeInfo()
        greenType.buoy_diameter = 0.5
        greenType.name = "green"
        greenType.hsv_bounds.lower_h = 81
        greenType.hsv_bounds.lower_s = 49
        greenType.hsv_bounds.lower_v = 120
        greenType.hsv_bounds.upper_h = 140
        greenType.hsv_bounds.upper_s = 255
        greenType.hsv_bounds.upper_v = 255
        self.buoy_types.append(greenType)

        whiteType = BuoyTypeInfo()
        whiteType.buoy_diameter = 0.5
        whiteType.name = "white"
        whiteType.hsv_bounds.lower_h = 0
        whiteType.hsv_bounds.lower_s = 0
        whiteType.hsv_bounds.lower_v = 245
        whiteType.hsv_bounds.upper_h = 255
        whiteType.hsv_bounds.upper_s = 255
        whiteType.hsv_bounds.upper_v = 255
        self.buoy_types.append(whiteType)

        self.set_parameters()
        self.get_parameters()

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.zed_image_callback,
            10)
        self.camera_depth_image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.camera_depth_image_callback,
            10)
        self.airmar_position_subscription = self.create_subscription(
                NavSatFix,
                '/airmar_data/lat_long',
                self.airmar_position_callback,
                10)
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            'heading',
            self.airmar_heading_callback,
            10)
        self.cv_parameters_subscription = self.create_subscription(
            CVParameters,
            'cv_parameters',
            self.cv_parameters_callback,
            10)
        
        self.mask_publisher = self.create_publisher(
            AnnotatedImage,
            'cv_mask',
            10
        )
        self.buoy_position_publisher = self.create_publisher(
            BuoyDetectionStamped,
            'buoy_position',
            10
        )
        self.initial_cv_parameters_publisher = self.create_publisher(
            CVParameters,
            'initial_cv_parameters',
            10
        )
        initial_parameters = CVParameters()
        initial_parameters.buoy_types.extend(self.buoy_types)
        initial_parameters.circularity_threshold = self.buoy_circularity_threshold
        sleep_rate = self.create_rate(1)
        while self.initial_cv_parameters_publisher.get_subscription_count() < 1:
            self.get_logger().info("waiting for cv parameters subscriber...")
            time.sleep(1)
        self.get_logger().info(f"Publishing initial cv parameters after finding {self.initial_cv_parameters_publisher.get_subscription_count()} subscribers")
        self.initial_cv_parameters_publisher.publish(initial_parameters)

        self.bridge = CvBridge()
    
        self.get_logger().info("Setup done")

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.cv.buoy_circularity_threshold', 0.6)
        self.declare_parameter('sailbot.cv.depth_error_threshold_meters', 3.0)
        self.declare_parameter('sailbot.cv.buoy_detection_lifetime_seconds', 3.0)



    def get_parameters(self) -> None:
        self.buoy_circularity_threshold = self.get_parameter('sailbot.cv.buoy_circularity_threshold').get_parameter_value().double_value
        self.depth_error_threshold_meters = self.get_parameter('sailbot.cv.depth_error_threshold_meters').get_parameter_value().double_value
        self.buoy_detection_lifetime_seconds = self.get_parameter('sailbot.cv.buoy_detection_lifetime_seconds').get_parameter_value().double_value



    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
    
    def airmar_heading_callback(self, msg: Float64) -> None:
        self.heading = msg.data
    
    def cv_parameters_callback(self, msg: CVParameters) -> None:
        #self.get_logger().info(f"Got new CV parameters: {msg}")
        self.buoy_types = msg.buoy_types
        self.buoy_circularity_threshold = msg.circularity_threshold

    def remove_stale_tracks(self):
        current_time = time.time()
        self.tracks = [track for track in self.tracks if current_time - track.last_update_time <= self.buoy_detection_lifetime_seconds]

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
        """
        Associates detections to existing tracks using a cost matrix based on the Euclidean distance between predicted track
        positions (from a Kalman filter, but we don't have velocity detection, so it's useless for now) and new detections. 
        The function handles cases where there are no tracks or detections and returns matches and unmatched detections for further processing.

        :param tracks: A list of track objects which maintain state and prediction methods.
        :param detections_enu: A list of detections in East-North-Up (ENU) coordinates.
        :param max_distance: The maximum allowable distance for a detection to be considered a match with a track. Default is 3 meters.

        :return: A tuple containing two elements:
                - matches: A list of tuples (track_index, detection_index) where each tuple represents a confirmed match.
                - unmatched_detections: A list of indices from 'detections_enu' that did not match with any existing track.

        Function behavior includes:
        - Building a cost matrix where each element represents the distance between a track's predicted position and a detection's position.
        - Using the Hungarian algorithm (linear sum assignment) to find the optimal matching of tracks to detections based on the cost matrix.
        - Filtering these matches based on the 'max_distance' threshold to ensure only reasonable matches are kept.
        - Determining which detections and tracks remain unmatched after the matching process.
        """
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

    def zed_image_callback(self, data) -> None:
        """
        Callback function for handling image data received from a ROS subscriber. This function processes each image frame to detect,
        identify, and track orange objects, likely buoys, based on their color and shape. It converts ROS image messages to OpenCV
        format, undistorts the image using camera intrinsics, and utilizes various helper functions to compute the objects' 
        geographical and relative positions.

        :param data: A ROS Image message that contains the image data captured by a camera.

        :return: None. This function directly updates the system's state by modifying the tracking information and publishing the results.

        Function behavior includes:
        - Converting the ROS image message to an OpenCV image format.
        - Calculating scaling factors based on the camera's resolution.
        - Undistorting the image using pre-defined camera matrix and distortion coefficients.
        - Detecting orange objects in the image by analyzing their contours.
        - Calculating the center of each detected object.
        - Converting the pixel coordinates of these centers to world coordinates, then to latitude and longitude.
        - Transforming geodetic coordinates to East-North-Up (ENU) coordinates.
        - Associating detected objects with existing tracks or creating new tracks for unmatched detections.
        - Updating the track information and publishing the updated tracks for further processing or visualization.

        This function relies on a set of predefined variables and configurations such as camera calibration parameters ('camera_matrix',
        'distortion_coefficients'), and scaling factors ('FX', 'FY', 'CX', 'CY').
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #self.get_logger().info("frame width: "+str(current_frame.shape))
        self.current_x_scaling_factor = current_frame.shape[1]/CAMERA_RESOLUTION[0]
        self.current_y_scaling_factor = current_frame.shape[0]/CAMERA_RESOLUTION[1]
        #self.get_logger().info("x scaling factor: "+str(self.current_x_scaling_factor))
        #self.get_logger().info("y scaling factor: "+str(self.current_y_scaling_factor))

        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v_eq = cv2.equalizeHist(v)
        hsv_image_eq = cv2.merge([h, s, v_eq])

        hsv_image_eq = cv2.undistort(hsv_image_eq, camera_matrix, distortion_coefficients)

        detections = []
        for buoy_type in self.buoy_types:
            contours = self.detect_colored_objects(hsv_image_eq, buoy_type)
            detections.extend(
                [
                    center for contour in contours
                    if (center := self.calculate_object_center(contour, buoy_type.buoy_diameter)) is not None
                ]
            )        
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
        self.remove_stale_tracks()

    def camera_depth_image_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self.depth_image = cv2.undistort(depth, camera_matrix, distortion_coefficients)


    def fill_holes(self, image):
        """
        Fills holes within binary images to create solid objects, which improves the reliability of subsequent image processing operations.
        This function identifies contours within the image that represent both external boundaries and internal holes, filling
        the latter to solidify the objects.

        :param image: A binary image (numpy array) where objects are typically represented by white on a black background.

        :return: A binary image (numpy array) where internal holes within objects have been filled.

        Function behavior includes:
        - Finding contours in the image using OpenCV's findContours method, configured to retrieve both external and internal contours.
        - Creating a new all-white image of the same size as the input to serve as the base for drawing filled contours.
        - Iterating through each contour and its corresponding hierarchy level to determine if it is an internal contour (a hole).
        - Filling holes by drawing the internal contours as solid shapes on the new image.
        - Returning the complement of the filled image, effectively reversing the colors to match the input format (objects are white).

        This method is used in preparation for detection of orange circles.
        """
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

    def detect_colored_objects(self, hsv, type_info: BuoyTypeInfo):
        """
        Detects colored objects in an image using OpenCV for color segmentation and shape analysis. The function processes an input image,
        converts it to HSV color space, applies a color mask to isolate color regions, and identifies contours that meet
        specific area and circularity criteria.

        :param hsv: An image array in BGR format, typically received from an image capturing device or simulation environment.

        :return: A list of contours that represent detected colored objects which meet the area and circularity thresholds.
                These objects are presumed to be circular in shape, fitting the typical characteristics of buoys.

        Function behavior includes:
        - Converting the image to HSV color space for better color segmentation.
        - Applying a mask to filter out colors that are not within a specified color range.
        - Refining the mask using morphological operations to reduce noise and improve object isolation.
        - Detecting contours in the masked image and filtering these based on their area to remove too small or large objects.
        - Further analyzing the filtered contours for circularity to confirm if they match the expected shape of buoys.
        - Drawing detected objects on the mask image for visual verification.
        - Publishing the result image with detected objects highlighted to a ROS topic for further use or debugging.
        - Optionally saving the processed image to disk for analysis (commented out).

        """

        # Define range for orange color and create a mask
        #lower_orange = np.array([3, 205, 74])
        bounds = type_info.hsv_bounds
        lower_range = np.array([bounds.lower_h, bounds.lower_s, bounds.lower_v])
        upper_range = np.array([bounds.upper_h, bounds.upper_s, bounds.upper_v])
        mask = cv2.inRange(hsv, lower_range, upper_range)
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
        img_ann = AnnotatedImage(image=img_msg, source=type_info.name)
        self.mask_publisher.publish(img_ann)
        #cv2.imwrite('/home/sailbot/test_image.jpg', mask_rgb)
        return contours_cirles

    def estimate_depth(self, contour, diameter):

        (x, y), radius = cv2.minEnclosingCircle(contour)
        #self.get_logger().info("radius: "+str(radius))

        #depth = (KNOWN_DIAMETER*PIXEL_SIZE*current_x_scaling_factor/2 * FOCAL_LENGTH*current_x_scaling_factor) / radius
        depth = (diameter*FX)/(radius*2/self.current_x_scaling_factor)
        return depth
    
    def calculate_depth(self, contour, diameter):
        """
        Finds the depth of a contour by sampling the depth image from the ZED camera.
        Validates the object and depth by checking the sampled depth vs. the depth this buoy should be at
        based on its contour diameter.

        :param contour: The contour we are checking
        :param diameter: The diameter of the contour


        :return: Depth of the object if valid, None if not.

        Function behavior includes:
        - Checking for availability of the depth texture
        - Sampling the depth texture
        - Checking for invalid values in the depth texture
        - Calculating the minimum enclosing circle and expected depth of the contour
        - Checking if the sampled depth and estimated depth agree with each other

        """
        # Ensure the depth image is available
        if self.depth_image is None:
            self.get_logger().warn("Depth texture is None.")
            return None# self.estimate_depth(contour, diameter)

        # Calculate the radius of the contour
        (y, x), radius = cv2.minEnclosingCircle(contour)
        x = int(x)
        y = int(y)

        # Sample the depth value from the depth image at the contour center
        depth = self.depth_image[x, y]

        # Handle invalid depth values
        if np.isnan(depth) or depth <= 0 or not np.isfinite(depth):
            #self.get_logger().warn("Encountered NaN, infinite, or negative value in depth texture.")
            return None#self.estimate_depth(contour, diameter)

        # Calculate the expected depth based on the known diameter and contour radius
        expected_depth = (diameter * FX) / (2 * radius / self.current_x_scaling_factor)

        # Validate the depth from texture against the expected depth
        if abs(depth - expected_depth) > self.depth_error_threshold_meters:
            # Depth values are not consistent
            #self.get_logger().warn(f"Depth inconsistency: texture={depth}, expected={expected_depth}")
            return None
        
        return depth

    def calculate_object_center(self, contour, diameter):
        # Calculate the center of the contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        
        cZ = self.calculate_depth(contour, diameter)
        
        if(cZ is None):
            return None
        
        return cX, cY, cZ

    def pixel_to_world(self, x_pixel, y_pixel, depth, f_x, f_y, c_x, c_y):
        """
        Converts pixel coordinates to world coordinates based on the intrinsic camera parameters and the depth information.

        :param x_pixel: Pixel x-coordinate of the sphere's center.
        :param y_pixel: Pixel y-coordinate of the sphere's center.
        :param depth: Depth of the sphere from the camera, in the same units used for world coordinates.
        :param f_x: Focal length of the camera along the x-axis, in pixels.
        :param f_y: Focal length of the camera along the y-axis, in pixels.
        :param c_x: x-coordinate of the optical center of the camera, in pixels.
        :param c_y: y-coordinate of the optical center of the camera, in pixels.

        :return: A tuple (x, y, z) representing the real-world coordinates of the sphere, where 'z' is the depth,
                and 'x' and 'y' are the corresponding world coordinates derived from the pixel coordinates.

        This function uses the pinhole camera model to translate pixel coordinates (x_pixel, y_pixel) and a depth measurement
        into a 3D point in space relative to the camera. The conversion accounts for camera focal lengths and optical center offsets.
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
