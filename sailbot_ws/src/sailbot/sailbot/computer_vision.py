import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2

class NeonOrangeDetector(Node):
    def __init__(self):
        super().__init__("computer_vision")
        self.publisher_ = self.create_publisher(Int32, "neon_orange_detected", 10)

        # Define the lower and upper bounds of the neon orange color in HSV format
        self.orange_lower = (5, 90, 80)
        self.orange_upper = (20, 254, 253)

        # Set the minimum area requirement for an object to be considered
        self.min_area = 1500

        # Open the default camera
        self.cap = cv2.VideoCapture("/dev/video0")

        # Set the resolution of the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.timer_ = self.create_timer(0.01, self.detect_orange)

    def detect_orange(self):
        # Read a frame from the camera
        ret, frame = self.cap.read()

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the neon orange color and apply it to the original frame
        neon_orange_mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)

        # Find contours in the mask
        contours, _ = cv2.findContours(neon_orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw a bounding box around the largest contour (if one exists and its area is greater than min_area)
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.publisher_.publish(Int32(data=1))
                self.get_logger().error("found a buoy")
                # cv2.imshow("neon orange detection", frame)
                return

        # cv2.imshow("neon orange detection", frame)

        # No object detected
        self.publisher_.publish(Int32(data=0))
        self.get_logger().error("did not find a buoy")

    def __del__(self):
        # Release the camera and close the window
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    neon_orange_detector = NeonOrangeDetector()
    while rclpy.ok():
        rclpy.spin_once(neon_orange_detector)
        cv2.waitKey(1)
    neon_orange_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()