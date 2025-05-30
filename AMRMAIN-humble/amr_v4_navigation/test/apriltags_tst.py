import rclpy,os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

robot_name = os.getenv('ROBOT_MODEL','amr_x')

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__(robot_name+'apriltag_detector')
        self.subscription = self.create_subscription(Image, "/"+robot_name+'/slam_lidar_camera/color/image_raw',
                                             self.image_callback,
                                             qos_profile=rclpy.qos.qos_profile_system_default)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.apriltag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters()

    def image_callback(self, msg):
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.apriltag_dict, parameters=self.parameters)

        if ids is not None:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Convert each of the (x, y)-coordinate pairs to integers
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

                # Draw the bounding box of the AprilTag detection
                cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)

                # Draw the center (x, y)-coordinates of the AprilTag
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

                # Draw the tag ID on the image
                cv2.putText(cv_image, str(markerID), (topLeft[0], topLeft[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.get_logger().info(f"Detected Tag ID: {markerID}")

        
        #cv2.imshow("AprilTag Detection", cv_image)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()