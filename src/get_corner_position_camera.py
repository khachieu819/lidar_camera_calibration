import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
import numpy as np
import cv2
import cv2.aruco as aruco
from std_msgs.msg import Float32MultiArray



file_path = '/home/khieu/lidar3d_ws/src/pose_cam_estimate/config/transform_matrix.txt'
d = np.array((0, 0, 0, 0))
rotation = None
translation = None


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub_image_depth = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.sub_image_color = rospy.Subscriber('/camera/color/image_raw', msg_Image, self.imageColorCallback)
        self.sub_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.instrinsics = None
        self.image = None
        self.markerCorners = None
        self.markerIds = None

    def imageColorCallback(self, data):
        self.image = np.asarray(self.bridge.imgmsg_to_cv2(data, 'bgr8'))
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
        dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.markerCorners, self.markerIds, _ = detector.detectMarkers(gray)
        
        

    def imageDepthCallback(self, data):
        global rotation, translation
        try:
            
            k = np.array(((self.intrinsics.fx, 0, self.intrinsics.ppx), (0, self.intrinsics.fy, self.intrinsics.ppy), (0,0,1)))        # instrinsic matrix
            
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            
            
            
            if np.asarray(self.markerIds).any() != None:
                for i in range(0, len(self.markerIds)):
                    marker_length = 0.14  # Length of the marker's side in meters

                    object_points = np.array([
                        [-marker_length / 2, -marker_length / 2, 0],  # Bottom-left
                        [marker_length / 2, -marker_length / 2, 0],   # Bottom-right
                        [marker_length / 2, marker_length / 2, 0],    # Top-right
                        [-marker_length / 2, marker_length / 2, 0]     # Top-left
                    ], dtype=np.float32)

                    for i in range(len(self.markerIds)):
                        marker_corners = self.markerCorners[i][0]

    
                    success, rotation_vector, translation_vector = cv2.solvePnP(object_points, marker_corners, k, d)
                    if success:
                        # print(f"Marker ID: {self.markerIds[i][0]}")
                        # print("Rotation Vector:\n", rotation_vector)
                        # print("Translation Vector:\n", translation_vector)

                        rotation = rotation_vector
                        translation = translation_vector
                        # Optionally, draw the axis on the marker
                        axis_length = 0.1  # Length of the axis
                        axis = np.array([
                            [0, 0, 0],  # Origin
                            [axis_length, 0, 0],  # X-axis
                            [0, axis_length, 0],  # Y-axis
                            [0, 0, axis_length]   # Z-axis
                        ], dtype=np.float32)

        except CvBridgeError as e:
            print(e)
            return
        

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.instrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
            
        except CvBridgeError as e:
            print(e)
            return
        
def main():
    global translation, rotation, poseFlag
    topic = '/camera/aligned_depth_to_color/image_raw'
    listener = ImageListener(topic)

    pub = rospy.Publisher("marker_pose_transformation", Float32MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        msg = Float32MultiArray()
        if rotation is not None:
            if translation is not None:
                
                # print(rotation.astype(float).tolist()[0][0])
                msg.data = [ translation.astype(float).tolist()[0][0], translation.astype(float).tolist()[1][0], translation.astype(float).tolist()[2][0],
                    rotation.astype(float).tolist()[0][0], rotation.astype(float).tolist()[1][0], rotation.astype(float).tolist()[2][0]
                            ]
                pub.publish(msg)
        
        rospy.sleep(2)




    rospy.spin()

if __name__ == '__main__':
    node_name = "getCorner3DPosition"
    rospy.init_node(node_name)
    main()
