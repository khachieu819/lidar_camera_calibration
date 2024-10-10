import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
import numpy as np
import cv2
import cv2.aruco as aruco


file_path = '/home/khieu/lidar3d_ws/src/pose_cam_estimate/config/points_python.txt'

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
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            
            if np.asarray(self.markerIds).any() != None:
                # parameters = []
                # # print(len(self.markerCorners))
                # for corners in self.markerCorners:
                #     for corner in corners:
                #         for point in corner:
                #             pix = point

                #             if self.intrinsics:
                #                 depth = cv_image[int(pix[0]), int(pix[1])]

                #                 # the result will return value in milimeter
                #                 result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[1], pix[0]], depth)
                #                 parameters.append(result)
                                
                #                 outputImage = self.image.copy()
                #                 cv2.aruco.drawDetectedMarkers(outputImage, self.markerCorners, self.markerIds)
                #                 cv2.imshow("image", outputImage)
                #                 cv2.waitKey(3)
            
                # # Save parameters to the text file
                # for para in parameters:
                #     for j in range(len(para)):
                #         para[j] /= 1000  # Modify the element in place (mili meter -> meter)
    
                # with open(file_path, 'w') as file:
                #     file.write(f" {parameters[0]}\n")
                #     file.write(f" {parameters[1]}\n")
                #     file.write(f" {parameters[2]}\n")
                #     file.write(f" {parameters[3]}\n")

                # print(f"Parameters saved to {file_path}")

            
            # print("----------------------")

        except CvBridgeError as e:
            print(e)
            return
        

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.instrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            # print(self.intrinsics.width)
            self.intrinsics.height = cameraInfo.height
            # print(self.intrinsics.height)
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
    topic = '/camera/aligned_depth_to_color/image_raw'
    listener = ImageListener(topic)


    rospy.spin()

if __name__ == '__main__':
    node_name = "getCorner3DPosition"
    rospy.init_node(node_name)
    main()
