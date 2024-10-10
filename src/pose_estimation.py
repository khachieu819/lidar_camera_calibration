import numpy as np
import cv2
import sys
from utils import ARUCO_DICT, closestDistanceBetweenLines
import pyrealsense2 as rs # type: ignore
import time
import numpy
from cv_bridge import CvBridge



pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgra8, 30)

pipeline.start(config)
color_sensor = pipeline.get_active_profile().get_device().query_sensors()[1]

color_sensor.set_option(rs.option.exposure, 200)
align = rs.align(rs.stream.color)

def pose_estimation(frame, aruco_dict_type, mtx, distortion):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

    # ------------------------------------------------
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2()


    instrinsic = rs.intrinsics()
    instrinsic.width = 1280
    instrinsic.height = 720
    instrinsic.ppx = 632.6346435546875
    instrinsic.ppy = 351.7813720703125
    instrinsic.fx = 911.451904296875
    instrinsic.fy = 911.5646362304688
    instrinsic.model = rs.distortion.brown_conrady
    result = rs.rs2_deproject_pixel_to_point(instrinsic, [743, 163], 3.0)
    print(result)
    #-------------------------------------------------



    
    if(numpy.asarray(markerIds).any() != None):
        for i in range(0, len(markerIds)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.04, mtx, distortion)     # 0.4 meter
        for corners in markerCorners:
            corner = corners.reshape((4, 2))

            # convert each of the (x, y)-coordinate pairs to integers
            (topLeft, topRight, bottomRight, bottomLeft) = corner
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            cv2.imshow('Color image', frame)



    

    # return frame
    

if __name__ == '__main__':
    aruco_dict_type = ARUCO_DICT["DICT_5X5_100"]
    # aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
    k = np.array(((911.451904296875, 0, 632.6346435546875), (0, 911.5646362304688, 351.7813720703125), (0,0,1)))        # instrinsic matrix
    d = np.array((0, 0, 0, 0))

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        print(color_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        

        # h,w,_ = color_image.shape
        # width = 1000
        # height = int(width*(h/w))
        # color_image = cv2.resize(color_image, (width, height), interpolation = cv2.INTER_CUBIC)

        pose_estimation(color_image, aruco_dict_type, k, d)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            
            # save to coordinate_marker.txt file
            filename = 'coordinate_marker.txt'

            with open(filename, 'w') as file:
                file.write()
            break

    
    pipeline.stop()
    cv2.destroyAllWindows()

    
    
