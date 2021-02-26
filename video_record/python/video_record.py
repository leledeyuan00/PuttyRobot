#!/usr/bin/env python
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ppr_msgs.srv import videoRecord
from std_srvs.srv import Empty
import os

class ROSNode:
    filename = ""
    parameter = ""
    start_record = False
    finished = False
    out_video = cv.VideoWriter()
    # out_video_2 = cv.VideoWriter()
    home_dir = os.path.expanduser('~')
    directory = home_dir + '/lg/video/'

    def __init__(self):
        rospy.init_node("camera_record_node")
        rospy.loginfo("Starting ROSNode as camera_record_node.")

        # ros initial
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        start_record_srv = rospy.Service('start_record_video', videoRecord, self.handle_start_record)
        stop_record_srv = rospy.Service('stop_record_video', Empty, self.handle_stop_record)
        
        # state initial
        if(os.path.exists(self.directory)):
            pass
        else:
            os.makedirs(self.directory)
        pass

    def handle_start_record(self,req):
        self.filename = req.file_name
        self.parameter = req.parameter
        rospy.loginfo("Video record is started, filename is %s",self.filename)
        # fourcc = cv.VideoWriter_fourcc('X', 'V', 'I', 'D')
        fourcc = cv.VideoWriter_fourcc('H','2','6','4')
        # 

        fps = 30
        size = (1280, 720)
        
        fileFullName = self.directory + self.filename + '.avi'
        # fileFullName2 = self.directory + self.filename + '_2' + '.avi'
        self.out_video = cv.VideoWriter(fileFullName, fourcc, fps, size)
        # self.out_video_2 = cv.VideoWriter(fileFullName2, fourcc2, fps, size)
        # 

        outfile = open(self.directory + self.filename + '.txt',"w")
        outfile.write(self.parameter)
        outfile.close()
        self.start_record = True
        return []
    
    def handle_stop_record(self,req):
        self.start_record = False
        self.finished = True
        rospy.loginfo("Video record is finished, filename is %s",self.filename)
        return []

    def callback(self,image):
        if self.start_record:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            
            cv.imshow('img',cv_image)
            self.out_video.write(cv_image)
            # self.out_video_2.write(cv_image)
        elif self.finished:
            self.finished = False
            self.out_video.release()
            # self.out_video_2.release()
        else:
            pass
        # cv.waitKey(0)        


if __name__ == "__main__":
    name_node = ROSNode()
    rospy.spin()

