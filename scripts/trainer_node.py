#!/usr/bin/env python3
import time
import rospy
from jetracer_ros.srv import recording, recordingResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import csv
from datetime import datetime
from time import time
import uuid
import math
import string
class trainer:

    is_recording  = False
    csv_filename = "training.csv"
    storage_location = None
    current_folder = None
    csv_file = None
    steering = 0
    throttle = 0
    image_counter = 0
    data = []
    last_sample = time()
    sample_interval_ms = 2000
    vel_treshold = 0.1

    def __init__(self):
        self.bridge = CvBridge()
        pass

    def save_image(self, image, filename):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(filename, cv2_img)

    def setup_location(self):
        timestamp = datetime.utcnow()
        timestr = timestamp.isoformat().replace(":", "")
        self.current_folder = os.path.join(self.storage_location, timestr)
        if not os.path.isdir(self.current_folder):
            os.makedirs(self.current_folder)
        filename = os.path.join(self.current_folder, self.csv_filename)
        self.csv_file = open(filename, 'w')
        rospy.loginfo("Writing %s", filename)


    def set_storage(self, storage):
        self.storage_location = storage
        rospy.loginfo("storage location %s", self.storage_location)

    def on_camera(self, image):
        if self.is_recording == False or (abs(self.throttle) < self.vel_treshold):
            return
        now = time()
        if( (now - self.last_sample) * 1000.0 < self.sample_interval_ms):
            return
        rospy.loginfo("is_recoridng %s, throttle %d", self.is_recording, self.throttle)

        self.last_sample = now
        #image_name = "".join([uuid.uuid4().hex,'.jpg'])
        image_name = "".join(["{:05d}".format(self.image_counter),".jpg"])
        self.image_counter += 1
        full_path_name = os.path.join(self.current_folder, image_name)
        self.save_image(image, full_path_name)
        self.data.append([image_name,self.steering, self.throttle])
        self.throttle = 0 # wait to receive a Twist msg with throttle != 0 to save the next sample
        
    def on_twist(self, data):
        self.steering = data.angular.z
        self.throttle = data.linear.x
        rospy.loginfo("steering %s, throttle %s",self.steering, self.throttle)
    
    def on_shutdown(self):
        self.stop_recording()

    def handle_recording(self, req):
        if(req.status):
            self.start_recording()
        else:
            self.stop_recording()
        return recordingResponse()
    def stop_recording(self):
        if not self.is_recording:
            return
        with self.csv_file:
            writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(['image','steering', 'throttle'])
            writer.writerows(self.data)
            self.data = []
        self.is_recording = False
        rospy.loginfo("stop recording")
    
    def start_recording(self):
        if(self.is_recording):
            return
        self.throttle = 0 # Wait for throttle to be != to acquire a sample        
        self.setup_location()
        self.is_recording = True
        rospy.loginfo("start recording")

def run(trainer_node):
    camera_topic = rospy.get_param("/trainer/camera_topic")
    manual_twist_topic = rospy.get_param("/trainer/manual_twist_topic")
    storage = rospy.get_param("/trainer/storage")
    trainer_node.set_storage(storage)
    if not os.path.isdir(storage):
        rospy.logerror("/trainer/storage is not set to an existing, valid path")
        return
    rospy.init_node('trainer')
    rospy.Subscriber(camera_topic, Image, trainer_node.on_camera, queue_size=10)
    rospy.Subscriber(manual_twist_topic, Twist, trainer_node.on_twist, queue_size=10)
    rospy.Service('recording', recording, trainer_node.handle_recording)
    rospy.on_shutdown(trainer_node.on_shutdown)
    rospy.spin()

if __name__ == '__main__':
    trainerNode = trainer()
    run(trainerNode)
