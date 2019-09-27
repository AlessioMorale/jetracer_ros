#!/usr/bin/env python
import time
import rospy
from jetracer_ros.srv import recording
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import csv
import datetime
import uuid
class trainer:

    is_recording  = False
    csv_filename = "training.csv"
    storage_location = None
    current_folder = None
    csv_file = None
    steering = 0
    throttle = 0

    def __init__(self):
        pass
    def setup_location(self):
        timestamp = datetime.utcnow() 
        timestr = timestamp.isoformat().replace(":", "")
        self.current_folder = os.path.join(self.storage_location, timestr)
        if not os.path.isdir(self.current_folder):
            os.makedirs(self.current_folder)
        filename = os.path.join(self.current_folder, csv_filename)
        self.csv_file = open(filename, 'w')
        rospy.loginfo("Writing %s", filename)


    def set_storage(self, storage):
        self.storage = storage

    def on_camera(self, image):
        if self.is_recording == True:
            image = cv2.imdecode(np.fromstring(image.data, np.uint8), cv2.IMREAD_COLOR)
            image_name = String.join([uuid.uuid4().hex,'.jpg'])
            full_path_name = os.path.join(self.folder_location, image_name)
            cv2.imwrite(full_path_name, image)
            self.data.append([image_name,self.steering, self.throttle])
        
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
    def stop_recording(self):
        if not self.is_recording:
            return
        with self.data_file:
            writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(['image','steering', 'throttle'])
            writer.writerows(self.data)
        self.is_recording = False
        rospy.loginfo("stop recording")
    
    def start_recording(self):
        if(self.is_recording):
            return
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
