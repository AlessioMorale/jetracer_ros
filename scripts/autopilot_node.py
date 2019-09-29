#!/usr/bin/env python3
import time
import rospy
import torchvision.transforms as transforms
import torch
import PIL.Image
from torch2trt import TRTModule
import torch.nn.functional as F
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

class autopilot:
    def __init__(self):
        self.throttle_threshold = 0
        self.steering = 0
        self.throttle = 0
        self.control_interval_ms = 0
        self.model_file = None
        self.model_trt = None
        self.transforms = None
        self.bridge = CvBridge()
        self.twist_publisher = None
        self.is_running = True
        self.preproc_mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.preproc_std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        self.throttle_timeout = 100
        self.throttle_timeout_counter = 0

    def preprocess(self, image):
        device = torch.device('cuda')
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(self.preproc_mean[:, None, None]).div_(
            self.preproc_std[:, None, None])
        return image[None, ...]

    def set_model(self, model):
        self.model_file = model
        self.model= TRTModule()
        self.model.load_state_dict(torch.load(self.model_file))
        self.twist_msg = Twist()
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        


    def set_twist_publisher(self, publisher):
        self.twist_publisher = publisher

    def set_control_interval(self, control_interval):
        self.control_interval_ms = control_interval

    def set_throttle_threshold(self, threshold):
        self.throttle_threshold = threshold


    def set_twist_publisher(self, twist_publisher):
        self.twist_publisher = twist_publisher

    def on_twist(self, data):
        rospy.loginfo(data)
        self.steering = data.angular.z
        self.throttle = data.linear.x
        self.throttle_timeout_counter = self.throttle_timeout

    def on_camera(self, image):
        if self.is_running == False:
            return
        if abs(self.throttle) < self.throttle_threshold:
            if(self.throttle_timeout_counter > 0):
                self.throttle_timeout_counter -= 1;
            else:
                self.twist_msg.linear.x = 0
                self.twist_publisher.publish(self.twist_msg)
            return
        now = time()
#        if((now - self.last_run) * 1000.0 < self.control_interval_ms):
#            return
        self.last_run = now
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        preprocessed = self.preprocess(cv2_img).half()
        output = self.model(preprocessed).detach().cpu().numpy().flatten()
        steer = float(output[0])
        thr = float(output[1])
        self.twist_msg.angular.z = steer
        self.twist_msg.linear.x = self.throttle
        self.throttle = 0  # wait to receive a Twist msg with throttle != 0 to save the next sample
        self.twist_publisher.publish(self.twist_msg)
    
    def on_shutdown(self):
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_publisher.publish(self.twist_msg)
        self.twist_publisher.publish(self.twist_msg)

def run(autopilot_node):
    camera_topic = rospy.get_param("/autopilot/camera_topic")
    manual_twist_topic = rospy.get_param("/autopilot/manual_twist_topic")

    control_interval = rospy.get_param("/autopilot/control_interval")
    autopilot_node.set_control_interval(control_interval)

    throttle_threshold = rospy.get_param("/autopilot/throttle_threshold")
    autopilot_node.set_throttle_threshold(throttle_threshold)

    model = rospy.get_param("/autopilot/model")
    autopilot_node.set_model(model)
    published_twist_topic = rospy.get_param("/autopilot/published_twist")

    rospy.init_node('autopilot')
    rospy.Subscriber(camera_topic, Image,
                     autopilot_node.on_camera, queue_size=5)
    rospy.Subscriber(manual_twist_topic, Twist,
                     autopilot_node.on_twist, queue_size=10)
    twist_publisher = rospy.Publisher(published_twist_topic, Twist, queue_size=10)
    autopilot_node.set_twist_publisher(twist_publisher)

    rospy.on_shutdown(autopilot_node.on_shutdown)
    rospy.spin()

if __name__ == '__main__':
    autopilot_node = autopilot()
    run(autopilot_node)
