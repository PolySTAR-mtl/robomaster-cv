#!/usr/bin/env python3
"""
@file track.py

@author Tzu-yi Chiu <tzuyi.chiu@gmail.com>
"""

import os
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from collections import defaultdict

from sensor_msgs.msg import Image
from detection.msg import Detections
from detection.msg import Detection as DetectionROS
from tracking.msg import Tracklets, Tracklet

from motpy import MultiObjectTracker, Track 
from motpy import Detection as DetectionMOT
from motpy.tracker import Tracker as TrackerMOT

def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass

class Tracking():
    def __init__(self, rate=24):
        # ROS
        node = rclpy.create_node('tracking', anonymous=False)
        rclpy.get_global_executor().add_node(node)
        self.pub = node.create_publisher( Tracklets, 'tracking/tracklets', queue_size=1)

        # motpy - no confidence with the motion model (0.1)
        #       - more confidence with detections (5000)
        model_spec = {'order_pos': 1, 'dim_pos': 2,
                      'order_size': 0, 'dim_size': 2,
                      'q_var_size': 100., 'r_var_size': 10.,
                      'q_var_pos': 5000., 'r_var_pos': 0.1}

        matching_fn_kwargs = {'min_iou': 0.5}
        
        self.model_spec = model_spec
        self.rate = rate
        self.no_detection = True

        # One MOT per class
        self.mots = defaultdict(lambda: MultiObjectTracker(
            dt=1/rate,
            model_spec=model_spec, 
            matching_fn_kwargs=matching_fn_kwargs,
            active_tracks_kwargs={'max_staleness_to_positive_ratio': 2000}
            # no early stage, one track per tracker, always active
        ))

    def detections_callback(self, detections_ros: Detections) -> Tracklets:
        self.no_detection = False
        class2detections = defaultdict(list) # Keys are classes (c)
        class2confs = defaultdict(list)      # Keys are classes (c)

        for detection_ros in detections_ros.detections:
            detection, clss = self.det_ros2mot(detection_ros)
            class2detections[clss].append(detection)

        tracklets = []
        for clss in set(self.mots.keys()).union(class2detections.keys()):
            detections = class2detections[clss]
            mot = self.mots[clss]
            tracks = mot.step(detections)
            for track in tracks:
                tracklet = self.trk_mot2ros(track=track, clss=clss)
                tracklets.append(tracklet)

        tracklets = Tracklets(tracklets=tracklets)
        self.pub.publish(tracklets)
        return tracklets

    def det_ros2mot(self, detection: DetectionROS) -> DetectionMOT:
        box = np.array([detection.x, detection.y, 
                        detection.x + detection.w, detection.y + detection.h])
        return DetectionMOT(box=box, score=detection.score), detection.clss

    def trk_mot2ros(self, track: Track, clss: int) -> Tracklet:
        xmin, ymin, xmax, ymax = track.box
        x, y, w, h = xmin, ymin, xmax - xmin, ymax - ymin
        tracklet = Tracklet(id=track.id, x=x, y=y, w=w, h=h, 
                            clss=clss, score=track.score)
        return tracklet

def main():
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()

    tracking = Tracking()
    node = rclpy.create_node('talker')
    rclpy.get_global_executor().add_node(node)
    sub = node.create_subscriber(Detections, 'detection/detections', tracking.detections_callback)
    rate = node.createRate(tracking.rate)
    while rclpy.ok():
        rate.sleep()
        if tracking.no_detection:
            tracking.detections_callback(Detections(detections=[]))
        else:
            tracking.no_detection = True

    t.join()
