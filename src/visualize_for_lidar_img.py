#!/usr/bin/env python2

import rospy
import cv2
import os
import glob
from cv_bridge import CvBridge
from utils import *
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
import kitti_data_utils
from visualization_msgs.msg import MarkerArray, Marker
import yaml


img_path = rospy.get_param("/setting_path/img_path")
pt_path = rospy.get_param("/setting_path/pt_path")
gt_path = rospy.get_param("/setting_path/gt_path")
calib_path = rospy.get_param("/setting_path/calib_path")


gt_class = rospy.get_param("/gt_class") # to select which type object you want to see
# gt_class = [val for val in dictionary['gt_class']]

def get_gt_3dboxes_corner(objects, calib):

    ''' Show all LiDAR points.
        Draw 3d box in LiDAR point cloud (in velo coord system) '''

    box3d_corners = []
    object_types = []

    for obj in objects:
        
        if obj.type not in  gt_class: 
            continue
        # Draw 3d bounding box
        box3d_pts_2d, box3d_pts_3d = kitti_data_utils.compute_box_3d(obj, calib.P)
        box3d_pts_3d_velo = calib.project_rect_to_velo(box3d_pts_3d)
        box3d_corners.append(box3d_pts_3d_velo)
        object_types.append(obj.type)

    return np.array(box3d_corners).reshape(-1, 8, 3), object_types

if __name__ == "__main__":

    
    rospy.init_node("kitti_node", anonymous=True)

    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    lidar_pub = rospy.Publisher('kitti_lidar', PointCloud2, queue_size=10)
    car_model = rospy.Publisher('car_model', Marker, queue_size=10)
    gt_box3d_pub = rospy.Publisher('gt', MarkerArray, queue_size=10)


    file_num = len(os.listdir(pt_path))

    bridge = CvBridge()   
    rate = rospy.Rate(10)
    frame = 0

    while not rospy.is_shutdown():

        gt_objects = []
   
        index = '%006d' % frame

        image_file = os.path.join(img_path , index  +".png")
        velo_file = os.path.join(pt_path, index  + ".bin")

        calib_file = os.path.join(calib_path, index + ".txt")
        calib = kitti_data_utils.Calibration(calib_file)

        gt_file = os.path.join(gt_path, index  + ".txt")
        gt_label = open(gt_file, 'r').readlines()
        # to generate objects in a label
        for gt_line in gt_label:
            gt_objects = np.append(gt_objects, kitti_data_utils.Object3d(gt_line.strip()))
        

        gt_coners_3d, gt_object_types = get_gt_3dboxes_corner(gt_objects, calib)
        img = read_img(image_file)
        pc  = read_point_cloud(velo_file)

       


        publish_car_model(car_model)
        publish_camera(cam_pub, bridge, img)
        publish_point_cloud(lidar_pub, pc)
        publish_gt_3dbox(gt_box3d_pub, gt_coners_3d)

        rate.sleep()
      
        frame = frame + 1
        if frame % file_num == 0:
            frame = 0