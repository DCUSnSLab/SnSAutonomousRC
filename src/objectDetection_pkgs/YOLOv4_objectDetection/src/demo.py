#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :
'''

# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :
'''


# import sys
# import time
# from PIL import Image, ImageDraw
# from models.tiny_yolo import TinyYoloNet

from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import message_filters
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import timeit
import statistics
import pyzed.sl as sl
#from threading import Lock, Thread


"""hyper parameters"""
use_cuda = True
exit_signal = False
width = 1280
height = 720
image_np_global = np.zeros([width, height, 3], dtype=np.uint8)
depth_np_global = np.zeros([width, height, 4], dtype=np.float64)
#lock = Lock()
#new_data = False

def zedCapture_thread_func():
    global image_np_global, depth_np_global, exit_signal, new_data
    zed = sl.Camera()
    zed_id = 0

    input_type = sl.InputType()

    input_type.set_from_camera_id(zed_id)
    init_params = sl.InitParameters(input_t=input_type)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.svo_real_time_mode = False

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    py_transform = sl.Transform()
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
    err = zed.enable_positional_tracking(tracking_parameters)

    #zed_pose = sl.Pose()
    zed_sensors = sl.SensorsData()

    if err != sl.ERROR_CODE.SUCCESS:
        print(1)

    image_mat = sl.Mat()
    depth_mat = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()
    #camera_info = zed.get_camera_information()

    #viewer = gl.GLViewer()
    #viewer.init(camera_info.camera_model)

    image_size = sl.Resolution(width, height)

    while not rospy.is_shutdown():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image_mat, sl.VIEW.LEFT, resolution=image_size)
            zed.retrieve_measure(depth_mat, sl.MEASURE.XYZRGBA, resolution=image_size)
            #lock.acquire()
            image_np_global = load_image_into_numpy_array(image_mat)
            depth_np_global = load_depth_into_numpy_array(depth_mat)
            new_data = True
            #lock.release()
            detect_cv2_camera(image_np_global)

            '''
            #zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            #zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)

            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            #print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz,zed_pose.timestamp.get_milliseconds()))

            py_translation = sl.Translation()
            '''

            zed_tracking(zed)
            zed_imu(zed, zed_sensors)
    zed.close()

def zed_tracking(zed):
    zed_pose = sl.Pose()

    zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
    py_translation = sl.Translation()

    tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
    ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
    tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
    #print(zed_pose.get_translation((py_translation).get()))
    #print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz,zed_pose.timestamp.get_milliseconds()))

    py_orientation = sl.Orientation()

    #zed_tf = []
    ox = round(zed_pose.get_orientation(py_orientation).get()[0], 3)
    oy = round(zed_pose.get_orientation(py_orientation).get()[1], 3)
    oz = round(zed_pose.get_orientation(py_orientation).get()[2], 3)
    ow = round(zed_pose.get_orientation(py_orientation).get()[3], 3)

    #print("Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))

    #zed_tf.append(ox)
    #zed_tf.append(oy)
    #zed_tf.append(oz)
    #zed_tf.append(ow)
    #print(zed_tf)

def zed_imu(zed, zed_sensors):
    zed_imu = zed_sensors.get_imu_data()
    zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)
    # Display the IMU acceleratoin
    acceleration = [0, 0, 0]
    zed_imu.get_linear_acceleration(acceleration)
    ax = round(acceleration[0], 3)
    ay = round(acceleration[1], 3)
    az = round(acceleration[2], 3)
    #print("IMU Acceleration: Ax: {0}, Ay: {1}, Az {2}\n".format(ax, ay, az))

    # Display the IMU angular velocity
    a_velocity = [0, 0, 0]
    zed_imu.get_angular_velocity(a_velocity)
    vx = round(a_velocity[0], 3)
    vy = round(a_velocity[1], 3)
    vz = round(a_velocity[2], 3)
    #print("IMU Angular Velocity: Vx: {0}, Vy: {1}, Vz {2}\n".format(vx, vy, vz))

    # Display the IMU orientation quaternion
    zed_imu_pose = sl.Transform()
    ox = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[0], 3)
    oy = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[1], 3)
    oz = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[2], 3)
    ow = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[3], 3)
    #print("IMU Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))
'''
def camera_tracking(zed):
    zed_pose = sl.Pose()
    py_translation = sl.Translation()
    pose_data = sl.Transform()

    zed = zed
    py_translation = py_translation
    tracking_state = zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)

    if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
        rotation = zed_pose.get_rotation_vector()
        translation = zed_pose.get_translation(py_translation)
        text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
        text_translation = str(
            (round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
        pose_data = zed_pose.pose_data(sl.Transform())
        print(pose_data)
   #print('pose_data', pose_data.get_translation().get())
'''


def load_image_into_numpy_array(image):
    ar = image.get_data()
    ar = ar[:, :, 0:3]
    (im_height, im_width, channels) = image.get_data().shape
    return np.array(ar).reshape((im_height, im_width, 3)).astype(np.uint8)

def load_depth_into_numpy_array(depth):
    ar = depth.get_data()
    ar = ar[:, :, 0:4]
    (im_height, im_width, channels) = depth.get_data().shape
    return np.array(ar).reshape((im_height, im_width, channels)).astype(np.float32)


def detect_cv2_camera(img):
    #cap = cv2.VideoCapture(0)

    # cap = cv2.VideoCapture("./test.mp4")
    #cap.set(3, 1280)
    #cap.set(4, 720)
    #print("Starting the YOLO loop...")

    #lock.acquire()
    num_classes = m.num_classes
    if num_classes == 20:
        namesfile = '/home/jaehoon/SnSAutonomousRC/src/objectDetection_pkgs/YOLOv4_objectDetection/src/data/voc.names'
    elif num_classes == 80:
        namesfile = '/home/jaehoon/SnSAutonomousRC/src/objectDetection_pkgs/YOLOv4_objectDetection/src/data/coco.names'
    else:
        namesfile = '/home/jaehoon/SnSAutonomousRC/src/objectDetection_pkgs/YOLOv4_objectDetection/src/data/x.names'
    class_names = load_class_names(namesfile)

    #ret, img = cap.read()
    start_t = timeit.default_timer()
    sized = cv2.resize(img, (m.width, m.height))
    sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)

    start = time.time()
    boxes = do_detect(m, sized, 0.4, 0.6, use_cuda)

    finish = time.time()
    #print('Predicted in %f seconds.' % (finish - start))

    result_img = plot_boxes_cv2(img, boxes[0], savename=None, class_names=class_names)

    #blank_img = np.zeros((1280, 720, 3), dtype=np.uint8)

    #time.sleep(0.3)
    thickness = 1
    for i in range(len(boxes[0])):
        #print('abcd',boxes)
       #print()
        #print('sdfs',boxes[0][i][0])
        if boxes[0][i][6] == 0:
            x1 = int(float(boxes[0][i][0]) * 1280)
            y1 = int(float(boxes[0][i][1]) * 720)
            x2 = int(float(boxes[0][i][2]) * 1280)
            y2 = int(float(boxes[0][i][3]) * 720)
            #segment = bridge.cv2_to_imgmsg(segment)

            center_x = (x2+x1) / 2
            center_y = (y2+y1) / 2
            box_width = (x2-x1)
            box_height = (y2-y1)


            bounds = [center_x, center_y, box_width, box_height]
            label = boxes[0][i][6]

            x, y, z = get_object_depth(depth_np_global, bounds)
            distance = math.sqrt(x * x + y * y + z * z)
            distance = "{:.2f}".format(distance)
            print(distance)
            cv2.putText(result_img, str(distance) + " m",
                        (x2 + (thickness * 4), y1 + (thickness * 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)


        #print(distance)
    terminate_t = timeit.default_timer()
    FPS = int(1./(terminate_t-start_t))

    cv2.imshow('Yolo demo', result_img)
    cv2.waitKey(1)
    #lock.release()
    print('FPS :',FPS)


def get_object_depth(depth, bounds):
    '''
    Calculates the median x, y, z position of top slice(area_div) of point cloud
    in camera frame.
    Arguments:
        depth: Point cloud data of whole frame.
        bounds: Bounding box for object in pixels.
            bounds[0]: x-center
            bounds[1]: y-center
            bounds[2]: width of bounding box.
            bounds[3]: height of bounding box.
    Return:
        x, y, z: Location of object in meters.
    '''
    area_div = 2

    x_vect = []
    y_vect = []
    z_vect = []
    print('bounds:', bounds)

    for j in range(int(bounds[0] - area_div), int(bounds[0] + area_div)):
        for i in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
            z = depth[i, j, 2]
            #z = depth[i, j]
            if not np.isnan(z) and not np.isinf(z):
                x_vect.append(depth[i, j, 1])
                y_vect.append(depth[i, j, 0])
                #x_vect.append(depth[i, j])
                #y_vect.append(depth[i, j])
                z_vect.append(z)
    try:
        x_median = statistics.median(x_vect)
        y_median = statistics.median(y_vect)
        z_median = statistics.median(z_vect)
    except Exception:
        x_median = -1
        y_median = -1
        z_median = -1
        pass

    return x_median, y_median, z_median


def region_of_interest(img, x1, y1, x2, y2):
    polygons = np.array([
        [(x1, y2), (x1, y1), (x2, y1), [x2, y2]]
    ])
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, polygons, (255,255,255))
    segment = cv2.bitwise_and(img, mask)
    return segment

def test():
    #lock.acquire()
    while exit_signal:
        print(1)
    #lock.release()

def get_args():
    parser = argparse.ArgumentParser('Test your image or video by trained model.')
    parser.add_argument('-cfgfile', type=str, default='/home/jaehoon/SnSAutonomousRC/src/objectDetection_pkgs/YOLOv4_objectDetection/src/model/yolov4-tiny.cfg',
                        help='path of cfg file', dest='cfgfile')
    parser.add_argument('-weightfile', type=str,
                        default='/home/jaehoon/SnSAutonomousRC/src/objectDetection_pkgs/YOLOv4_objectDetection/src/model/yolov4-tiny.weights',
                        help='path of trained model.', dest='weightfile')
    parser.add_argument('-imgfile', type=str,
                        default='./data/dog.jpg',
                        help='path of your image file.', dest='imgfile')
    #parser.add_argument('-')
    args = parser.parse_args()
    return args

def modelConfig(m):
    m.print_network()
    m.load_weights(weightfile)
    #print('Loading weights from %s... Done!' % (weightfile))
    if use_cuda:
        m.cuda()

if __name__ == '__main__':
    rospy.init_node("Yolo_Test", anonymous=True)
    args = get_args()
    cfgfile = args.cfgfile
    weightfile = args.weightfile
    m = Darknet(cfgfile)
    modelConfig(m)

    #capture_thread = Thread(target=zedCapture_thread_func)
    #capture_thread.start()
    zedCapture_thread_func()
