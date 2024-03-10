#!/usr/bin/env python3

# Based on code from
# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import numpy as np
import cv2 # OpenCV library
import traceback
import yaml

from copy import deepcopy

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.time import Time
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ball_detector_msgs.msg import Ball, BallList
from simple_ball_detector.cv2_demo_image_proc import ImageProc

class SimpleBallDectector(Node):
    """
    Create an SimpleBallDectector class, which is a subclass of the ROS Node class.
    """
    def __init__(self, name='ball_detector'):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__(name)

        self.declare_parameter('target_frame', "map")       # Desired frame
        self.declare_parameter('robot_frame', "base_link")
        self.declare_parameter('camera_frame', "camera_rgb_optical_frame")
        self.declare_parameter('ball_radius', 0.0254) # ball radius to be detected (assume same size)
        self.declare_parameter('z_offset', 0.0) # z offset for projection
        self.declare_parameter('min_col_pixel', -1) # Minimum column(x-axis) of image
        self.declare_parameter('max_col_pixel', 5000) # Maximum column(x-axis) of image
        self.declare_parameter('min_row_pixel', -1) # Minimum row (y-axis) of image (matrix coordinates)
        self.declare_parameter('max_row_pixel', 5000) # Maximum row (y-axis) of image (matrix coordinates)
        self.declare_parameter('min_range', -1.0) # Minimum range to allow detection (meters)
        self.declare_parameter('max_range', 100.0) # Minimum range to allow detection (meters along camera z-axis)
        self.declare_parameter('min_box_ratio', 0.75) # ratio of size of rectangle
        self.declare_parameter('min_box_size', 5) # minimum size of bounding rectangle in pixels
        self.declare_parameter('min_radius', 2)  # minimum size of minEnclosingCircle in pixels

        self.declare_parameter('marker_alpha', 0.4) # RViz marker array (use negative to trigger no publish)
        self.declare_parameter('marker_lifetime_seconds', 5)
        self.declare_parameter('marker_topic', 'markers_out') # RViz marker array (use negative to trigger no publish)
        self.declare_parameter('augmented_image_topic', 'augmented_image')
        self.declare_parameter('image_in_topic', 'image_in')
        self.declare_parameter('camera_info_topic', 'info_in')
        self.declare_parameter('detected_balls_topic', 'detected_balls')
        self.declare_parameter('average_focal_length', 390.0) # Will be overwritten by any camera info message received
        self.declare_parameter('desired_update_period', 0.19) # ~5hz default
        self.declare_parameter('tf_wait_time_seconds', 0.010)  # Wait time for transforms in seconds
        self.declare_parameter('filter_map_file', '')  # color name to HSV limits

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.true_ball_radius =  self.get_parameter('ball_radius').get_parameter_value().double_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value

        self.min_col_pixel = self.get_parameter('min_col_pixel').get_parameter_value().integer_value # Minimum column(x-axis) of image
        self.max_col_pixel = self.get_parameter('max_col_pixel').get_parameter_value().integer_value  #  Maximum column(x-axis) of image
        self.min_row_pixel = self.get_parameter('min_row_pixel').get_parameter_value().integer_value  #  Minimum row (y-axis) of image (matrix coordinates)
        self.max_row_pixel = self.get_parameter('max_row_pixel').get_parameter_value().integer_value  #  Maximum row (y-axis) of image (matrix coordinates)
        self.ball_radius = self.get_parameter('ball_radius').get_parameter_value().integer_value  #  ball radius to be detected (assume same size)
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value  #  Minimum range to allow detection (meters)
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value  #  Minimum range to allow detection (meters)
        self.min_box_ratio = self.get_parameter('min_box_ratio').get_parameter_value().double_value # ratio of size of rectangle
        self.min_box_size = self.get_parameter('min_box_size').get_parameter_value().integer_value # minimum size of bounding rectangle
        self.min_radius = self.get_parameter('min_radius').get_parameter_value().integer_value # minimum size of circle in pixels

        self.marker_alpha = self.get_parameter('marker_alpha').get_parameter_value().double_value  #  RViz marker array (use negative to trigger no publish)
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value  #  RViz marker array topic (empty "" to ignore)
        self.augmented_image_topic = self.get_parameter('augmented_image_topic').get_parameter_value().string_value  #  RViz marker array topic (empty "" to ignore)
        self.image_input_topic = self.get_parameter('image_in_topic').get_parameter_value().string_value  #  RViz marker array topic (empty "" to ignore)
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value  #  RViz marker array topic (empty "" to ignore)
        self.detected_balls_topic = self.get_parameter('detected_balls_topic').get_parameter_value().string_value  #  RViz marker array topic (empty "" to ignore)

        self.average_focal_length = self.get_parameter('average_focal_length').get_parameter_value().double_value

        self.desired_update_period = self.get_parameter('desired_update_period').get_parameter_value().double_value
        self.marker_lifetime = rclpy.duration.Duration(seconds=self.get_parameter('marker_lifetime_seconds').get_parameter_value().integer_value).to_msg()
        self.tf_wait_time = rclpy.duration.Duration(nanoseconds=int(1e9*self.get_parameter('tf_wait_time_seconds').get_parameter_value().double_value))

        self.filter_map = None
        filter_map_file = self.get_parameter('filter_map_file').get_parameter_value().string_value
        if filter_map_file != '':
            try:
                with open(filter_map_file, 'rt') as fin:
                    self.filter_map = yaml.safe_load(fin)['filter_map']

                for item in self.filter_map:
                    for key in item:
                        if key != 'name':
                            item[key] = int(item[key]) # Convert limits to int

            except Exception as exc:
                self.get_logger().error(f" Error processing filter map {filter_map_file}!\n{exc}")

        if self.filter_map is None:
            self.get_logger().info(' No filter map file!')

        self._name = self.get_name() # Use node name

        self.get_logger().info(f"Using OpenCV version {cv2.__version__}")

        # Subscribe to images that we should be processing
        self._subscribe()

        # Required publisher
        self._balls_publisher = self.create_publisher(BallList, self.detected_balls_topic, 15)


        # Create the optional publishers.
        self._image_publisher = None
        self._info_publisher = None
        self._marker_publisher = None
        if self.augmented_image_topic != "":
            self._image_publisher = self.create_publisher(Image, self.augmented_image_topic+"/image_raw", qos_profile=qos_profile_sensor_data)
            self._info_publisher = self.create_publisher(CameraInfo, self.augmented_image_topic+"/camera_info", qos_profile=qos_profile_sensor_data)

        if self.marker_alpha > 0.0 and self.marker_topic != "":
            self._marker_publisher = self.create_publisher(MarkerArray, 'markers_out', qos_profile=qos_profile_sensor_data)

        # Used to convert between ROS and OpenCV images
        self._br = CvBridge()

        self._balls_publisher # prevent PyLint unused variable warning
        self._image_publisher # prevent PyLint unused variable warning
        self._image_subscription # prevent PyLint unused variable warning
        self._info_publisher # prevent PyLint unused variable warning
        self._info_subscription # prevent PyLint unused variable warning

        self._image_proc = ImageProc()
        self._camera_info = None
        self._balls_to_publish = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _subscribe(self):

        # Values only used in the image callback
        self._prior_data_timestamp = self.get_clock().now()
        self._prior_processing_timestamp = self.get_clock().now()
        self._skipped_images = 0


        self._image_subscription = self.create_subscription(Image, self.image_input_topic,
                                                            self._image_callback,
                                                            qos_profile=qos_profile_sensor_data)

        self._info_subscription = self.create_subscription(CameraInfo, self.camera_info_topic,
                                                            self._info_callback,
                                                            qos_profile=qos_profile_sensor_data)

    def _dump_parameters(self):
        # Print parameter data
        self.get_logger().info(80*"=")
        self.get_logger().info(f"Parameters:")
        keys = [key for key in self.__dict__ if key[0] != '_' and key != 'filter_map']
        keys.sort()
        for key in keys:
            self.get_logger().info(f"   {key}: {self.__dict__[key]}")
        self.get_logger().info(80*"=")
        if self.filter_map:
            self.get_logger().info(f"Filter map values:")
            for item in self.filter_map:
                self.get_logger().info(45*"-")
                for key, val in item.items():
                    self.get_logger().info(f"       {key} : {val}")
            self.get_logger().info(80*"=")

    def _info_callback(self, data):
        """
        Camera Info callback function.
        Just relay the incoming informatin if any
        """
        self._camera_info = data
        self.average_focal_length = 0.5*(self._camera_info.p[0] + self._camera_info.p[5])


        if self._info_publisher:
            # self.get_logger().info(f"Forwarding camera info message with average focal length of {self.average_focal_length}")
            self._info_publisher.publish(data)

    def project_ball(self, center, radius, rows, cols):
        # Calculate pixels relative to the center reference frame
        x = center[0] - cols//2
        y = center[1] - rows//2

        z = -99.99  # cannot have negative values in projection, so this is just flag

        # convert from pixels to meters
        # projection ratio based on ball radius size and average focal length (from parameter or camera info)
        projection_ratio = self.true_ball_radius/radius
        z = self.average_focal_length * projection_ratio
        z -= self.z_offset

        if z < self.min_range or z > self.max_range:
            self.get_logger().warning(f"projected z is out of range {self.min_range} < {z} < {self.max_range} for center={center} radius={radius}")
            return None

        # Assuming that I can measure radius to +/- 2 pixels, what is expected error here?

        # convert x,y pixels to fraction given focal length
        x /= self.average_focal_length
        y /= self.average_focal_length

        x *= z # convert from image plane to object plane
        y *= z
        return Point(x=x, y=y, z=z)

    @staticmethod
    def _transform_matrix(transform):
        r00 = 1 - 2 * transform.rotation.y ** 2 - 2 * transform.rotation.z ** 2
        r01 = 2 * transform.rotation.x * transform.rotation.y - 2 * transform.rotation.z * transform.rotation.w
        r02 = 2 * transform.rotation.x * transform.rotation.z + 2 * transform.rotation.y * transform.rotation.w

        r10 = 2 * transform.rotation.x * transform.rotation.y + 2 * transform.rotation.z * transform.rotation.w
        r11 = 1 - 2 * transform.rotation.x ** 2 - 2 * transform.rotation.z ** 2
        r12 = 2 * transform.rotation.y * transform.rotation.z - 2 * transform.rotation.x * transform.rotation.w

        r20 = 2 * transform.rotation.x * transform.rotation.z - 2 * transform.rotation.y * transform.rotation.w
        r21 = 2 * transform.rotation.y * transform.rotation.z + 2 * transform.rotation.x * transform.rotation.w
        r22 = 1 - 2 * transform.rotation.x ** 2 - 2 * transform.rotation.y ** 2

        transform_matrix = np.array([[r00, r01, r02, transform.translation.x],
                                     [r10, r11, r12, transform.translation.y],
                                     [r20, r21, r22, transform.translation.z],
                                     [0.0, 0.0, 0.0, 1.0]])
        return transform_matrix

    @staticmethod
    def _transform_point(transform_matrix, point):

        camera_point = np.array([[point.x],
                                 [point.y],
                                 [point.z],
                                 [1.0]])

        target_point = np.matmul(transform_matrix, camera_point)
        return Point(x=target_point[0][0], y=target_point[1][0], z=target_point[2][0])


    def _image_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        #self.get_logger().info(f'Receiving video frame at t={data.header.stamp}')

        try:
            image_timestamp = Time(seconds=data.header.stamp.sec, nanoseconds=data.header.stamp.nanosec, clock_type=rclpy.clock.ClockType.ROS_TIME)
            #self.get_logger().info(f"timestamp check p={image_timestamp} {self._prior_processing_timestamp} {self._prior_data_timestamp}")
            elapsed_nanoseconds = (image_timestamp - self._prior_processing_timestamp).nanoseconds
            #if elapsed_nanoseconds < 0:
            #    #self.get_logger().info(f'Skipping image - wait for newer data from after we started prior processing : {elapsed_nanoseconds/1e9:.6f} seconds')
            #    self._skipped_images += 1
            #    return

            elapsed_seconds = (image_timestamp - self._prior_data_timestamp).nanoseconds/1e9
            if elapsed_seconds < self.desired_update_period:
                #self.get_logger().info(f'Skipping image - too close to prior data {elapsed_seconds:.6f} < {self.desired_update_period}')
                self._skipped_images += 1
                return

            # Log when we start so we can avoid processing too many old messages that are already in the queue
            self._prior_processing_timestamp = self.get_clock().now()
            elapsed_seconds = (image_timestamp - self._prior_processing_timestamp).nanoseconds/1e9
            if self._skipped_images > 0:
                self.get_logger().info(f"Skipped {self._skipped_images} prior images - this image is {elapsed_seconds:.6f} seconds old!")

            self._skipped_images = 0

        except Exception as exc:
            self.get_logger().error(f' time stamp check error:\n{exc}')
            self.get_logger().info(f"         timestamp check p={image_timestamp} {self._prior_processing_timestamp} {self._prior_data_timestamp}")

        try:
            if data.header.frame_id != self.camera_frame:
                self.get_logger().warning(f'Using actual image frame {data.header.frame_id} not {self.camera_frame} ...')
                self.camera_frame = data.header.frame_id

            # Convert ROS Image message to OpenCV image
            current_frame = self._br.imgmsg_to_cv2(data, "bgr8")

            if self._image_publisher:
                augmented_image = current_frame.copy()
            else:
                augmented_image = None

            rows, cols, channels = current_frame.shape
            min_col = max(0,    self.min_col_pixel)
            max_col = min(cols, self.max_col_pixel)
            min_row = max(0,    self.min_row_pixel)
            max_row = min(rows, self.max_row_pixel)

            image_size = (rows, cols)
            window = (min_col, min_row, max_col, max_row)

            #self.get_logger().info(f"input image of {rows} x {cols} x {channels} received")
            if min_col > 0 or \
               max_col < cols or \
               min_row > 0 or \
               max_row < rows:
                #self.get_logger().info(f" use window {min_row},{min_col} : {max_row}, {max_col} with max size = {rows}, {cols}")
                current_frame = current_frame[min_row:max_row, min_col:max_col, :]

        except Exception as e:
            self.get_logger().error(f'Failed to convert input image frame\n{e}')
            return

        try:
            # Call the basic image processing
            labeled_balls, hue, sat, val = self._image_proc.process(current_frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process input image frame\n{e}')
            return

        balls = BallList()
        balls.header = deepcopy(data.header)
        try:
            if labeled_balls is not None:
                self._process_labeled_regions(balls, labeled_balls, window, image_size, hue, sat, val, augmented_image)
            else:
                self.get_logger().info(f'No labeled regions in image frame\n')

        except Exception as e:
            self.get_logger().error(f'Failed to extract ball list from processed image \n{e}')
            self.get_logger().error(traceback.format_exc())

        # Final processing and publishing markers and augmented image
        if self._process_balls_list(balls, augmented_image):
            # Update time stamp if we publish anything
            self._prior_data_timestamp = image_timestamp


    def _process_labeled_regions(self, balls, labeled_balls, window, image_size, hue, sat, val, augmented_image):
        """
        Extract labeled balls from image
        @param balls - BallList we will fill
        @param labeled_balls - processed image containing pixels labeled with region index
        @param window - (min_col, min_row, max_col, max_row) used to window the full image
        @param image_size = (rows, cols) of full input image
        @param augmented_image - add indications of detected regions
        @return None
        """
        try:

            #self.get_logger().info(f"Begin extracting balls from input image")
            max_label = np.amax(labeled_balls)
            if max_label > 0:
                # labeled_balls has numbers for pixels,
                # so labeled_balls == 1  would get all pixels with label 1
                # self.get_logger().info(f"   extracting {max_label} balls from input image")

                min_col, min_row, max_col, max_row = window
                rows, cols = image_size

                for i in range(1, max_label+1):

                    # self.get_logger().info(f"extract ball {i} from input image")
                    logical_indices = labeled_balls == i
                    avg_hue = int(np.sum(hue[logical_indices])/np.sum(logical_indices))
                    if avg_hue < 0 or avg_hue >= 180:
                        self.get_logger().error(f"Problem with average hue for {i} {avg_hue}!")
                        continue

                    avg_sat = int(np.sum(sat[logical_indices])/np.sum(logical_indices))
                    if avg_sat < 0 or avg_sat >= 256:
                        self.get_logger().error(f"Problem with average sat for {i} {avg_sat}!")
                        continue

                    avg_val = int(np.sum(val[logical_indices])/np.sum(logical_indices))
                    if avg_val < 0 or avg_val >= 256:
                        self.get_logger().error(f"Problem with average hue for {i} {avg_hue}!")
                        continue

                    #self.get_logger().info(f"find contours of labeled {i} with average hsv={avg_hue}/{avg_sat}/{avg_val} from input image")

                    check = 0*labeled_balls
                    check[labeled_balls == i] = 255
                    #self.get_logger().info(f"   find contours of balls from input image")

                    contours_list, _ = cv2.findContours(check, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
                    if len(contours_list) > 0:
                        if len(contours_list) != 1:
                            self.get_logger().info(f"Problem with label {i} contour check!")
                            contours_list.sort(key=cv2.contourArea, reverse=True)

                        c = contours_list[0]
                        # self.get_logger().info(f"raw contour = {c.shape} {c[0:2,0,:]}")

                        # Reset contour relative to the original image frame,
                        # not the masked image processed based on min row/col
                        c[:,:,0] += min_col  # x is col in contour
                        c[:,:,1] += min_row  # y is row
                        # self.get_logger().info(f"adj contour =  {c.shape} {c[0:2,0,:]}")

                        # Get the bounding circle for the contour
                        center, radius = cv2.minEnclosingCircle(c)
                        rect = cv2.minAreaRect(c)
                        rect_center, size, angle = rect
                        ratio = min(size)/max(size) # equal side lengths for square bounding circle

                        if self._image_publisher:
                            frame = np.zeros((4, 2))
                            frame[0,1] = min_row # y is row
                            frame[0,0] = min_col # x is col in contour
                            #frame[-1,:] = frame[0,:] # close the contour
                            frame[1,1] = min_row
                            frame[1,0] = max_col-1
                            frame[2,1] = max_row-1
                            frame[2,0] = max_col-1
                            frame[3,1] = max_row-1
                            frame[3,0] = min_col
                            cv2.drawContours(augmented_image,
                                             [frame.astype(np.int32)],        # fake contours list
                                             0,              # index into fake contours list
                                             (0, 0, 0), #tuple(cv2.cvtColor(np.uint8([[(0, 0, 0)]]), cv2.COLOR_HSV2BGR)[0][0].tolist()),
                                             2)

                            # Create single pixel image and map color
                            bgr_color = tuple(cv2.cvtColor(np.uint8([[(avg_hue, 255, 255)]]), cv2.COLOR_HSV2BGR)[0][0].tolist())
                            #self.get_logger().info(f"   augmented image of {rows} x {cols} x {channels} received use color {bgr_color}")

                            #self.get_logger().info(f"   draw contours on augmented image")
                            cv2.drawContours(augmented_image,
                                             [c],            # fake contours list
                                             0,              # index into fake contours list
                                             bgr_color,
                                             2)

                            radius += 2  # we eroded a bit in process of labeling


                        if ratio > self.min_box_ratio and min(size) > self.min_box_size and radius > self.min_radius:
                            if self._image_publisher:
                                try:
                                    #self.get_logger().info(f"   draw bounding box on augmented image")
                                    box_points = np.int0(cv2.boxPoints(rect))
                                    #self.get_logger().info("box points: ", box_points)
                                    cv2.drawContours(augmented_image,
                                                     [box_points],            # fake contours list
                                                     0,
                                                     (255, 255, 0),  # draw bounding box as yellow (OpenCV used BGR)
                                                     2)
                                    # Draw defined center of rectangle using small white circle
                                    cv2.circle(augmented_image, (int(rect_center[0]), int(rect_center[1])), 2, (255, 255, 255), -1)
                                except Exception as e:
                                    if cv_show:
                                        self.get_logger().error(e)

                            # self.get_logger().info(f"  Found ball like object {i} of hue={avg_hue} at ({center}) [{rect_center}] with radius={radius} in pixel coordinates")


                            #self.get_logger().info(f" construct ball {i} at {center} r={radius}")
                            ball = Ball()
                            ball.id = i

                            if self.filter_map:
                                label = None
                                for limits in self.filter_map:
                                    if avg_hue < limits['min_hue'] or avg_hue > limits['max_hue']:
                                        continue
                                    if avg_sat < limits['min_sat'] or avg_sat > limits['max_sat']:
                                        continue
                                    if avg_val < limits['min_value'] or avg_val > limits['max_value']:
                                        continue

                                    label = limits['name']

                                    ############################ Debug data #######################################################
                                    min_hue = np.min(hue[logical_indices])
                                    max_hue = np.max(hue[logical_indices])
                                    min_sat = np.min(sat[logical_indices])
                                    max_sat = np.max(sat[logical_indices])
                                    min_val = np.min(val[logical_indices])
                                    max_val = np.max(val[logical_indices])
                                    self.get_logger().info(f"found {i} with {label} with HSV={min_hue}:{max_hue}/{min_sat}:{max_sat}/{min_val}:{max_val} radius={radius} and ratio={ratio} size = {size} ")
                                    ################################################################################################

                                    break

                                if not label:
                                    self.get_logger().info(f"No valid match for avg HSV ={avg_hue}/{avg_sat}/{avg_val} for label={i} radius={radius} and ratio={ratio} size = {size} ")
                                    continue # Get next labeled region - don't use this ball
                                else:
                                    ball.label = label
                            else:
                                if avg_hue > 150:
                                    ball.label = f"blue"
                                elif avg_hue > 110:
                                    ball.label = f"bluish"
                                elif avg_hue > 75:
                                    ball.label = f"bluish-green"
                                elif avg_hue > 60:
                                    ball.label = f"green"
                                elif avg_hue > 45:
                                    ball.label = f"redish-green"
                                elif avg_hue > 20:
                                    ball.label = f"redish"
                                else:
                                    ball.label = f"red"

                            #self.get_logger().info(f"  {ball.id} {ball.label}")
                            # Define single pixel image, and convert color map
                            ball.rgb_color = tuple(cv2.cvtColor(np.uint8([[(avg_hue, 255, 255)]]), cv2.COLOR_HSV2RGB)[0][0].tolist())

                            # Center in original image, not windowed!
                            avg_center = 0.33333333*(2*np.array(center) + np.array(rect_center)) # weight toward circle

                            #self.get_logger().info(f"  avg center = {avg_center}")
                            ball.center = Point(x=avg_center[0], y=avg_center[1])
                            ball.radius = radius
                            ball.position = self.project_ball(avg_center, radius, rows, cols)

                            # Valid ball - add to list
                            if ball.position:
                                balls.balls.append(ball)

                        else:
                            # Not squarish - meaning not likely symmetric ball shape
                            self.get_logger().info(f" object {i} of hue={avg_hue} at ({center}) with radius={radius} and ratio={ratio} size = {size} is not symmetric")
                    else:
                        # No valid contour found
                        self.get_logger().info(f" object {i} of hue={avg_hue} does not have a valid contour!")
            #else:
            #    self.get_logger().error(f'Max label is {max_label} - no valid labeled balls for latest image')

        except Exception as e:
            self.get_logger().error(f'Failed to extract ball list from processed image \n{e}')
            self.get_logger().error(traceback.format_exc())


    def _process_balls_list(self, balls, augmented_image):
        """
        Final processing and publishing markers and augmented image
        """
        try:
            if self._marker_publisher:
                markers = MarkerArray()

            if len(balls.balls) > 0 :
                self._balls_to_publish = True
            else:
                if self._marker_publisher:
                    marker = Marker()
                    marker.header = balls.header
                    marker.header.frame_id = self.target_frame
                    marker.ns = self._name
                    marker.action = Marker.DELETEALL # in this namespace
                    markers.markers.append(marker)

            if self._balls_to_publish:

                try:
                    # Get transform from image frame to robot frame
                    transform_matrix = None
                    try:
                        robot_transform = self.tf_buffer.lookup_transform(self.target_frame, balls.header.frame_id, balls.header.stamp, self.tf_wait_time)
                        transform_matrix = self._transform_matrix(robot_transform.transform)
                        balls.header.frame_id = self.target_frame
                    except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                        self.get_logger().warning(f'Failed to get timely {balls.header.frame_id} to {self.target_frame} transform - try {self.robot_frame} ...\n{exc}')
                        try:
                            robot_transform = self.tf_buffer.lookup_transform(self.robot_frame, balls.header.frame_id, balls.header.stamp, self.tf_wait_time)
                            transform_matrix = self._transform_matrix(robot_transform.transform)
                            balls.header.frame_id = self.robot_frame
                        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                            self.get_logger().warning(f'Failed to get timely {balls.header.frame_id} to {self.robot_frame} transform - use image frame ...\n{exc}')

                    for ball in balls.balls:
                        camera_z = ball.position.z  # Camera frame z
                        if transform_matrix is not None:
                            # Apply transform to the desired frame
                            ball.position = self._transform_point(transform_matrix, ball.position)

                        if self._image_publisher is not None and augmented_image is not None:
                            center_pix = (int(ball.center.x), int(ball.center.y))
                            bgr_color = ball.rgb_color[::-1]
                            cv2.circle(augmented_image, center_pix, max(2, int(ball.radius)), bgr_color, -1)

                        if self._marker_publisher:
                            marker = Marker()
                            marker.header = balls.header
                            marker.ns = self._name
                            marker.id = ball.id
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            marker.pose = Pose()
                            marker.pose.position = ball.position
                            marker.scale.x = camera_z*(ball.radius+2)/self.average_focal_length
                            marker.scale.y = marker.scale.x
                            marker.scale.z = marker.scale.x
                            marker.color.r = ball.rgb_color[0]/255.001
                            marker.color.g = ball.rgb_color[1]/255.001
                            marker.color.b = ball.rgb_color[2]/255.001
                            marker.color.a = self.marker_alpha
                            marker.lifetime = self.marker_lifetime
                            marker.frame_locked = False
                            markers.markers.append(marker)
                except Exception as exc:
                    self.get_logger().error(f'Error transforming ball positions to target frames - publish in image frame\n{exc}')
                    self.get_logger().error(traceback.format_exc())

                # Publish anytime a ball is detected, or the first empty list after detection
                self._balls_publisher.publish(balls)

                ball_colors = [ball.label for ball in balls.balls]
                self.get_logger().info(f"Published list of {len(balls.balls)} balls\n{ball_colors}")

                if self._marker_publisher:
                    self._marker_publisher.publish(markers)


            if self._image_publisher is not None and augmented_image is not None:
                try:
                    # Publish the resulting image.
                    # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image
                    # Request to use the original encoding
                    ros_image = self._br.cv2_to_imgmsg(augmented_image, "bgr8")
                    ros_image.header = balls.header # Keep time stamp and frame from original
                    ros_image.header.frame_id = self.camera_frame
                    self._image_publisher.publish(ros_image)

                    # Display the message on the console
                    #self.get_logger().info(f"Published ROS Image: \n{ros_image.header}\n{ros_image.height} {ros_image.width} {ros_image.encoding}")
                except Exception as e:
                    self.get_logger().error(f'Failed to published output image\n{e}')
                    return

            if len(balls.balls) == 0 and self._balls_to_publish:
                # This will force us to publish the first empty ball list once,
                # but not again consecutively, while found balls will be
                # continually streamed
                self._balls_to_publish = False
                return True

            return self._balls_to_publish

        except Exception as e:
            self.get_logger().error(f'Failed to extract ball list from processed image \n{e}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    ball_detector = SimpleBallDectector()
    ball_detector._dump_parameters()

    # Spin the node so the callback function is called.
    rclpy.spin(ball_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ball_detector.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
