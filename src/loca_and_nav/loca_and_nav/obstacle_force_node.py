import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

# from std_msgs.msg import String, Bool, Float64, Int8
# from sensor_msgs.msg import NavSatFix, Imu
# import tf_transformations
# from tf_transformations import euler_from_quaternion
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import haversine as hs
# from haversine import Unit
# import math


#declare parameters
F_obstacle_pub_topic = 'force/obstacle'
lidar_scan_subscript_topic = 'scan'
depth_subscript_topic = "camera/aligned_depth_to_color/image_raw"
parameter_subscript_topic = 'loca_and_nav/parameters'
scout_lidar_topic = 'scout_lidar_check'
enable_camera = True
enable_plant_scout_check = True
publish_rate = 2.5		#Hz
K_e_default = 0.8              #K_e = k * rou * q'
lidar_effective_distance_default = 2.5        #m
camera_effective_distance_default = 2.5         #m
# vehcle_width = 0.426        #m
vehcle_width = 0.25        #m, in narrow space
vehcle_length = 0.455       #m
lidar_position_x = 0.09     #m
lidar_position_y = 0.0     #m
obstacle_point_num = 100
decay_constant = 2.0

#plant lidar check parameter
check_distance = 3.0       #m
check_angle_range = 20.0 * np.pi / 180.0
left_angle_min = 0.5 * (np.pi - check_angle_range)
left_angle_max = 0.5 * (np.pi + check_angle_range)
right_angle_min = 0.5 * (- np.pi - check_angle_range)
right_angle_max = 0.5 * (- np.pi + check_angle_range)
low_threshold = 0.15
high_threshold = 0.75

# camera info
depthScale = 0.001      #m
image_width = 640
image_height = 480
cx = 317.3588562011719
cy = 237.2884521484375
fx = 380.35723876953125
fy = 380.35723876953125
min_obstacle_height = 0.05      #m
max_obstacle_height = 0.4       #m
camera_height = 0.12        #m

obstacle_point_resolution = np.pi*2 / obstacle_point_num
dt = 1/publish_rate
fx_inv = 1 / fx
fy_inv = 1 / fy
# K_e_camera = K_e * fx_inv * fy_inv / (max_obstacle_height - min_obstacle_height)    

class Obstacle_Force(Node):

    # F_x_lidar = 0.0
    # F_y_lidar = 0.0
    lidar_flag = False
    depth_camera_flag = False
    # F_x_cam = 0.0
    # F_y_cam = 0.0
    angle_min = 0.0
    obstacle_distance_array = np.full((obstacle_point_num, 1), np.inf)       #it is a np.array
    obstacle_distance_cam_array = np.full((obstacle_point_num, 1), np.inf)
    K_e = K_e_default
    lidar_effective_distance = lidar_effective_distance_default
    camera_effective_distance = camera_effective_distance_default
    left_status_last = True
    right_status_last = True

    def __init__(self):
        super().__init__("Obstacle_Force")	#node name

        self.pub_F = self.create_publisher(Vector3Stamped, F_obstacle_pub_topic, 10)
        self.scout_lidar_pub = self.create_publisher(UInt8MultiArray, scout_lidar_topic, 10)

        self.lidar_scan_subscription = self.create_subscription(LaserScan, lidar_scan_subscript_topic, self.F_lidar_obstacle, 10)
        self.depth_cam_subscription = self.create_subscription(Image, depth_subscript_topic, self.depth_reading, 10)
        self.parameter_subscription = self.create_subscription(Float64MultiArray, parameter_subscript_topic, self.parameter_update, 10)

        self.timer = self.create_timer(dt, self.force_pub)       #period time in sec, function name

    # Main loop
    def force_pub(self):
        Force = Vector3Stamped()
        Force.header.stamp = self.get_clock().now().to_msg()
        if enable_camera == True:
            if self.depth_camera_flag == True:
                self.depth_camera_flag = False
            else:
                for i in range(obstacle_point_num):
                    decay_distance = self.obstacle_distance_cam_array[i] * decay_constant
                    if decay_distance < self.obstacle_distance_array[i]:   
                        self.obstacle_distance_array[i] = decay_distance
        if self.lidar_flag == True:
            self.lidar_flag = False
            F_x = 0.0
            F_y = 0.0
            for i in range(obstacle_point_num):
                F = - self.K_e * obstacle_point_resolution / (self.obstacle_distance_array[i])**2
                F = F[0]     # convert 1x1 numpy array to single value
                angle = i * obstacle_point_resolution + self.angle_min
                F_x += F * np.cos(angle)
                F_y += F * np.sin(angle)
            Force.vector.x = F_x
            Force.vector.y = F_y
            self.pub_F.publish(Force)

        self.obstacle_distance_array = np.full((obstacle_point_num, 1), np.inf)


    # Update lidar data to obstacle array
    def F_lidar_obstacle(self, msg):
        self.lidar_flag = True
        self.angle_min = msg.angle_min
        angle = msg.angle_min
        len_count = len(msg.ranges)

        count = 0
        left_p_count = 0
        left_count = 0
        right_p_count = 0
        right_count = 0

        while count < len_count:
            if msg.ranges[count] < self.lidar_effective_distance:      # msg.ranges[count] is the distance
                if msg.ranges[count] > self.find_boundary_distance(vehcle_width, vehcle_length, lidar_position_x, lidar_position_y, angle):
                    # F = -K_e * msg.angle_increment / msg.ranges[count]
                    # self.F_x_lidar += F * np.cos(angle)
                    # self.F_y_lidar += F * np.sin(angle)
                    angle_num = np.divmod(angle - self.angle_min, obstacle_point_resolution)
                    # print(angle_num)
                    if int(angle_num[0]) < obstacle_point_num:
                        if msg.ranges[count] < self.obstacle_distance_array[int(angle_num[0])]:
                            self.obstacle_distance_array[int(angle_num[0])] = msg.ranges[count]
                    
                    if enable_plant_scout_check == True:
                        check_angle = self.wrapToPi(angle)
                        if check_angle > left_angle_min and check_angle < left_angle_max:
                            left_count += 1
                            if  msg.ranges[count] < check_distance:
                                left_p_count += 1
                        if check_angle > right_angle_min and check_angle < right_angle_max:
                            right_count += 1
                            if  msg.ranges[count] < check_distance:
                                right_p_count += 1

            angle += msg.angle_increment
            count += 1

        if enable_plant_scout_check:
            pub_msg = UInt8MultiArray()
            pub_msg.data = [0, 0, 0, 0]
            if left_count == 0:
                left_ocupancy = 0
            else:
                left_ocupancy = left_p_count / left_count
            if right_count == 0:
                right_ocupancy = 0
            else:
                right_ocupancy = right_p_count / right_count
            # data[1] is left, tiggered on high to low
            if self.left_status_last == False and left_ocupancy > high_threshold:
                self.left_status_last = True
                pub_msg.data[1] = 0
            elif self.left_status_last == True and left_ocupancy < low_threshold:
                self.left_status_last = False
                pub_msg.data[1] = 1
            else:
                pub_msg.data[1] = 0

            # data[2] is right, tiggered on high to low
            if self.right_status_last == False and right_ocupancy > high_threshold:
                right_status = True
                self.right_status_last = right_status
                pub_msg.data[2] = 0
            elif self.right_status_last == True and right_ocupancy < low_threshold:
                right_status = False
                self.right_status_last = right_status
                pub_msg.data[2] = 1
            else:
                pub_msg.data[2] = 0
            
            if left_ocupancy < low_threshold and right_ocupancy < low_threshold:
                pub_msg.data[0] = 0
            else:
                pub_msg.data[0] = 1
            self.scout_lidar_pub.publish(pub_msg)
    
    # Update depth camera data to obstacle array
    def depth_reading(self, msg):
        if enable_camera == True:
            self.depth_camera_flag = True
            self.obstacle_distance_cam_array = np.full((obstacle_point_num, 1), np.inf)
            raw_bytes_data = np.array(msg.data, dtype=np.uint8).tobytes()
            depth_data = np.frombuffer(raw_bytes_data, dtype=np.uint16) * depthScale
            for v in range(0, image_height, 10):
                # u = i % image_width     # remainder
                # v = i // image_width    # quotient
                for u in range(0, image_width, 5):
                    x = depth_data[u + v * image_width]
                    y = - (u - cx) * x * fx_inv
                    z = - (v - cy) * x * fy_inv
                    if z + camera_height > min_obstacle_height and z + camera_height < max_obstacle_height:
                        distance = np.sqrt(x**2 + y**2)
                        if distance < self.camera_effective_distance and distance > 0.0:
                            angle = np.arctan2(y, x) 
                            angle_num = np.divmod(angle - self.angle_min, obstacle_point_resolution)
                            if int(angle_num[0]) < obstacle_point_num:
                                if distance < self.obstacle_distance_array[int(angle_num[0])]:   
                                    self.obstacle_distance_array[int(angle_num[0])] = distance      # Update obstacle array
                                    self.obstacle_distance_cam_array[int(angle_num[0])] = distance      # store obstacle from depth_cam
    
    def parameter_update(self, msg):
        self.K_e = msg.data[0]
        self.lidar_effective_distance = msg.data[3]
        self.camera_effective_distance = self.lidar_effective_distance

    # Calculate the distance between boundary and lidar
    def find_boundary_distance(self, w, l, x, y, angle):
        d = 0.0
        if np.arctan2(-(w/2+y), l/2-x) < angle and angle <= np.arctan2(w/2-y, l/2-x):
            d = (l/2 - x) / np.cos(angle)
        elif np.arctan2(w/2-y, l/2-x) < angle and angle <= np.arctan2(w/2-y, -(l/2+x)):
            d = (w/2 - y) / np.sin(angle)
        elif np.arctan2(w/2-y, l/2-x) < angle and angle <= np.pi:
            d = -(l/2 + x) / np.cos(angle)
        elif -np.pi < angle and angle <= np.arctan2(-(w/2+y), -(l/2+x)):
            d = -(l/2 + x) / np.cos(angle) 
        elif np.arctan2(-(w/2+y), -(l/2+x)) < angle and angle <= np.arctan2(-(w/2+y), l/2-x):
            d = -(w/2 + y) / np.sin(angle) 
        return d
    
    def wrapToPi(self, angle):
        # takes an angle as input and calculates its equivalent value within the range of -pi (exclusive) to pi 
        wrapped_angle = angle % (2 * math.pi)
        if wrapped_angle > math.pi:
            wrapped_angle -= 2 * math.pi
        return wrapped_angle

def main():
	rclpy.init()
	my_pub = Obstacle_Force()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
