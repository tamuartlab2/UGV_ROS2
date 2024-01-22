import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped 
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

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
F_obstacle_subscript_topic = 'force/obstacle'
F_linear_drive_subscript_topic = 'force/linear_drive'
motor_speed_subscript_topic = 'Roboclaw/Odom'
speed_pub_topic = 'cmd_vel'
enable_obstacle_force_topic = 'force/enable_obstacle_force'
parameter_subscript_topic = 'loca_and_nav/parameters'
publish_rate = 10.0		#Hz, must be the same as F_obstacle and F_linear_drive
K_w_default = 1.5       # turning coefficient
f_c_default = 1.0       # drag coefficient
# back_turn_constant = 5.0
# f_angular_c = 0.05
linear_speed_limitation_default = 1.0       #m/s
# back_speed_limitation = 0.1         #m/s
angular_speed_limitation_default = 0.2      #rad/s
enable_linear_force = True

dt = 1/publish_rate
    

class Speed_Pub(Node):

    F_x = 0.0
    F_y = 0.0
    linear_vel_last = 0.0
    enable_obstacle_force = False
    # angular_vel_last = 0.0
    K_w = K_w_default
    f_c = f_c_default
    linear_speed_limitation = linear_speed_limitation_default
    angular_speed_limitation = angular_speed_limitation_default

    def __init__(self):
        super().__init__("Speed_Pub")	#node name

        self.pubVel = self.create_publisher(Twist, speed_pub_topic, 10)

        self.F_obstacle_subscription = self.create_subscription(Vector3Stamped, F_obstacle_subscript_topic, self.F_obstacle_update, 10)
        self.F_obstacle_subscription = self.create_subscription(Vector3Stamped, F_linear_drive_subscript_topic, self.F_linear_drive_update, 10)
        self.motor_speed_subscription = self.create_subscription(Odometry, motor_speed_subscript_topic, self.motor_speed_update, 10)
        self.enable_obstacle_force_subscription = self.create_subscription(Bool, enable_obstacle_force_topic, self.enable_obstacle_force_update, 10)
        self.parameter_subscription = self.create_subscription(Float64MultiArray, parameter_subscript_topic, self.parameter_update, 10)

        self.timer = self.create_timer(dt, self.loca_pub)  #period time in sec, function name

    def loca_pub(self):
        vel = Twist()
        # vel.linear.x = self.linear_vel_last * (1.0 - f_c) + self.F_x * dt
        vel.linear.x = self.linear_vel_last + (self.F_x  - self.linear_vel_last * self.f_c) * dt
        # self.F_y -= self.angular_vel_last * f_angular_c

        # if vel.linear.x == 0:
        #     vel.angular.z = self.F_y * K_w
        # else:
        #     vel.angular.z = self.F_y/vel.linear.x

        # if vel.linear.x > linear_speed_limitation:
        #     vel.linear.x = linear_speed_limitation
        # elif vel.linear.x < -back_speed_limitation:     #UGV turn around if backward speed is too fast
        #     if vel.linear.x < -linear_speed_limitation: 
        #         vel.linear.x = -linear_speed_limitation
        #     vel.angular.z = back_turn_constant * vel.linear.x
        # if vel.angular.z > angular_speed_limitation:
        #     vel.angular.z = angular_speed_limitation
        # elif vel.angular.z < -angular_speed_limitation: 
        #     vel.angular.z = -angular_speed_limitation

        
        if vel.linear.x > self.linear_speed_limitation:
            vel.linear.x = self.linear_speed_limitation
        elif vel.linear.x < 0.0:     
            vel.linear.x = 0.0      # disable backward speed and let UGV turn around
        if vel.linear.x == 0.0:
            vel.angular.z = self.F_y * self.K_w
        else:
            vel.angular.z = self.F_y/vel.linear.x
        # vel.angular.z = self.F_y * K_w
        if vel.angular.z > self.angular_speed_limitation:
            vel.angular.z = self.angular_speed_limitation
        elif vel.angular.z < - self.angular_speed_limitation: 
            vel.angular.z = - self.angular_speed_limitation

        self.pubVel.publish(vel)
        self.F_x = 0.0
        self.F_y = 0.0
        self.linear_vel_last = 0.0
        # self.angular_vel_last = vel.angular.z

    def F_obstacle_update(self, msg):
        # vel.header.stamp = self.get_clock().now().to_msg()
        if self.enable_obstacle_force == True:
            self.F_x += msg.vector.x
            self.F_y += msg.vector.y

    def motor_speed_update(self, msg):
        self.linear_vel_last = msg.twist.twist.linear.x
        # self.angular_vel_last = msg.twist.twist.angular.z
    
    def F_linear_drive_update(self, msg):
        if enable_linear_force == True:
            self.F_x += msg.vector.x 
            self.F_y += msg.vector.y 

    def enable_obstacle_force_update(self, msg):
        self.enable_obstacle_force = msg.data
        
        # if msg.vector.x == 0.0 and msg.vector.y == 0.0:
        #     self.enable_obstacle_force = False
        # else:
        #     self.enable_obstacle_force = True
    
    def parameter_update(self, msg):
        self.f_c = msg.data[1]
        self.K_w = msg.data[2]
        self.linear_speed_limitation = msg.data[4]
        self.angular_speed_limitation = msg.data[5]

    # def wrapToPi(self, angle):
    #     # takes an angle as input and calculates its equivalent value within the range of -pi (exclusive) to pi 
    #     wrapped_angle = angle % (2 * math.pi)
    #     if wrapped_angle > math.pi:
    #         wrapped_angle -= 2 * math.pi
    #     return wrapped_angle
    
    # def quaternion_to_ypr(self, x, y, z, w):
    #     # Convert quaternion components to YPR angles
    #     sinr_cosp = 2 * (w * x + y * z)
    #     cosr_cosp = 1 - 2 * (x**2 + y**2)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)

    #     sinp = 2 * (w * y - z * x)
    #     pitch = math.asin(sinp)

    #     siny_cosp = 2 * (w * z + x * y)
    #     cosy_cosp = 1 - 2 * (y**2 + z**2)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)
    #     return yaw, pitch, roll
		

def main():
	rclpy.init()
	my_pub = Speed_Pub()

	try:
		rclpy.spin(my_pub)
	except KeyboardInterrupt:
		my_pub.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
