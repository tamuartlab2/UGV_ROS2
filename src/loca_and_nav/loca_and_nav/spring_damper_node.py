import rclpy
import struct
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64, Int8, UInt8MultiArray
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, Vector3Stamped 
from nav_msgs.msg import Odometry
from .o_point import O_Point

Robot_ID = 1.0
F_spring_damper_force_topic = 'force/spring_damper'
angle_subscript_topic = 'Localization/Odom'
position_subscript_topic = 'Localization/GPS'
spring_damper_enabled_topic = 'force/spring_damper_enabled'

publish_rate = 10		#Hz
dt = 1/publish_rate

SD_connection_number = 3
K_spring = 1.0
K_damper = 0.5
d_desired = 3.0

# original point of the global coordinate
Original_Point = O_Point()
lat_0 = Original_Point.lat_0
lon_0 = Original_Point.lon_0
# lat/lon to meter linear converter
lat_to_m = Original_Point.lat_to_m
lon_to_m = Original_Point.lon_to_m
m_to_lat = Original_Point.m_to_lat
m_to_lon = Original_Point.m_to_lon

class SpringDamper(Node):
    F_x = 0.0
    F_y = 0.0
    F_linear = 0.0
    F_linear_angle = 0.0
    vehicle_angle = 0.0
    vehicle_lat = 0.0
    vehicle_lon = 0.0
    spring_damper_enabled = False
    receive_postion = False
    robot_data = []
    robot_dis_matrix_last = []

    def __init__(self):
        super().__init__('Spring_Damper')

        self.pubXbeeRX = self.create_publisher(UInt8MultiArray, 'Teensy/XbeeTX', 10)
        self.pubForce = self.create_publisher(Vector3Stamped, F_spring_damper_force_topic, 10)

        self.timer = self.create_timer(dt, self.main_loop)  # period time in sec, function name

        self.subscription_xbee = self.create_subscription(UInt8MultiArray, 'Teensy/XbeeRX', self.xbee_callback, 10)
        self.postion_subscription = self.create_subscription(NavSatFix, position_subscript_topic, self.postion_update, 10)
        self.angle_subscription = self.create_subscription(Odometry, angle_subscript_topic, self.angle_update, 10)
        self.subscription_xbee = self.create_subscription(Bool, spring_damper_enabled_topic, self.spring_damper_enabled_update, 10)

    def main_loop(self):
        if self.spring_damper_enabled and len(self.robot_data) > 0 and self.receive_postion:
            self.receive_postion = False
            robot_dis_matrix = self.find_distance(self.robot_data)
            robot_dis_matrix = sorted(robot_dis_matrix, key=lambda x: x[1])     # sort the matrix based on the distance from smaller to greater
            robot_dis_matrix_effective = robot_dis_matrix[:SD_connection_number]    # only connect the closest # of robot
            # print(robot_dis_matrix_effective)
            for row in robot_dis_matrix_effective:
                F_s = K_spring * (row[1] - d_desired)
                row_n_last = self.find_index(self.robot_dis_matrix_last, row[0])
                if row_n_last != -1:
                    F_d = K_damper * (row[1] - self.robot_dis_matrix_last[row_n_last][1]) * publish_rate
                else:
                    F_d = 0.0
                self.F_x += (F_s + F_d) * np.cos(row[2] - self.vehicle_angle)
                self.F_y += (F_s + F_d) * np.sin(row[2] - self.vehicle_angle)

            F = Vector3Stamped()
            F.header.stamp = self.get_clock().now().to_msg()
            F.vector.x = self.F_x
            F.vector.y = self.F_y
            self.pubForce.publish(F)
            self.F_x = 0.0
            self.F_y = 0.0
            self.robot_dis_matrix_last = robot_dis_matrix
            self.robot_data = []

    def angle_update(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        ypr = self.quaternion_to_ypr(x, y, z, w)
        self.vehicle_angle = ypr[0]

    def postion_update(self, msg):
        self.receive_postion = True
        self.vehicle_lat = msg.latitude
        self.vehicle_lon = msg.longitude

        # boardcast current position
        if self.spring_damper_enabled:
            float_data = [Robot_ID, self.vehicle_lat, self.vehicle_lon]
            uint8_array = UInt8MultiArray()
            uint8_array.data = []
            for value in float_data:
                uint8_array.data.extend(struct.pack('d', value))
            self.pubXbeeRX.publish(uint8_array)

    def xbee_callback(self, msg):
        data = msg.data
        float_values = []
        for i in range(0, len(data)-1, 8):
            float_values.append(struct.unpack('d', bytearray(data[i:i+8]))[0])
        signal_strength = float(msg.data[len(data)-1])
        Robot_ID_received = float_values[0]
        robot_received_lat = float_values[1]
        robot_received_lon = float_values[2]

        # Check if robot_id already exists in the list
        found = False
        for data_entry in self.robot_data:
            if data_entry[0] == Robot_ID_received:
                # Append the data to the existing entry
                data_entry[1] = robot_received_lat
                data_entry[2] = robot_received_lon
                data_entry[3] = signal_strength
                found = True
                break
        if not found:
            # Create a new entry for the robot_id
            self.robot_data.append([Robot_ID_received, robot_received_lat, robot_received_lon, signal_strength])

    def spring_damper_enabled_update(self, msg):
        self.spring_damper_enabled = msg.data

    def wrapToPi(self, angle):
        # takes an angle as input and calculates its equivalent value within the range of -pi (exclusive) to pi 
        wrapped_angle = angle % (2 * math.pi)
        if wrapped_angle > math.pi:
            wrapped_angle -= 2 * math.pi
        return wrapped_angle

    def quaternion_to_ypr(self, x, y, z, w):
        # Convert quaternion components to YPR angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw, pitch, roll
    
    # Function to find the index of a robot ID in the data array
    def find_index(self, data, robot_id):
        for i, row in enumerate(data):
            if row[0] == robot_id:
                return i
        return -1  # Robot ID not found
    
    def find_distance(self, robot_data_array):
        robot_dis = []
        for data_entry in robot_data_array:
            lat = data_entry[1]
            lon = data_entry[2]
            d_x = (lon - self.vehicle_lon) * lon_to_m
            d_y = (lat - self.vehicle_lat) * lat_to_m
            distance = np.sqrt(d_x**2 + d_y**2)
            linear_angle = np.arctan2(d_y, d_x)
            robot_dis.append([data_entry[0], distance, linear_angle, data_entry[3]])
        return robot_dis

def main():
    rclpy.init()
    my_pub = SpringDamper()

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()