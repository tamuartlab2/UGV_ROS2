import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu, CompressedImage, Image, LaserScan
from nav_msgs.msg import Odometry
import rosbag2_py
import datetime

imu_topic = 'Teensy/IMU'
gps1_topic = 'Teensy/GPS'
gps2_topic = 'ReachM2/GPS'
loca_odom_topic = 'Localization/Odom'
loca_gps_topic = 'Localization/GPS'
roboclaw_odom_topic = 'Roboclaw/Odom'
cam_rgb_topic = 'ra_camera'
cam_depth_topic = 'camera/aligned_depth_to_color/image_raw'
lidar_topic = 'scan'

date_now = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

class BagRecorder(Node):
    def __init__(self):
        super().__init__('BagRecorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='/media/artlab/Samsung_USB/DataCollection/'+date_now,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # all topic info
        teensy_imu_topic_info = rosbag2_py._storage.TopicMetadata(
            name=imu_topic,
            type='sensor_msgs/msg/Imu',
            serialization_format='cdr')
        self.writer.create_topic(teensy_imu_topic_info)

        teensy_gps_topic_info = rosbag2_py._storage.TopicMetadata(
            name=gps1_topic,
            type='sensor_msgs/msg/NavSatFix',
            serialization_format='cdr')
        self.writer.create_topic(teensy_gps_topic_info)

        reach_gps_topic_info = rosbag2_py._storage.TopicMetadata(
            name=gps2_topic,
            type='sensor_msgs/msg/NavSatFix',
            serialization_format='cdr')
        self.writer.create_topic(reach_gps_topic_info)
        
        roboclaw_odom_topic_info = rosbag2_py._storage.TopicMetadata(
            name=roboclaw_odom_topic,
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr')
        self.writer.create_topic(roboclaw_odom_topic_info)

        loca_odom_topic_info = rosbag2_py._storage.TopicMetadata(
            name=loca_odom_topic,
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr')
        self.writer.create_topic(loca_odom_topic_info)

        loca_gps_topic_info = rosbag2_py._storage.TopicMetadata(
            name=loca_gps_topic,
            type='sensor_msgs/msg/NavSatFix',
            serialization_format='cdr')
        self.writer.create_topic(loca_gps_topic_info)

        cam_rgb_topic_info = rosbag2_py._storage.TopicMetadata(
            name=cam_rgb_topic,
            type='sensor_msgs/msg/CompressedImage',
            serialization_format='cdr')
        self.writer.create_topic(cam_rgb_topic_info)

        cam_depth_topic_info = rosbag2_py._storage.TopicMetadata(
            name=cam_depth_topic,
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        self.writer.create_topic(cam_depth_topic_info)

        lidar_topic_info = rosbag2_py._storage.TopicMetadata(
            name=lidar_topic,
            type='sensor_msgs/msg/LaserScan',
            serialization_format='cdr')
        self.writer.create_topic(lidar_topic_info)

        # create subscriptions
        self.teensy_imu_subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.teensy_imu_topic_callback,
            10)
        
        self.teensy_gps_subscription = self.create_subscription(
            NavSatFix,
            gps1_topic,
            self.teensy_gps_topic_callback,
            10)
        
        self.reach_gps_subscription = self.create_subscription(
            NavSatFix,
            gps2_topic,
            self.reach_gps_topic_callback,
            10)
        
        self.roboclaw_odom_subscription = self.create_subscription(
            Odometry,
            roboclaw_odom_topic,
            self.roboclaw_odom_topic_callback,
            10)

        self.loca_odom_subscription = self.create_subscription(
            Odometry,
            loca_odom_topic,
            self.loca_odom_topic_callback,
            10)
        
        self.loca_gps_subscription = self.create_subscription(
            NavSatFix,
            loca_gps_topic,
            self.loca_gps_topic_callback,
            10)
        
        self.cam_rgb_subscription = self.create_subscription(
            CompressedImage,
            cam_rgb_topic,
            self.cam_rgb_topic_callback,
            10)
        
        self.cam_depth_subscription = self.create_subscription(
            Image,
            cam_depth_topic,
            self.cam_depth_topic_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.lidar_topic_callback,
            10)

    def teensy_imu_topic_callback(self, msg):
        self.writer.write(
            imu_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
    
    def teensy_gps_topic_callback(self, msg):
        self.writer.write(
            gps1_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
    def reach_gps_topic_callback(self, msg):
        self.writer.write(
            gps2_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def roboclaw_odom_topic_callback(self, msg):
        self.writer.write(
            roboclaw_odom_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
           
    def loca_odom_topic_callback(self, msg):
        self.writer.write(
            loca_odom_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
    def loca_gps_topic_callback(self, msg):
        self.writer.write(
            loca_gps_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
    def cam_rgb_topic_callback(self, msg):
        self.writer.write(
            cam_rgb_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
    def cam_depth_topic_callback(self, msg):
        self.writer.write(
            cam_depth_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
        
    def lidar_topic_callback(self, msg):
        self.writer.write(
            lidar_topic,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    br = BagRecorder()
    rclpy.spin(br)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
