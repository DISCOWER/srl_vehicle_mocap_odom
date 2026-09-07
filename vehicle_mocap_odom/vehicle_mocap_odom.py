import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import socket, re
import numpy as np

class MyPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_mocap_odom')
        namespace = self.declare_parameter('namespace', '').value
        if namespace == '':
            namespace = socket.gethostname()
            namespace = re.sub(r'[^a-zA-Z0-9_~{}]', '_', namespace)        
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(VehicleOdometry,
            f"/{namespace}/fmu/in/vehicle_visual_odometry",
            qos_profile
        )
        self.odom_sub = self.create_subscription(Odometry,
            f"/{namespace}/odom",
            self.odom_cb,
            10
        )

        timer_period = 0.01  # seconds
        self.got_odom = False
        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        if self.got_odom:
            msg = VehicleOdometry()

            # Set time
            time_us = int(self.get_clock().now().nanoseconds / 1000)
            msg.timestamp = time_us 
            msg.timestamp_sample = time_us

            # Build the message
            # Here we convert frames from mocap's /odom to PX4's /vehicle_visual_odometry
            
            # Set the frames
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
            
            # Position in global NED frame
            msg.position = [self.pose.position.y, self.pose.position.x, -self.pose.position.z]
            
            # Quaternion (qw, qx, qy, qz) as body FRD frame to global ENU frame
            q_ned = self.q_enu_to_q_ned([self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z])
            msg.q = [q_ned[0], q_ned[1], q_ned[2], q_ned[3]]

            # Velocity in body FRD frame
            msg.velocity = [self.twist.linear.x, -self.twist.linear.y, -self.twist.linear.z]

            # Angular velocity in body FRD frame
            msg.angular_velocity = [self.twist.angular.x, -self.twist.angular.y, -self.twist.angular.z]

            self.publisher_.publish(msg)

    def odom_cb(self, msg: Odometry):
        # Receives odom message from mocap
        # Position is in global ENU frame
        # Quaternion (qw, qx, qy, qz) is in body FLU frame to global ENU frame
        # Velocity is in body FLU frame
        # Angular velocity is in body FLU frame
        self.pose = msg.pose.pose
        self.twist = msg.twist.twist
        self.got_odom = True

    def q_ned_to_q_enu(self, q_ned):
        # Convert NED quaternion to ENU quaternion
        # q is in the form (qw, qx, qy, qz) and describes the rotation from body frame to global frame
        # Yes, NED <-> ENU  is symmetric
        q_enu = 1/np.sqrt(2) * np.array([q_ned[0] + q_ned[3], q_ned[1] + q_ned[2], q_ned[1] - q_ned[2], q_ned[0] - q_ned[3]])
        q_enu /= np.linalg.norm(q_enu)
        return q_enu.astype(float)
    
    def q_enu_to_q_ned(self, q_enu):
        # Convert ENU quaternion to NED quaternion
        # q is in the form (qw, qx, qy, qz) and describes the rotation from body frame to global frame
        # Yes, NED <-> ENU  is symmetric
        q_ned = 1/np.sqrt(2) * np.array([q_enu[0] + q_enu[3], q_enu[1] + q_enu[2], q_enu[1] - q_enu[2], q_enu[0] - q_enu[3]])
        q_ned /= np.linalg.norm(q_ned)
        return q_ned.astype(float)

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
