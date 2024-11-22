import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleAttitude, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

try:
    import numpy as np
    from pyquaternion import Quaternion
except ImportError:
    print("pyquaternion not found. Installing it now...")
    import subprocess, sys
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy"])
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyquaternion"])
        from pyquaternion import Quaternion
        print("pyquaternion successfully installed.")
    except Exception as e:
        print(f"Failed to install pyquaternion: {e}")
        sys.exit(1)


def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    # msg.header.stamp = Clock().now().nanoseconds / 1000
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.running_simulation = self.get_parameter_or('simulated', False)
        self.publisher_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)
        if self.running_simulation:
            self.sub_1 = self.create_subscription(PoseStamped, '/orion/loc/truth/pose', self.pose_cb, 1)
            self.sub_2 = self.create_subscription(TwistStamped, '/orion/loc/truth/twist', self.twist_cb, 1)
        else:
            self.sub_1 = self.create_subscription(Odometry, '/pop/odom', self.odom_cb, 1)
            self.attitude_sub = self.create_subscription(
                VehicleAttitude,
                "/fmu/out/vehicle_attitude",
                self.vehicle_attitude_callback,
                qos_profile,
            )
            self.local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                "/fmu/out/vehicle_local_position",
                self.vehicle_local_position_callback,
                qos_profile,
            )
            self.vehicle_pose_pub = self.create_publisher(
                PoseStamped, "/mocap_log/vehicle_pose", 10
            )

        timer_period = 0.01  # seconds
        self.got_pose = False
        self.got_twist = False
        self.got_odom = False
        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.odom = Odometry()
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):

        if (self.got_pose and self.got_twist) or (self.got_odom):
            msg = VehicleOdometry()

            # Set time
            time_s, time_ns = self.get_clock().now().seconds_nanoseconds()
            time_us = (time_s * 1000000) + (time_ns / 1000)
            msg.timestamp = int(time_us)
            msg.timestamp_sample = int(time_us)

            # Set frames
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.velocity_frame = VehicleOdometry.POSE_FRAME_NED

            # NED pose
            msg.position = [self.pose.pose.position.x, -self.pose.pose.position.y, -self.pose.pose.position.z]

            att_q = Quaternion(self.pose.pose.orientation.w, self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z)
            # msg.q = self.rotateQuaternion(Quaternion(att_q))
            msg.q = [att_q.w, att_q.x, -att_q.y, -att_q.z]
            #msg.velocity = [self.twist.twist.linear.x, -self.twist.twist.linear.y, -self.twist.twist.linear.z]
            #msg.angular_velocity = [self.twist.twist.angular.x, -self.twist.twist.angular.y, -self.twist.twist.angular.z]
            msg.velocity = [float('nan'), float('nan'), float('nan')]
            msg.angular_velocity = [float('nan'), float('nan'), float('nan')]
            self.publisher_.publish(msg)

            # Log vehicle local position
            vehicle_pose_msg = vector2PoseMsg(
                "mocap", self.vehicle_local_position, self.vehicle_attitude
            )
            self.vehicle_pose_pub.publish(vehicle_pose_msg)

    def rotateQuaternion(self, q_FLU_to_ENU):
        # quats are w x y z
        q_FLU_to_FRD = Quaternion(0, 1, 0, 0)
        # q_ENU_to_NED = Quaternion(0, 0.70711, 0.70711, 0)

        # q_rot = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.inverse
        q_rot = q_FLU_to_ENU * q_FLU_to_FRD.inverse
        return [q_rot.w, q_rot.x, q_rot.y, q_rot.z]

    def pose_cb(self, msg):
        self.pose = msg
        self.got_pose = True

    def twist_cb(self, msg):
        self.twist = msg
        self.got_twist = True

    def odom_cb(self, msg):
        self.pose = msg.pose
        self.twist = msg.twist
        self.got_odom = True

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz


def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
