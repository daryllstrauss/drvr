import os
import socket
import asyncio
import sys
import asyncio
import json
import rclpy
import struct
import traceback
from math import radians
from tf_transformations import quaternion_from_euler
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from .rvr_io import RVR_IO
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from rvr_interfaces.msg import Raw
from tf2_ros import TransformBroadcaster

# See https://sdk.sphero.com/docs/getting_started/raspberry_pi/raspberry_pi_hardware_setup
# for correct serial cable connections.

class RVR(Node):
    bot = None

    def __init__(self, loop):
        super().__init__('rvr')
        self.rvr = RVR_IO(loop)
        report_description = ParameterDescriptor(description='position reporting: [imu, pos, transform, none]')
        self.declare_parameter("report", "odom", report_description)
        rate_description = ParameterDescriptor(description='update rate in hz')
        self.declare_parameter("rate", 20.0, rate_description)
        self.odom_publisher = None
        self.raw_publisher = None
        self.transform_publisher = None
        rate = self.get_parameter('rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/rate, self.timer_cb)
        report = self.get_parameter('report').get_parameter_value().string_value
        if report not in ["odom", "raw", "transform", "none"]:
            self.get_logger().warn(f"Unknown reporting method (report). No reporting will be done.")
        self.setup_socket()
        RVR.bot = self
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10)
        self.get_logger().info("RVR Node Startup")
    
    def setup_socket(self):
        self.path = "/tmp/rvr.socket"
        try:
            os.unlink(self.path)
        except OSError:
            if os.path.exists(self.path):
                raise
        self.server = None

    async def setup(self):
        self.get_logger().info("RVR Setup")
        await self.rvr.setup()
        report = self.get_parameter('report').get_parameter_value().string_value
        self.get_logger().info(f"Reporting {report}")
        try:
            if report == "raw":
                await self.rvr.enable_quaternion()
                await self.rvr.enable_accelerometer()
                await self.rvr.enable_gyroscope()
                await self.rvr.enable_encoders()
                self.raw_publisher = self.create_publisher(Raw, "raw", 10)
            elif report == "odom":
                await self.rvr.enable_imu()
                # await self.rvr.enable_locator()
                await self.rvr.enable_velocity()
                await self.rvr.enable_encoders()
                self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
            elif report == "transform":
                await self.rvr.enable_imu()
                await self.rvr.enable_locator()
                self.transform_publisher = TransformBroadcaster(self)

            # await self.rvr.enable_color_detection()
            # await self.rvr.enable_ambient_light()

            # await self.rvr.enable_speed()
            # await self.rvr.enable_core_time()
            rate = self.get_parameter('rate').get_parameter_value().double_value
            await self.rvr.change_rate(int(1000/rate))
        except Exception as e:
            print("Exception setup", e, flush=True)
        self.get_logger().info("RVR Setup Complete")

    async def stop(self):
        await self.rvr.close()

    def publish_raw(self, timestamp):
        if self.rvr.state.quaternion is None:
            return
        msg = Raw()
        msg.header.stamp = timestamp

        if self.rvr.state.quaternion["Quaternion"]['is_valid']:
            msg.orientation.x = self.rvr.state.quaternion["Quaternion"]['X']
            msg.orientation.y = self.rvr.state.quaternion["Quaternion"]['Y']
            msg.orientation.z = self.rvr.state.quaternion["Quaternion"]['Z']
            msg.orientation.w = self.rvr.state.quaternion["Quaternion"]['W']

        if self.rvr.state.quaternion["Gyroscope"]['is_valid']:
            msg.gyroscope.x = self.rvr.state.quaternion["Gyroscope"]['X']
            msg.gyroscope.y = self.rvr.state.quaternion["Gyroscope"]['Y']
            msg.gyroscope.z = self.rvr.state.quaternion["Gyroscope"]['Z']
        
        if self.rvr.state.quaternion["Accelerometer"]['is_valid']:
            msg.accelerometer.x = self.rvr.state.quaternion["Accelerometer"]['X']
            msg.accelerometer.y = self.rvr.state.quaternion["Accelerometer"]['X']
            msg.accelerometer.z = self.rvr.state.quaternion["Accelerometer"]['X']
        
        if self.rvr.state.encoders["Encoders"]['is_valid']:
            msg.encoders.left = self.rvr.state.encoders["Encoders"]["LeftTicks"]
            msg.encoders.right = self.rvr.state.encoders["Encoders"]["RightTicks"]

        self.raw_publisher.publish(msg)

    def publish_odom(self, timestamp):
        if (self.rvr.state.imu is None
            or self.rvr.state.velocity is None):
            print("Waiting for valid data from RVR", flush=True)
            return

        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = "odom"
        if self.rvr.state.imu["IMU"]['is_valid']:
            quaternion = quaternion_from_euler(
                radians(self.rvr.state.imu["IMU"]["Roll"]),
                radians(self.rvr.state.imu["IMU"]["Pitch"]),
                radians(self.rvr.state.imu["IMU"]["Yaw"]))
            msg.pose.pose.orientation.x = quaternion[0]
            msg.pose.pose.orientation.y = quaternion[1]
            msg.pose.pose.orientation.z = quaternion[2]
            msg.pose.pose.orientation.w = quaternion[3]
        else:
            msg.pose.covariance[0] = -1
        if self.rvr.state.velocity["Velocity"]['is_valid']:
            msg.twist.twist.linear.x = self.rvr.state.velocity["Velocity"]["X"]
            msg.twist.twist.linear.y = self.rvr.state.velocity["Velocity"]["Y"]
            msg.twist.twist.linear.z = 0.0
        else:
            msg.twist.covariance = -1
        self.odom_publisher.publish(msg)

    def publish_transform(self, timestamp):
        if (self.rvr.state.imu is None
            or self.rvr.state.imu["IMU"]['is_valid'] is False
            or self.rvr.state.locator is None
            or self.rvr.state.locator["Locator"]['is_valid'] is False):
            print("Waiting for valid data from RVR", flush=True)
            return

        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        quaternion = quaternion_from_euler(
            radians(self.rvr.state.imu["IMU"]["Roll"]),
            radians(self.rvr.state.imu["IMU"]["Pitch"]),
            radians(self.rvr.state.imu["IMU"]["Yaw"]))
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        t.transform.translation.x = self.rvr.state.locator["Locator"]["X"]
        t.transform.translation.y = self.rvr.state.locator["Locator"]["Y"]
        t.transform.translation.z = 0.0
        self.transform_publisher.sendTransform(t)

    def timer_cb(self):
        try:
            timestamp = self.get_clock().now().to_msg()
            if self.odom_publisher is not None:
                self.publish_odom(timestamp)
            if self.raw_publisher is not None:
                self.publish_raw(timestamp)
            if self.transform_publisher is not None:
                self.publish_transform(timestamp)
        except Exception as e:
            print("Exception timer", e, flush=True)
    
    def velocity_callback(self, velocity_msg):
        print("Velocity", velocity_msg, flush=True)


    @staticmethod
    async def handle_cmd(reader, writer):
        try:
            while True:
                cmd = await reader.read(4)
                if cmd == b"RVEL":
                    if (RVR.bot.rvr.state.encoders is not None
                        and RVR.bot.rvr.state.encoders["Encoders"]['is_valid']):
                        left = RVR.bot.rvr.state.encoders["Encoders"]["LeftTicks"]
                        right = RVR.bot.rvr.state.encoders["Encoders"]["RightTicks"]
                        # RVR.bot.get_logger().info(f"Encoders {left} {right}")
                    else:
                        left = 0
                        right = 0
                    ba = bytearray(struct.pack("II", left, right))
                    writer.write(ba)
                    await writer.drain()
                elif cmd == b'WVEL':
                    size = struct.calcsize("dd")
                    ba = b''
                    while size > 0:
                        ba += await reader.read(size)
                        size -= len(ba)
                    if len(ba) != 16:
                        RVR.bot.get_logger().info(f"Array Len {len(ba)}")
                    left_vel, right_vel  = struct.unpack("dd", ba)
                    RVR.bot.get_logger().info(f"Set Left {left_vel} Right {right_vel}")
                    if left_vel > .1:
                        left_vel = .1
                    if right_vel > .1:
                        right_vel = .1
                    await RVR.bot.rvr.dev.drive_tank_si_units(left_velocity=left_vel, right_velocity=right_vel)
                elif cmd == b"QUIT":
                    break
        except Exception as e:
            RVR.bot.get_logger().warn(f"Exception CMD: {traceback.format_exc()}")

async def run_ros(rvr):
    try:
        await rvr.setup()
        while True:
            rclpy.spin_once(rvr, timeout_sec=0)
            await asyncio.sleep(0.0)
    except Exception as e:
        print("Exception run_ros", e, flush=True)

async def run_server(rvr):
    rvr.server = await asyncio.start_unix_server(rvr.handle_cmd, rvr.path)
    with rvr.server:
        await rvr.server.run_forever()

def main(args=None):
    rclpy.init(args=args)

    loop = asyncio.get_event_loop()
    rvr = RVR(loop)
    asyncio.ensure_future(run_ros(rvr))
    asyncio.ensure_future(run_server(rvr))
    try:
        loop.run_forever()
        rvr.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        loop.run_until_complete(rvr.stop())


if __name__ == '__main__':
    main()
    sys.exit(0)
