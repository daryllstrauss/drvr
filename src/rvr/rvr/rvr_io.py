from dataclasses import dataclass
import os
import sys
import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups

class RVRState:
    color = None
    ambient = None
    quaternion = None
    imu = None
    accelerometer = None
    gyroscope = None
    locator = None
    velocity = None
    speed = None
    core_time = None
    encoders = None

class RVR_IO(object):
    state = RVRState()

    def __init__(self, loop, ttydev="/dev/ttyAMA0", rate=250):
        self.dev = SpheroRvrAsync(
            dal=SerialAsyncDal(loop, ttydev)
        )
        self.rate = rate
        self.ready = False

    async def setup(self):
        await self.dev.wake()
        await asyncio.sleep(2)
        bat = await self.dev.get_battery_percentage()
        print("Battery", bat, flush=True)
        self.ready = True
    
    async def close(self):
        await self.dev.close()
    
    async def enable_service(self, cb, service):
        # await self.dev.sensor_control.stop()
        await self.dev.sensor_control.add_sensor_data_handler(
            service=service,
            handler=cb
        )
        # await self.dev.sensor_control.start(interval=self.rate)

    async def disable_service(self, service):
        # await self.dev.sensor_control.stop()
        await self.dev.sensor_control.remove_sensor_data_handler(
            service=service
        )
        # await self.dev.sensor_control.start(interval=rate)
    
    async def change_rate(self, rate):
        self.rate = rate
        # await self.dev.sensor_control.stop()
        await self.dev.sensor_control.start(interval=rate)
    
    @staticmethod
    async def color_cb(data):
        RVR_IO.state.color = data

    async def enable_color_detection(self):
        await self.enable_service(self.color_cb, RvrStreamingServices.color_detection)

    async def disable_color_detection(self):
        await self.disable_service(RvrStreamingServices.color_detection)
        RVR_IO.state.color = None

    @staticmethod
    def ambient_light_cb(data):
        RVR_IO.state.ambient_light = data

    async def enable_ambient_light(self):
        await self.enable_service(
            self.ambient_light_cb,
            RvrStreamingServices.ambient_light)

    async def disable_ambient_light(self):
        await self.disable_service(RvrStreamingServices.ambient_light)
        RVR_IO.state.ambient_light = None

    @staticmethod
    def quaternion_cb(data):
        RVR_IO.state.quaternion = data

    async def enable_quaternion(self):
        await self.enable_service(self.quaternion_cb, RvrStreamingServices.quaternion)

    async def disable_quaternion(self):
        await self.disable_service(RvrStreamingServices.quaternion)
        RVR_IO.state.quaternion = None

    @staticmethod
    def imu_cb(data):
        RVR_IO.state.imu = data

    async def enable_imu(self):
        await self.enable_service(self.imu_cb, RvrStreamingServices.imu)
    
    async def disable_imu(self):
        await self.disable_service(cb, RvrStreamingServices.imu)
        RVR_IO.state.imu = None

    @staticmethod
    def accelerometer_cb(data):
        RVR_IO.state.accelerometer = data

    async def enable_accelerometer(self):
        await self.enable_service(self.accelerometer_cb, RvrStreamingServices.accelerometer)

    async def disable_accelerometer(self):
        await self.disable_service(RvrStreamingServices.accelerometer)
        RVR_IO.state.accelerometer = None

    @staticmethod
    def gyroscope_cb(data):
        RVR_IO.state.gyroscope = data

    async def enable_gyroscope(self):
        await self.enable_service(self.gyroscope_cb, RvrStreamingServices.gyroscope)

    async def disable_gyroscope(self, cb):
        await self.disable_service(RvrStreamingServices.gyroscope)
        RVR_IO.state.gyroscope = None

    @staticmethod
    def locator_cb(data):
        RVR_IO.state.locator = data

    async def enable_locator(self):
        await self.enable_service(self.locator_cb, RvrStreamingServices.locator)

    async def disable_locator(self):
        await self.disable_service(RvrStreamingServices.locator)
        RVR_IO.state.locator = None

    @staticmethod
    def velocity_cb(data):
        RVR_IO.state.velocity = data

    async def enable_velocity(self):
        await self.enable_service(self.velocity_cb, RvrStreamingServices.velocity)

    async def disable_velocity(self):
        await self.disable_service(RvrStreamingServices.velocity)
        RVR_IO.state.velocity = None

    @staticmethod
    def speed_cb(data):
        RVR_IO.state.speed = data

    async def enable_speed(self):
        await self.enable_service(self.speed_cb, RvrStreamingServices.speed)

    async def disable_speed(self):
        await self.disable_service(RvrStreamingServices.speed)
        RVR_IO.state.speed = None

    @staticmethod
    def core_time_cb(data):
        RVR_IO.state.core_time = data

    async def enable_core_time(self):
        await self.enable_service(self.core_time_cb, RvrStreamingServices.core_time)

    async def disable_core_time(self):
        await self.disable_service(RvrStreamingServices.core_time)
        RVR_IO.state.core_time = None

    @staticmethod
    def encoders_cb(data):
        RVR_IO.state.encoders = data

    async def enable_encoders(self):
        await self.enable_service(self.encoders_cb, RvrStreamingServices.encoders)

    async def disable_encoders(self):
        await self.disable_service(RvrStreamingServices.encoders)
        RVR_IO.state.encoders = None
