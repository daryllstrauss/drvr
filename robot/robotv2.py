import os
import cv2
import time
import asyncio
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal, DriveFlagsBitmask, SpheroRvrTargets
from predict import OakD

# RVR
# +5v GND TX  RX  (front)
#  NC Blk Gry Wht

# Pi
# +5v +5v GND TXO RXI
#  NC  NC Blk Wht Gry

class RobotV2(object):
    datadir = None
    rvr = None
    busy = False
    speed = 100
    heading = 0
    completeTime = 0.25
    oakD = None
    frame = 1

    def __init__(self, loop: asyncio.AbstractEventLoop, datadir = "data"):
        self.datadir = datadir
        self.rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
        self.oakD = OakD()
        self.oakD.startDevice()
        self.datadir = f"v2-{int(time.time())}"
        os.mkdir(self.datadir)
    
    def shutdown(self):
        self.oakD.shutdown()

    async def setBusy(self):
        while self.busy:
            await asyncio.sleep(self.completeTime)
        self.busy = True
    
    def clearBusy(self):
        self.busy = False

    async def startup(self):
        await self.setBusy()
        await self.rvr.wake()
        await asyncio.sleep(2)
        await self.rvr.reset_yaw()
        await asyncio.sleep(1)
        self.clearBusy()
    
    async def shutdown(self):
        await self.rvr.close()

    async def battery(self) -> dict:
        result = {}
        percentage = await self.rvr.get_battery_percentage()
        result.update(percentage)
        state = await self.rvr.get_battery_voltage_state()
        result.update(state)
        return result

    async def drive(self, duration=1.4) -> None:
        await self.setBusy()
        await self.rvr.drive_with_heading(
            speed=self.speed,
            heading=self.heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(duration)
        await self.rvr.drive_with_heading(
            speed=0,
            heading=self.heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(self.completeTime)
        self.clearBusy()

    async def turnTo(self, heading):
        await self.setBusy()
        await self.rvr.drive_with_heading(
            speed=0,
            heading=heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(self.completeTime)
        self.heading = heading
        self.clearBusy()
    
    async def turn(self, num):
        heading = (self.heading + num) % 360
        await self.turnTo(heading)
    
    async def predict(self):
        results = self.oakD.getResult()
        predictions = self.oakD.getPredictions(results)
        self.oakD.analyze(predictions)

    async def snapshot(self):
        results = self.oakD.getResult()
        predictions = self.oakD.getPredictions(results)
        image = self.oakD.createTagImage(predictions)
        name = f"{self.datadir}/predict-{str(self.frame).zfill(5)}.png"
        cv2.imwrite(name, image)
        rgb = self.oakD.getRGB()
        name = f"{self.datadir}/rgb-{str(self.frame).zfill(5)}.png"
        cv2.imwrite(name, rgb)
        self.frame += 1

    async def next(self):
        await self.snapshot()
        await self.drive()