import os
import asyncio
import json
import io
from datetime import datetime
from picamera import PiCamera  # pylint: disable = import-error
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import DriveFlagsBitmask
import numpy as np
import tensorflow as tf
from tensorflow.keras.preprocessing import image
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input as mobilenet_preprocess


class Robot(object):
    datadir = None
    rvr = None
    camera = None
    gps = None
    heading = 0
    speed = 100
    frame = 0
    duration = 1
    interp = None
    predictions = None
    # image_width = 299
    # image_height = 299
    image_width = 224
    image_height = 224
    busy = False

    def __init__(self, loop: asyncio.AbstractEventLoop, datadir: str = "data"):
        self.datadir = datadir
        self.rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
        self.camera = PiCamera(resolution=(320, 240), framerate=30)
        self.gps = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
        self.busy = False
        try:
            self.interp = tf.lite.Interpreter("steer.tflite")
            self.interp.allocate_tensors()
        except Exception:
            pass
    
    async def setBusy(self):
        while self.busy:
            await asyncio.sleep(0.25)
        self.busy = True
    
    def clearBusy(self):
        self.busy = False

    async def startup(self):
        await self.setBusy()
        await self.rvr.wake()
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

    def writeFrame(self, speed, duration):
        self.frame += 1
        command = {
            "heading": self.heading,
            "speed": speed,
            "duration": duration,
            "predictions": self.predictions
        }
        with open(f"{self.datadir}/command-{self.frame:04}.json", "w") as file:
            json.dump(command, file)

    async def drive(self) -> None:
        await self.setBusy()
        await self.rvr.drive_with_heading(
            speed=self.speed,
            heading=self.heading,
            flags=DriveFlagsBitmask.drive_reverse.value)
        await asyncio.sleep(self.duration)
        await self.rvr.drive_with_heading(
            speed=0,
            heading=self.heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(0.25)
        self.writeFrame(self.speed, self.duration)
        self.clearBusy()

    async def turnTo(self):
        await self.setBusy()
        await self.rvr.drive_with_heading(
            speed=0,
            heading=self.heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(0.25)
        self.clearBusy()

    async def turn(self, cmd: str):
        try:
            delta = self.parseTurn(cmd)
        except Exception:
            print("Failed", cmd)
            return
        self.heading = self.normalizeHeading(self.heading+delta)
        await self.turnTo()
        self.writeFrame(0, 0)

    async def snapshot(self, prefix="cam") -> None:
        await self.setBusy()
        path = f"{self.datadir}/{prefix}-{self.frame:04}.jpg"
        self.camera.capture(path, 'jpeg', use_video_port=True)
        self.clearBusy()

    async def next(self) -> None:
        await self.predict()
        await self.snapshot()
        await self.altShots()
        await self.position()
        await self.drive()

    async def predict(self) -> None:
        if self.interp is None:
            return
        await self.setBusy()
        imgdata = io.BytesIO()
        self.camera.capture(imgdata, 'jpeg', use_video_port=True)
        img = image.load_img(
            imgdata,
            target_size=(self.image_height, self.image_width))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = mobilenet_preprocess(x)
        input_details = self.interp.get_input_details()
        output_details = self.interp.get_output_details()
        self.interp.set_tensor(input_details[0]['index'], x)
        self.interp.invoke()
        result = self.interp.get_tensor(output_details[0]['index'])
        self.predictions = result[0].tolist()
        self.clearBusy()

    async def act(self, photos=True, predict=True) -> None:
        max = 0
        best = -1
        for i in range(0, 3):
            if self.predictions[i] > max:
                max = self.predictions[i]
                best = i
        if best == 0:
            await self.turn("t-5")
        elif best == 1:
            await self.turn("t5")
        else:
            if photos:
                await self.next()
            else:
                # await self.position()
                await self.drive()
        if predict:
            await self.predict()

    async def report(self):
        return self.gps.next()

    async def position(self) -> None:
        # Drain the serial line
        while self.gps.waiting():
            try:
                report = await asyncio.wait_for(self.report(), timeout=5.0)
            except asyncio.TimeoutError:
                return None
        count = 0
        while True:
            try:
                report = await asyncio.wait_for(self.report(), timeout=5.0)
            except asyncio.TimeoutError:
                return None
            if report['class'] == "TPV":
                if 'lat' in report:
                    break
                count += 1
                if count == 10:
                    return None
        with open(f"{self.datadir}/position-{self.frame:04}.json", "w") as file:
            json.dump(dict(report), file)
        return dict(report)

    def parseTurn(self, turn: str) -> int:
        if turn[0:4] == "turn":
            if len(turn) == 4:
                return 45
            return int(turn[5:])
        if turn[0] == "t":
            if len(turn) == 1:
                return 45
            return int(turn[1:])
        return 0

    def parseSpeed(self, speed: str) -> int:
        if speed[0:5] == "speed":
            if len(speed) == 5:
                return 128
            return int(speed[6:])
        if speed[0] == "t":
            if len(speed) == 1:
                return 128
            return int(speed[1:])
        return 0

    async def setSpeed(self, speed: str):
        try:
            self.speed = self.parseSpeed(speed)
        except Exception:
            pass

    def normalizeHeading(self, heading: int) -> int:
        while heading < 0:
            heading += 360
        while heading >= 360:
            heading -= 360
        return heading

    async def altShots(self) -> None:
        heading = self.heading
        for delta in [-30, -20, -10, 10, 20, 30]:
            self.heading = self.normalizeHeading(heading+delta)
            await self.turnTo()
            await self.snapshot("neg"+str(delta))
        self.heading = heading
        await self.turnTo()
