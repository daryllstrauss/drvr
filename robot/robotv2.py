import os
import time
from sphero_sdk.common.commands.drive import drive_with_heading
import cv2
import math
import asyncio
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal, DriveFlagsBitmask, SpheroRvrTargets
from predict import OakD
import numpy as np

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
    turnStep = 3
    moveStep = 0.5
    # settleTime = 0.25
    settleTime = 0
    threshold = 0.4
    minDist = 10
    maxDist = 50
    normalOffset = 120
    currentOffset = 120
    oakD = None
    frame = 1
    prediction = None

    def __init__(self, loop: asyncio.AbstractEventLoop, datadir = "../data"):
        self.datadir = datadir
        self.rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
        self.oakD = OakD()
        self.oakD.startDevice()
        self.datadir = datadir
    
    async def setBusy(self):
        while self.busy:
            await asyncio.sleep(self.settleTime)
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
        print("RVR Startup complete", flush=True)
    
    async def shutdown(self):
        self.oakD.shutdown()
        await self.rvr.close()

    async def battery(self) -> dict:
        await self.setBusy()
        result = {}
        percentage = await self.rvr.get_battery_percentage()
        result.update(percentage)
        state = await self.rvr.get_battery_voltage_state()
        result.update(state)
        self.clearBusy()
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
        await asyncio.sleep(self.settleTime)
        self.clearBusy()

    async def turnTo(self, heading):
        await self.setBusy()
        await self.rvr.drive_with_heading(
            speed=0,
            heading=heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(self.settleTime)
        self.heading = heading
        self.clearBusy()
    
    async def turn(self, num):
        heading = (self.heading + num) % 360
        await self.turnTo(heading)

    def createPredictions(self, data):
        predictions = np.zeros((data.shape[1], data.shape[2]), dtype=np.uint8)
        for y in range(data.shape[1]):
            for x in range(data.shape[2]):
                max = 0
                for p in range(1, data.shape[0]):
                    if data[p][y][x] > data[max][y][x] and data[p][y][x] > self.threshold:
                        max = p
                predictions[y][x] = max
        return predictions

    def createTagImage(self, predictions):
        colors = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [255, 255, 255]]
        image = np.zeros((predictions.shape[0], predictions.shape[1], 3), dtype=np.uint8)
        for y in range(predictions.shape[0]):
            for x in range(predictions.shape[1]):
                image[y][x] = colors[predictions[y][x]]
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image
    
    def scanColumn(self, data, start, type, width=10):
        for y in range(start, -1, -1):
            for x in range(0, width):
                if data[y][x] != type and data[y][x] != 0:
                    break
            return y
        return -1

    def scanRow(self, data, row, type, width=100):
        w = data.shape[1]
        minx = -1
        
        for x in range(0, w):
            if data[row][x] == type:
                stopX = min(x + width, w-1)
                widthMet = True
                for x1 in range(x+1, stopX):
                    if data[row][x1] != type and data[row][x1] != 0:
                        widthMet = False
                        break
                if widthMet or stopX == w-1:
                    minx = x
                    break
        maxx = -1
        # for x in range(w-1, -1, -1):
        #     if data[row][x] == type:
        #         stop = max(x - width, 0)
        #         widthMet = True
        #         for x1 in range(x-1, stop, -1):
        #             if data[row][x1] != type and data[row][x1] != 0:
        #                 widthMet = False
        #                 break
        #         if widthMet or stop == 0:
        #             maxx = x
        #             break
        return (minx, maxx)

    # Robot after move trapazoid
    # (35, 35) (190, 35)
    # (5, 0), (220, 0)
    def analyze(self, data):
        ty = self.scanColumn(data, data.shape[0] - self.normalOffset, 1)
        if ty == -1:
            print("Force left no reference")
            self.heading = (self.heading - 10) % 360
            return "S"
        while ty > 0:
            (minPath, _maxPath) = self.scanRow(data, ty, 1)
            if minPath == -1:
                ty -= 5
            # elif minPath > data.shape[1]/2: 
            #     print("Force left seeing right edge")
            #     self.heading = (self.heading - 10) % 360
            #     return "S"
            else:
                break
        if ty <= 0:
            print("Force right no path in front")
            self.heading = (self.heading + 10) % 360
            return "S"
        self.currentOffset = ty

        tx = self.minDist + (self.maxDist - self.minDist)/2
        offset = ty + 50 # Estimate of distance we'll move forward
        ang = math.atan((minPath - tx) / offset) / (2 * math.pi) * 360
        print("Ang", minPath, tx, offset, ang)
        prev = self.heading
        self.heading = int(self.heading + ang) % 360
        print(f"Header: {prev} to {self.heading}")
        return "S"

    async def predict(self, writeImage=False):
        results = self.oakD.getResult()
        predictions = self.createPredictions(results)
        self.prediction = self.analyze(predictions)
        if writeImage:
            await self.snapshot()
            image = self.createTagImage(predictions)
            h = image.shape[0]
            w = image.shape[1]
            tx = int(self.minDist + (self.maxDist - self.minDist)/2)
            ty = self.currentOffset
            print("TY", ty)
            cv2.line(image, (0, ty), (w-1, ty), (255, 255, 255), 1)
            cv2.line(image, (tx, 0), (tx, h-1), (255, 255, 255), 1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                image,
                f"{int(self.heading)}",
                (predictions.shape[1]-60, 30),
                font,
                1,
                (255, 255, 255),
                1,
                cv2.LINE_AA)
            name = f"{self.datadir}/predict-{str(self.frame).zfill(5)}.jpg"
            cv2.imwrite(name, image)
        return self.prediction

    async def snapshot(self):
        rgb = self.oakD.getRGB()
        name = f"{self.datadir}/rgb-{str(self.frame).zfill(5)}.jpg"
        cv2.imwrite(name, rgb)
        self.frame += 1

    async def next(self):
        if self.prediction == "L":
            await self.turn(-self.turnStep)
        elif self.prediction == "R":
            await self.turn(self.turnStep)
        elif self.prediction == "S":
            await self.drive(self.moveStep)
    
    async def position(self):
        pass

    async def benchmark(self):
        start = time.time()
        count = 100
        for x in range(count):
            await self.predict()
        end = time.time()
        print(f"{count} preidctions in {(end-start)/count} seconds")