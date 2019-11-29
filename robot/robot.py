import asyncio
import json
from picamera import PiCamera  #pylint: disable = import-error
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates
from sphero_sdk import DriveFlagsBitmask

class Robot(object):
    datadir = None
    rvr = None
    camera = None
    gps = None
    heading = 0
    speed = 100
    frame = 0
    duration = 1

    def __init__(self, loop: asyncio.AbstractEventLoop, datadir: str = "data"):
        self.datadir = datadir
        self.rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
        self.camera = PiCamera(resolution=(1280, 720), framerate=30)
        self.gps = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)

    async def startup(self):
        await self.rvr.wake()
        await self.rvr.reset_yaw()
        await asyncio.sleep(1)

    async def shutdown(self):
        await self.rvr.close()

    async def battery(self) -> dict:
        result = {}
        percentage = await self.rvr.get_battery_percentage()
        result.update(percentage)
        state = await self.rvr.get_battery_voltage_state()
        result.update(state)
        return result

    async def drive(self) -> None:
        await self.rvr.drive_with_heading(
            speed=self.speed,
            heading=self.heading,
            flags=DriveFlagsBitmask.drive_reverse.value)
        await asyncio.sleep(self.duration)
        await self.rvr.drive_with_heading(
            speed=0,
            heading=self.heading,
            flags=DriveFlagsBitmask.drive_reverse.value)
        await asyncio.sleep(1)
        command = {
            "heading": self.heading,
            "speed": self.speed,
            "duration": self.duration
        }
        with open(f"{self.datadir}/command-{self.frame:04}.json", "w") as file:
            json.dump(command, file)
        await self.rvr.drive_with_heading(
            speed=0,
            heading=self.heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(1)
    
    async def turnTo(self):
        await self.rvr.drive_with_heading(
            speed=0,
            heading=self.heading,
            flags=DriveFlagsBitmask.none.value)
        await asyncio.sleep(1)
    
    async def turn(self, cmd: str):
        try:
            delta = self.parseTurn(cmd)
        except Exception:
            print("Failed", cmd)
            return
        self.heading = self.normalizeHeading(self.heading+delta)
        await self.turnTo()

    async def snapshot(self, prefix="cam") -> None:
        self.camera.capture(f"{self.datadir}/{prefix}-{self.frame:04}.png")

    async def position(self) -> None:
        # Drain the serial line
        while self.gps.waiting():
            report = self.gps.next()
        count = 0
        while True:
            report = self.gps.next()
            count += 1
            if count == 10:
                return None
            if report['class'] == "TPV" and 'lat' in report:
                break
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

    async def next(self):
        self.frame += 1

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
