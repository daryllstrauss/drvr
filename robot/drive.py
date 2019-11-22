
import time
import json
import asyncio
from datetime import datetime
from picamera import PiCamera  #pylint: disable = import-error
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates
from sphero_sdk import DriveFlagsBitmask


async def battery(rvr: SpheroRvrAsync) -> None:
    battery_percentage = await rvr.get_battery_percentage()
    print('Battery percentage: ', battery_percentage)

    battery_voltage_state = await rvr.get_battery_voltage_state()
    print('Voltage state: ', battery_voltage_state)

    state_info = '['\
        f'{VoltageStates.unknown.name}: {VoltageStates.unknown.value}, '\
        f'{VoltageStates.ok.name}: {VoltageStates.ok.value}, '\
        f'{VoltageStates.low.name}: {VoltageStates.low.value}, '\
        f'{VoltageStates.critical.name}: {VoltageStates.critical.value}]'
    print('Voltage states: ', state_info)


async def drive(rvr: SpheroRvrAsync, frame: int, speed: int, heading:int) -> None:
    await rvr.drive_with_heading(
        speed=speed,  # Valid speed values are 0-255
        heading=heading,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.drive_reverse.value
    )
    await asyncio.sleep(1)
    await rvr.drive_with_heading(
        speed=0,  # Valid speed values are 0-255
        heading=heading,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.drive_reverse.value
    )
    await asyncio.sleep(1)
    command = {
        "heading": heading,
        "speed": speed,
        "duration": 1
    }
    with open(f"data/command-{frame:04}.json", "w") as file:
        json.dump(command, file)


async def turn(rvr: SpheroRvrAsync, heading: int):
    await rvr.drive_with_heading(
        speed=0,  # Valid speed values are 0-255
        heading=heading,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )
    await asyncio.sleep(1)


def snapshot(camera: PiCamera, frame: int, prefix="cam") -> None:
    camera.capture(f"data/{prefix}-{frame:04}.jpg")


async def position(gpsd: gps, frame: int) -> None:
    # Drain the serial line
    while gpsd.waiting():
        gpsd.next()
    count = 0
    while True:
        report = gpsd.next()
        if report['class'] == "TPV":
            break
        count += 1
        if count == 100:
            return None
    print(
        report.get('lat', 0.0), "\t",
        report.get('lon', 0.0), "\t",
        report.get('time', ''), "\t",
        report.get('alt', 'nan'), "\t",
        report.get('epv', 'nan'), "\t",
        report.get('ept', 'nan'), "\t",
        report.get('speed', 'nan'), "\t",
        report.get('climb', 'nan'), "\t")
    with open(f"data/position-{frame:04}.json", "w") as file:
        json.dump(dict(report), file)
    return report


def parseTurn(turn: str) -> int:
    if turn[0:4] == "turn":
        if len(turn) == 4:
            return 45
        return int(turn[5:])
    if turn[0] == "t":
        if len(turn) == 1:
            return 45
        return int(turn[1:])
    return 0


def parseSpeed(speed: str) -> int:
    if speed[0:5] == "speed":
        if len(speed) == 5:
            return 128
        return int(speed[6:])
    if speed[0] == "t":
        if len(speed) == 1:
            return 128
        return int(speed[1:])
    return 0


def normalizeHeading(heading: int) -> int:
    while heading < 0:
        heading += 360
    while heading >= 360:
        heading -= 360
    return heading


async def altShots(rvr: SpheroRvrAsync, camera: PiCamera, heading: int, frame: int) -> None:
    for delta in [-30, -20, -10, 10, 20, 30]:
        newheading = normalizeHeading(heading+delta)
        await turn(rvr, newheading)
        snapshot(camera, frame, "neg"+str(delta))
    await turn(rvr, heading)


async def main(rvr: SpheroRvrAsync) -> None:
    await rvr.wake()
    camera = PiCamera(resolution=(1280, 720), framerate=30)
    await rvr.reset_yaw()
    gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
    await asyncio.sleep(2)
    run = True
    heading = 0
    speed = 100
    frame = 0
    while run:
        cmd = input("Command [bpdstgnqh]: ")
        if cmd == "b" or cmd == "battery":
            await battery(rvr)
        elif cmd == "p" or cmd == "picture":
            snapshot(camera, frame)
        elif cmd == "d" or cmd == "drive":
            await drive(rvr, frame, speed, heading)
        elif cmd[0] == "s":
            try:
                speed = parseSpeed(cmd)
            except Exception:
                pass
        elif cmd[0] == "t":
            try:
                heading += parseTurn(cmd)
                heading = normalizeHeading(heading)
                await turn(rvr, heading)
            except Exception:
                pass
        elif cmd == "g" or cmd == "gps":
            await position(gpsd, frame)
        elif cmd == "n" or cmd == "next":
            frame += 1
            snapshot(camera, frame)
            await altShots(rvr, camera, heading, frame)
            await position(gpsd, frame)
            await drive(rvr, frame, speed, heading)
        elif cmd == "q" or cmd == "quit":
            run = False
        else:
            print(
                "(b)attery\n"
                "(p)icture\n"
                "(d)drive\n"
                "(s)peed\n"
                "(t)urn\n"
                "(g)ps\n"
                "(n)ext\n",
                "(h)elp\n"
                "(q)uit")
        await asyncio.sleep(0)
    await rvr.close()


def mainloop() -> None:
    loop = asyncio.get_event_loop()
    rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))

    try:
        loop.run_until_complete(main(rvr))

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')
        loop.run_until_complete(rvr.close())

    finally:
        if loop.is_running():
            loop.close()


if __name__ == '__main__':
    mainloop()
