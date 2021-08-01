import os
import sys
import time
import json
import asyncio
import select
from datetime import datetime
# from robotv1 import RobotV1
from robotv2 import RobotV2
from web import server

def create_data_dir():
    now = datetime.now()
    dir = "../data/data-"+format(now, "%Y%m%d-%H%M%S")
    try:
        os.makedirs(dir)
    except OSError:
        pass
    return dir

async def benchmark(robot):
    for _ in range(10):
        start = datetime.now()
        await robot.predict()
        end = datetime.now()
        print("Delta", end-start)

def getNum(s):
    try:
        num = int(s)
    except:
        num = 0
    return num

async def main(robot: RobotV2) -> None:
    run = True
    auto = False
    hasPredict = False
    cnt = 0
    # Let the camera agc/focus initialize
    while cnt < 10:
        robot.predict(False)
        cnt += 1
    cmdstr = "Command [abcpdlgrstgnqhz]: "
    print(cmdstr)
    while run:
        if hasPredict is False:
            await robot.predict(True)
            hasPredict = True
        try:
            if not select.select([sys.stdin,],[],[],0.0)[0]:
                if auto:
                    await robot.next()
                    hasPredict = False
                else:
                    await asyncio.sleep(0.25)
                continue
            cmd = input()
            # await asyncio.sleep(1)
        except Exception:
            print("Input exception", flush=True)
            await asyncio.sleep(30)
            continue
        hasPredict = False
        if cmd == "":
            if auto:
                auto = False
            else:
                await robot.next()
        elif cmd == "a":
            auto = not auto
            print("Autopilot", auto)
        elif cmd == "b" or cmd == "battery":
            bat = await robot.battery()
            print('Battery percentage: ', bat["percentage"])
            print('Voltage state: ', bat["state"])
        elif cmd == "c":
            print(f"turnStep ({robot.turnStep})")
            tsval = input()
            print(f"moveStep ({robot.moveStep})")
            msval = input()
            print(f"Target Distance ({robot.normalOffset})")
            noff = input()
            print(f"Min Left Range ({robot.minDist})")
            mind = input()
            print(f"Max Left Range ({robot.maxDist})")
            maxd = input()
            try:
                robot.turnStep = int(tsval)
                robot.moveStep = float(msval)
                robot.normalOffset = int(noff)
                robot.minDist = int(mind)
                robot.maxDist = int(maxd)
            except:
                print("Invalid number")
        elif cmd == "d" or cmd == "drive":
            await robot.drive(0.5)
        elif cmd[0] == "l":
            await robot.turn(-10)
        elif cmd == "n" or cmd == "next":
            await robot.next()
        elif cmd == "p" or cmd == "picture":
            await robot.snapshot()
        elif cmd == "q" or cmd == "quit":
            await robot.shutdown()
            run = False
        elif cmd[0] == "r":
            await robot.turn(10)
        elif cmd[0] == "s":
            amt = input()
            num = getNum(amt)
            await robot.setSpeed(num)
        elif cmd[0] == "t":
            amt = input()
            num = getNum(amt)
            await robot.turn(num)
        elif cmd == "g" or cmd == "gps":
            pos = await robot.position()
            if pos:
                print(
                    pos.get('lat', 0.0), "\t",
                    pos.get('lon', 0.0), "\t",
                    pos.get('time', ''), "\t",
                    pos.get('alt', 'nan'), "\t",
                    pos.get('epv', 'nan'), "\t",
                    pos.get('ept', 'nan'), "\t",
                    pos.get('speed', 'nan'), "\t",
                    pos.get('climb', 'nan'), "\t")
            else:
                print(None)
        elif cmd == "z":
            await benchmark(robot)
        else:
            print(
                "(b)attery\n"
                "(p)icture\n"
                f"(d)drive {robot.moveStep}\n"
                "(s)peed\n"
                f"(l)left {robot.turnStep}\n"
                f"(r)ight {robot.turnStep}\n"
                "(t)urn\n"
                "(g)ps\n"
                "(n)ext\n"
                "(h)elp\n"
                "(q)uit")
        print(cmdstr)

def mainloop() -> None:
    loop = asyncio.get_event_loop()
    datadir = create_data_dir()
    robot = RobotV2(loop, datadir)
    loop.run_until_complete(robot.startup())
    loop.create_task(server(robot))
    try:
        loop.run_until_complete(main(robot))
    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')
    finally:
        if loop.is_running():
            loop.close()
    loop.run_until_complete(robot.shutdown())


if __name__ == '__main__':
    mainloop()
