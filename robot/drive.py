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
    dir = "data-"+format(now, "%Y%m%d-%H%M%S")
    try:
        os.mkdir(dir)
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
    print("Command [bpdstgnqh]: ")
    while run:
        try:
            if not select.select([sys.stdin,],[],[],0.0)[0]:
                await asyncio.sleep(1)
                continue
            cmd = input()
        except Exception:
            await asyncio.sleep(3600)
            continue
        if cmd == "b" or cmd == "battery":
            bat = await robot.battery()
            print('Battery percentage: ', bat["percentage"])
            print('Voltage state: ', bat["state"])
        elif cmd == "p" or cmd == "picture":
            await robot.snapshot()
        elif cmd == "d" or cmd == "drive":
            await robot.drive()
        elif cmd[0] == "s":
            num = getNum(cmd[1:])
            await robot.setSpeed(num)
        elif cmd[0] == "t":
            num = getNum(cmd[1:])
            await robot.turn(num)
        elif cmd[0] == "l":
            await robot.turn(-10)
        elif cmd[0] == "r":
            await robot.turn(10)
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
        elif cmd == "n" or cmd == "next":
            await robot.next()
        elif cmd == "q" or cmd == "quit":
            await robot.shutdown()
            run = False
        elif cmd == "z":
            await benchmark(robot)
        else:
            print(
                "(b)attery\n"
                "(p)icture\n"
                "(d)drive\n"
                "(s)peed\n"
                "(l)left 10\n"
                "(r)ight 10\n"
                "(t)urn\n"
                "(g)ps\n"
                "(n)ext\n"
                "(h)elp\n"
                "(q)uit")
        print("Command [bpdstgnqh]: ")


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
