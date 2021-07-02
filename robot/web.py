import os
import asyncio
from aiohttp import web
import aiohttp_mako
from robotv2 import RobotV2
# from operationsv1 import OperationsV1
from dummyops import DummyOperations

async def server(robot: RobotV2):
    app = web.Application()
    app['robot'] = robot
    # ops = OperationsV1()
    ops = DummyOperations()
    ops.register(app)
    lookup = aiohttp_mako.setup(
        app,
        input_encoding='utf-8',
        output_encoding='utf-8',
        default_filters=['decode.utf8'])
    with open("page.mako") as f:
        template = f.read()
    lookup.put_string('index.html', template)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    while True:
        await asyncio.sleep(1)
