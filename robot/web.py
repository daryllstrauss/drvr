import os
import asyncio
from aiohttp import web
import aiohttp_mako
from robot import Robot

@aiohttp_mako.template('index.html')
async def handle(request):
    robot = request.app['robot']
    bat = await robot.battery()
    pos = await robot.position()
    return {"frame": robot.frame, "battery": bat, "position": pos}

async def battery(request):
    robot = request.app['robot']
    bat = await robot.battery()
    return web.json_response(bat)

async def turn(request):
    robot = request.app['robot']
    data = await request.post()
    amount = data['amount']
    await robot.turn("t"+amount)
    return web.HTTPFound('/')

async def next(request):
    robot = request.app['robot']
    await robot.next()
    await robot.drive()
    await robot.snapshot()
    await robot.altShots()
    await robot.position()
    return web.HTTPFound('/')

async def position(request):
    robot = request.app['robot']
    pos = await robot.position()
    return web.json_response(dict(pos))

async def image(request):
    robot = request.app['robot']
    last = f"data/cam-{robot.frame:04}.jpg"
    with open(last, "rb") as f:
        body = f.read()
    resp = web.Response(body=body, content_type="image/jpg")
    return resp

async def halt(request):
    os.system("sudo halt")

async def reboot(request):
    os.system("sudo reboot")

async def server(robot: Robot):
    app = web.Application()
    app['robot'] = robot
    app.add_routes([
        web.get('/', handle),
        web.get('/image', image),
        web.get('/battery', battery),
        web.post('/turn', turn),
        web.post('/next', next),
        web.post('/position', position),
        web.post('/halt', halt),
        web.post('/reboot', reboot)
    ])
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
        await asyncio.sleep(0)
