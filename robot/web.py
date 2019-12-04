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
    result = {
        "directory": robot.datadir,
        "frame": robot.frame,
        "battery": bat,
        "position": pos,
        "predictions": robot.predictions}
    return result

async def battery(request):
    robot = request.app['robot']
    bat = await robot.battery()
    return web.json_response(bat)

async def turn(request):
    robot = request.app['robot']
    data = await request.post()
    amount = data['amount']
    await robot.turn("t"+amount)
    await robot.predict()
    return web.HTTPFound('/')

async def next(request):
    robot = request.app['robot']
    await robot.next()
    await robot.predict()
    return web.HTTPFound('/')

async def position(request):
    robot = request.app['robot']
    pos = await robot.position()
    return web.json_response(dict(pos))

async def image(request):
    robot = request.app['robot']
    last = f"{robot.datadir}/cur.jpg"
    try:
        with open(last, "rb") as f:
            body = f.read()
        headers = {
            "Cache-Control": "no-cache"
        }
        resp = web.Response(body=body, content_type="image/jpeg", headers=headers)
        return resp
    except Exception:
        return web.Response()

async def act(request):
    robot = request.app['robot']
    await robot.act()
    return web.HTTPFound('/')

async def correct(request):
    robot = request.app['robot']
    data = await request.post()
    direction = data['direction']
    if direction == "left":
        await robot.snapshot("negcorrect")
        await robot.turn("t-5")
    else:
        await robot.snapshot("neg-correct")
        await robot.turn("t5")
    await robot.predict()
    return web.HTTPFound('/')

async def halt(request):
    os.system("sudo halt")
    return web.HTTPFound('/')

async def reboot(request):
    os.system("sudo reboot")
    return web.HTTPFound('/')

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
        web.post('/correct', correct),
        web.post('/act', act),
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
