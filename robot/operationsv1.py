import os
from aiohttp import web
import aiohttp_mako

class OperationsV1(object):
    def register(self, app):
        app.add_routes([
            web.get('/', self.handle),
            web.get('/image', self.image),
            web.post('/turn', self.turn),
            web.post('/next', self.next),
            web.post('/api/position', self.position),
            web.post('/correct', self.correct),
            web.post('/act', self.act),
            web.post('/halt', self.halt),
            web.post('/reboot', self.reboot),
            web.get('/api/battery', self.battery),
            web.post('/api/predict', self.predict),
            web.post('/api/act', self.act_api),
            web.post('/api/turn', self.turn_api),
            web.post('/api/straight', self.straight_api),
            web.post('/api/halt', self.halt_api),
            web.post('/api/record', self.record_api),
            web.static('/web', 'web', show_index=True)
        ])

    @staticmethod
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

    @staticmethod
    async def battery(request):
        robot = request.app['robot']
        bat = await robot.battery()
        return web.json_response(bat)

    @staticmethod
    async def turn(request):
        robot = request.app['robot']
        data = await request.post()
        amount = data['amount']
        await robot.turn("t"+amount)
        await robot.predict()
        return web.HTTPFound('/')

    @staticmethod
    async def turn_api(request):
        robot = request.app['robot']
        data = await request.json()
        amount = data['amount']
        await robot.turn("t"+str(amount))
        return web.Response()

    @staticmethod
    async def straight_api(request):
        robot = request.app['robot']
        await robot.drive()
        return web.Response()

    @staticmethod
    async def record_api(request):
        robot = request.app['robot']
        await robot.next()
        return web.Response()

    @staticmethod
    async def next(request):
        robot = request.app['robot']
        await robot.next()
        await robot.predict()
        return web.HTTPFound('/')

    @staticmethod
    async def position(request):
        robot = request.app['robot']
        pos = await robot.position()
        return web.json_response(dict(pos))

    @staticmethod
    async def image(request):
        robot = request.app['robot']
        last = f"{robot.datadir}/cur.jpg"
        try:
            with open(last, "rb") as f:
                body = f.read()
            headers = {
                "Cache-Control": "no-cache"
            }
            resp = web.Response(
                body=body,
                content_type="image/jpeg",
                headers=headers)
            return resp
        except Exception:
            return web.Response()

    @staticmethod
    async def act(request):
        robot = request.app['robot']
        await robot.act()
        return web.HTTPFound('/')

    @staticmethod
    async def act_api(request):
        robot = request.app['robot']
        data = await request.json()
        photos = data.get('photos', False)
        predict = data.get('predict', False)
        await robot.act(photos, predict)
        return web.Response()

    @staticmethod
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

    @staticmethod
    async def predict(request):
        robot = request.app['robot']
        await robot.predict()
        return web.json_response(robot.predictions)

    @staticmethod
    async def halt(request):
        os.system("sudo halt")
        return web.HTTPFound('/')

    @staticmethod
    async def halt_api(request):
        os.system("sudo halt")
        return web.Response()

    @staticmethod
    async def reboot(request):
        os.system("sudo reboot")
        return web.HTTPFound('/')