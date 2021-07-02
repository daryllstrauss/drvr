from aiohttp import web

class DummyOperations(object):
    def register(self, app):
        app.add_routes([
            web.get('/{tail:.*}', self.get),
            web.post('/{tail:.*}', self.post),
            web.static('/web', 'web', show_index=True)
        ])
    
    @staticmethod
    def get(tail=None):
        print(f"GET {tail}")
    
    @staticmethod
    def post(tail=None):
        print(f"POST {tail}")
