import json
import threading
from binascii import b2a_hex

from django.views.decorators.csrf import csrf_exempt
from core.RequestHandler import myview
from dwebsocket.decorators import accept_websocket
from home.apihandler import ApiHandler
from script.MQhandler import MQSend
from script.MapHandler import MapHandler

# Create your views here.

@myview()
def index(request, handler=None):
    return handler.render("home/index.html")


@accept_websocket
def echo(request):
    if request.is_websocket:
        map_handler = MapHandler()
        map_handler.mark_node(x=0.283, y=2.86)
        from RobotsServer.settings import BASE_DIR
        path = BASE_DIR + "/home/static/home/img/1.jpg"
        map_handler.static_map.save(path)
        lock = threading.RLock()
        try:
            lock.acquire()
            client = request.websocket
            for message in client:

                if not message:
                    break
                else:
                    message = json.loads(message)
                    mq = MQSend()
                    me = mq.message("BaseMove", **message)
                    mq.send(message=me)
        finally:
            lock.release()


@myview()
@csrf_exempt
def api(request, var, handler=None):
    handler.jsonresponse({"status": -1, "response_text": "unknown error"})
    api_handler = ApiHandler(handler)
    var = var.lower()
    obj = getattr(api_handler, var)
    obj()
