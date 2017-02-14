import json
import threading
from django.views.decorators.csrf import csrf_exempt
from core.RequestHandler import myview
from dwebsocket.decorators import accept_websocket
from home.apihandler import ApiHandler
from src.MQhandler import MQSend


# Create your views here.

@myview()
def index(request, handler=None):
    return handler.render("home/index.html")


@accept_websocket
def echo(request):
    if request.is_websocket:
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
