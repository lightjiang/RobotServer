# coding=utf-8
from django.core.handlers.wsgi import WSGIRequest
from django.shortcuts import render, redirect
from django.http import HttpResponseRedirect, JsonResponse
from django.http.response import HttpResponse
from django.core.urlresolvers import reverse
from functools import wraps
from core.models import AccessTrack


class RequestHandler(object):
    """
    本类可以作为基类使用
    用特殊返回要求可以重写response函数
    """

    def __init__(self, request):
        """
        初始化Handler类
        :type request: WSGIRequest
        """
        self.request = request  # 请求request
        self.__response = HttpResponseRedirect(reverse("home:index"))  # 要返回的response 默认为home
        self.cookies = self.request.COOKIES  # 读取的cookie
        self.session = self.request.session  # session
        self.post = self.request.POST.dict()  # POST字典
        self.get = self.request.GET.dict()  # GET字典
        self.response_cookies = {}  # 要设置的cookie

    def __track(self):
        if 'HTTP_X_FORWARDED_FOR' in self.request.META:
            ip = self.request.META['HTTP_X_FORWARDED_FOR']
        else:
            ip = self.request.META['REMOTE_ADDR']
        ap = self.request.get_full_path()
        if "history_request" not in self.session:
            self.session["history_request"] = []
        elif len(self.session["history_request"]) > 40:
            del self.session["history_request"][0:20]
        self.session["history_request"].append(ap)
        tem = AccessTrack(active_position=ap, active_ip=ip)
        tem.save()

    def response(self, mode=1):
        """
        返回一个response，该过程中会
        有两个模式:
        1  带返回前判断和返回后判断。
        其他  不判断直接返回
        :param mode:
        :return:
        """
        if mode:
            self.__check_response_before()
            self.push_cookies()
            self.__check_response_after()
        else:
            self.push_cookies()
        self.__track()
        return self.__response

    def push_cookies(self):
        """
        添加cookies
        向前端添加cookies
        需要添加的cookie 请放在response_cookies中，cookie默认100天（如果我没算错的话）。
        @return:
        """
        for i in self.response_cookies:
            self.__response.set_cookie(i, self.response_cookies[i], max_age=60 * 60 * 24 * 100)
        self.__response.delete_cookie("django_language")

    def __check_response_before(self):
        self.set_cookie("stat", 1)

    def __check_response_after(self):
        pass

    # 会使用到的response类型，进行封装
    def render(self, template_name, context=None):
        self.__response = render(self.request, template_name, context=context)

    def redirect(self, to, *args, **kwargs):
        self.__response = redirect(to, *args, **kwargs)

    def httpresponseredirect(self, redirect_to, *args, **kwargs):
        self.__response = HttpResponseRedirect(redirect_to, *args, **kwargs)

    def jsonresponse(self, data):
        self.__response = JsonResponse(data)

    def httpresponse(self, content=b'', *args, **kwargs):
        self.__response = HttpResponse(content=content, *args, **kwargs)

    @staticmethod
    def reverse(viewname, urlconf=None, args=None, kwargs=None, current_app=None):
        return reverse(viewname, urlconf=urlconf, args=args, kwargs=kwargs, current_app=current_app)

    def set_cookie(self, key, value):
        """
        给自身的response_cookies设置值
        :param key:
        :param value:
        :return:
        """
        self.response_cookies[key] = value

    def set_stat(self, stat, app=None):
        pass

    def clean_all(self):
        """清理缓存的cookie session"""
        self.session.clear()
        self.response_cookies = {}
        self.cookies = {}


def myview():
    def myfunc(func):
        @wraps(func)
        def wraper(request, *args, **kwargs):
            handler = RequestHandler(request)
            response = func(request, handler=handler, *args, **kwargs)
            if response:
                return response
            else:
                return handler.response()
        return wraper
    return myfunc
