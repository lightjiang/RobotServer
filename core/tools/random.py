# coding=utf-8
from Crypto.Random import random


def random_str(length=16, digital=0):
    """
    随机字符串
    :param length:
    :return:
    @param length:
    @param digital:
    """
    seed = random.StrongRandom()
    chars = u'AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz0123456789'
    if digital:
        chars = u"0123456789"
    res = seed.sample(chars, length)
    return "".join(res)
