# coding=utf-8
"""
20170216
light

"""
import rospy
from PIL import Image
from PIL import ImageDraw
from nav_msgs.srv import GetMap


class MapHandler(object):
    def __init__(self):
        rospy.wait_for_service('static_map')
        self.static_srv = rospy.ServiceProxy('static_map', GetMap)
        self.static_map = None
        self.width = 0
        self.height = 0
        self.orign_x = 0
        self.orign_y = 0
        self.resolution = 0
        self.update_static_map()

    def update_static_map(self):
        data = self.static_srv.call().map
        self.width = data.info.width
        self.height = data.info.height
        print(self.width, self.height, data.info)
        self.resolution = 1.0/data.info.resolution
        self.orign_x = abs(data.info.origin.position.x)
        self.orign_y = abs(data.info.origin.position.y)
        self.static_map = Image.new("RGBA", (self.width, self.height), color=(255, 255, 255))
        for x in range(0, self.width):
            for y in range(0, self.height):
                i = data.data[x + (self.height - y - 1) * self.width]       # 看map_server源码得来的,,我也不知道是按什么顺序排的
                if i == -1:
                    color = (255, 255, 255)
                elif i == 100:
                    color = (0, 0, 0)
                elif i == 0:
                    color = (200, 200, 200)
                else:
                    color = (255, 255, 255)
                self.static_map.putpixel((x, y), color)
    
    def mark_node(self, x=0.0, y=0.0):
        print(self.orign_x, self.orign_y)
        self.orign_x = 23.7
        self.orign_y = 24.3
        x = (x + self.orign_x) * self.resolution
        y = self.height - (self.orign_y + y)*self.resolution
        draw = ImageDraw.Draw(self.static_map)
        draw.ellipse([x-3, y-3, x+3, y+3], fill="red")


if __name__ == '__main__':
    h = MapHandler()
    # h.mark_node(x=0.283, y=2.86)
    h.mark_node(x=0, y=0)
    h.static_map.show()
