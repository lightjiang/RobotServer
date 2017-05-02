import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.template
import tornado.options
import tornado.httpserver
import time as t
import csv

import frmoves as fr
import ffmove as ff
import positions as p

import os

global csv_positions
global with_freight

root = os.path.dirname(__file__)

class MainHandler(tornado.web.RequestHandler):
  def get(self):
    loader = tornado.template.Loader(".")
    #print self.request.headers.get('Referer') + 'ddd'

    #if self.request.headers.get('Referer') == '/index.html':
    #if with_freight == True:
        #self.write(loader.load("index.html").generate())
    #elif self.request.headers.get('Referer') == '/select.html':
    #    self.write(loader.load("select.html").generate())

class WSHandler(tornado.websocket.WebSocketHandler):
    global csv_positions
    connections = set()
    message = ""

    def fr(self, mopt):
        global with_freight
        msg = ""
        #f = fr.FRGoto()
        #print mopt

        opt = int(mopt)

        pos = opt + 1
        print "pos:",pos
        if pos >= 0 and pos <= 1:
            #print pos
            msg = "1:Moving to position " + csv_positions.data[pos].position + "," + csv_positions.data[pos].position
            if with_freight == True:
                f = fr.FRGoto()
                [con.write_message(msg) for con in self.connections]
                f.movetopoint(csv_positions.data[pos].position)


            msg = "Movement complete to " + csv_positions.data[pos].location_name
        if pos >= 2 and pos <= 4:
            #print pos
            msg = "1:Moving to position " + csv_positions.data[pos].position + "," + csv_positions.data[pos].position
            if with_freight == True:
                f = ff.FRGoto1()
                [con.write_message(msg) for con in self.connections]
                f.movetopoint(csv_positions.data[pos].position)


            msg = "Movement complete to " + csv_positions.data[pos].location_name
        else:
            msg = "Wrong selection"

        return msg

    def uinterface(self, mopt):
        print "mopt:",mopt
        global with_freight
        #print "User Interface"
        if mopt == '201':
            self.fr(-1)
            #self.message = "Moving to start position"

        else:
            print "ok"
            self.fr(0)
            self.message = ""

        return self.message

    def open(self):
        self.connections.add(self)
        print 'New connection was opened'

    def on_message(self, message):
        #print message
        #return

        global csv_positions

        arg0 = message.split(',')
        #print "arg0:",arg0
        obj_id = arg0[0].split('=')[1]
        print"obj_id",obj_id

        if obj_id == "300":
            print arg0[1].split('&')[0].split('=')[1]
            print arg0[1].split('&')[1].split('=')[1]
            print arg0[1].split('&')[2].split('=')[1]

            csv_positions.data[1].quantity = arg0[1].split('&')[0].split('=')[1]
            csv_positions.data[2].quantity = arg0[1].split('&')[1].split('=')[1]
            csv_positions.data[3].quantity = arg0[1].split('&')[2].split('=')[1]

        obj_value = arg0[1].split('&')[0].split('=')[1]

        obj_value_ = arg0[1].split('&')
        #print "obj_value_:",obj_value_

        rmessage = ''

        if obj_id >= "100" and obj_id < "200":
            if obj_value == "100" or obj_value == "111":
                print "obj_value:",obj_value
                self.fr(obj_value)
                rmessage = "200:" + csv_positions.data[1].quantity + "," + csv_positions.data[2].quantity + "," + \
                           csv_positions.data[3].quantity
            if obj_id == "100":
                print "obj_value:", obj_value
                self.fr(obj_value)
                rmessage = "200:" + csv_positions.data[1].quantity + "," + csv_positions.data[2].quantity + "," + \
                           csv_positions.data[3].quantity
        #elif obj_id == "211":
            #rmessage = "211:"
        elif obj_id >= "300" and obj_id < "400":
            self.uinterface(obj_value)
            rmessage = "200:" + csv_positions.data[1].quantity + "," + csv_positions.data[2].quantity + "," + \
                       csv_positions.data[3].quantity
        elif obj_id == "1":
            print "obj_value:   "+obj_value
            rmessage = "0:" + self.uinterface(obj_value)
        else:
            rmessage = "500:"+"Forbidden: " + str(obj_id)

        #print rmessage
        [con.write_message(rmessage) for con in self.connections]

    def on_close(self):
        print 'connection closed...'

    def check_origin(self, origin):
        return True

application = tornado.web.Application([
  (r'/ws', WSHandler),
  (r'/', MainHandler),
  (r"/(.*)", tornado.web.StaticFileHandler, {"path": "./resources"}),
], static_path=os.path.join(root, 'static'))

if __name__ == "__main__":
  global csv_positions
  global with_freight
  with_freight = True


  csv_positions = p.Positions('positions.csv')

  #if with_freight == True:
  #    f = fr.FRGoto()
  #    f.movetopoint(csv_positions.data[0].position)

  tornado.options.parse_command_line()
  app = tornado.web.Application(handlers=[(r"/", MainHandler)])
  http_server = tornado.httpserver.HTTPServer(app)

  application.listen(9090)
  tornado.ioloop.IOLoop.instance().start()