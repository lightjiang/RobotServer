#!/usr/bin/python

import rospy

from fetchcore_client.reactor import Reactor

if __name__ == "__main__":

    # TODO: Potentially move loop control to here for easy SIGTERM?
    rospy.init_node("reactor")

    reactor = Reactor()
    reactor.start()
