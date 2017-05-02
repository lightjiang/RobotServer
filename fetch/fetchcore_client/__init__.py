import logging

from .client import FetchcoreClient
try:
    from .reactor import Reactor
except ImportError:
    logging.warn("An error occured trying to import reactor. This is most " +
                 "likely because you do not have all the ROS and fetch packages " +
                 "installed. You will be able to run fetchcore_client against a " +
                 "running fetchcore server, but robots run from this environment " +
                 "will not work.")
