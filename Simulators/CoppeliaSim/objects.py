from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from threading import Thread, Event

from Helpers.log import Log

class CoppeliaObj(Thread):
    def __init__(self, obj: str, isThread: bool = True) -> None:
        if isThread:
            Thread.__init__(self)
            self.daemon = True
            self.event = Event()
        Log(f'Init Coppelia {obj}...')
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')