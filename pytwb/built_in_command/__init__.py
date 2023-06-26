from abc import abstractmethod

class Com:
    @abstractmethod
    def invoke(self, api, args):
        pass

from . import base, use