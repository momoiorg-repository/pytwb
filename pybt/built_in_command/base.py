import os

from ..common import command
from . import Com
from ..py_tree_loader import TreeLoader

@command
class ComEcho(Com):
    name = 'echo'
    num_arg = None
    help = 'print arguments'

    def invoke(self, api, args):
        mes = ' '.join(args)
        print(mes)

@command
class ComRun(Com):
    name = 'run'
    num_arg = 1
    help = 'run tree_name (without .xml)'

    def invoke(self, api, args):
        api.run(args[0])

@command
class ComCd:
    name = 'cd'
    num_arg = 1
    help = 'change working directory'
    
    def inovke(self, api, args):
        dir = args[0]
        if dir.startswith('~'):
            dir = os.path.expanduser(dir)
        api.change_dirctory(dir)
