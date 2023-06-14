import os
import subprocess

from ..common import command
from . import Com
from ..py_tree_loader import TreeLoader

# native commands
@command
class ComRun(Com):
    name = 'run'
    num_arg = 1
    help = 'run tree_name (without .xml)'

    def invoke(self, api, args):
        api.run(args[0])

@command
class ComBase:
    name = 'base'
    num_arg = 1
    help = 'change base directory'
    
    def invoke(self, api, args):
        dir = args[0]
        if dir.startswith('~'):
            dir = os.path.expanduser(dir)
        dir = os.path.abspath(dir)
        api.change_directory(dir)

@command
class ComCreate:
    name = 'create'
    num_arg = 1
    help = 'create ROS package'
    
    def invoke(self, api, args):
        dir = os.getcwd() # get work space address
        if dir.endswith('src'):
            dir = dir[:-4]
        api.create(dir, args[0])

# installation commands
@command
class ComPip3:
    name = 'pip3'
    num_arg = None
    help = 'install Python libraries and record them'
    
    def invoke(self, api, args):
        if len(args) < 2 or args[0] != 'install':
            com = ['pip3'] + args
            subprocess.run(com)
            return
        for t in args[1:]:
            api.set_config('pip3', t)

@command
class ComApt:
    name = 'apt'
    num_arg = None
    help = 'install apt modules and record them'
    
    def invoke(self, api, args):
        if len(args) < 2 or args[0] != 'install':
            com = ['apt'] + args
            subprocess.run(com)
            return
        for t in args[1:]:
            api.set_config('apt', t)

# linux compatible commands
class CmdLinux(Com):
    def invoke(self, api, args):
        com = [self.name] + args
        subprocess.run(com)
    
@command
class ComCd(Com):
    name = 'cd'
    num_arg = None
    help = 'change current directory'
    
    def invoke(self, api, args):
        if len(args) >= 1:
            dir = args[0]
        else:
            dir = '~'
        if dir.startswith('~'):
            dir = os.path.expanduser(dir)
        dir = os.path.abspath(dir)
        os.chdir(dir)

@command
class ComPwd(Com):
    name = 'pwd'
    num_arg = 0
    help = 'print current directory'
    
    def invoke(self, api, args):
        print(os.getcwd())

@command
class ComLs(CmdLinux):
    name = 'ls'
    num_arg = None
    help = 'print file list'

@command
class ComEcho(CmdLinux):
    name = 'echo'
    num_arg = None
    help = 'print arguments'
