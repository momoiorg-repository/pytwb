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
class ComPackage:
    name = 'package'
    num_arg = None
    help = 'change current package'
    
    def invoke(self, api, args):
        if len(args) == 0:
            package = api.get_current_package()
            if not package:
                print('no current package')
                return
            print(package)
            print(f'env:\n{package.env}')
            print(f'parameter:\n{package.param}')
            return
        package = api.search_package(os.getcwd(), args[0])
        api.set_current_package(package)
        if package:
            print(package)
        else:
            print('package not found')
    
@command
class ComPls:
    name = 'pls'
    num_arg = None
    help = 'list registered packages'
    
    def invoke(self, api, args):
        for p in api.get_config('packages').values():
            print(p)

@command
class ComDel:
    name = 'del'
    num_arg = None
    help = 'delete registered packages'
    
    def invoke(self, api, args):
        for name in args:
            package = api.search_package(os.getcwd(), name)
            api.delete_package(package)

@command
class ComCreate:
    name = 'create'
    num_arg = 1
    help = 'create ROS package'
    
    def invoke(self, api, args):
        dir = os.getcwd() # get work space address
        if dir.endswith('src'):
            dir = dir[:-4]
        name = args[0]
        if os.path.isdir(os.path.join(dir, f'src/{name}')):
            res = input('package directory already exists. register it?[Y/n]')
            if res.upper() == 'Y':
                api.register(dir, name)
        else:
            api.create(dir, name)

# installation commands
@command
class ComPip3:
    name = 'pip3'
    num_arg = None
    help = 'install Python libraries and record them'
    
    def invoke(self, api, args):
        com = ['pip3'] + args
        subprocess.run(com)
        if len(args) < 2 or args[0] != 'install':
            return
        for t in args[1:]:
            api.add_pip3(t)

@command
class ComApt:
    name = 'apt'
    num_arg = None
    help = 'install apt modules and record them'
    
    def invoke(self, api, args):
        com = ['apt'] + args
        subprocess.run(com)
        if len(args) < 2 or args[0] != 'install':
            return
        for t in args[1:]:
            api.add_apt(t)

@command
class ComConfig:
    name = 'config'
    num_arg = 0
    help = 'print pip3 and apt records'
    
    def invoke(self, api, args):
        print(f'pip3: {api.get_config("pip3")}')
        print(f'apt: {api.get_config("apt")}')

@command
class ComDockerfile:
    name = 'dockerfile'
    num_arg = 0
    help = 'generate dockerfile template to _Dockerfile'
    
    def invoke(self, api, args):
        api.gen_dockerfile()

@command
class ComBehaviors:
    name = 'behaviors'
    num_arg = None
    help = 'print registered behaviors [-l]'
    
    def invoke(self, api, args):
        lopt = False
        if len(args) > 0:
            opt = args[0]
            if opt.startswith('-'):
                opt = opt[1:]
                if opt == 'l':
                    lopt = True
        if lopt:
            for b in api.get_behaviors():
                print(f'{b.name}: {b.bt_class.desc if hasattr(b.bt_class, "desc") else ""}')
        else:
            l = ''
            for b in api.get_behaviors():
                l += f' {b.name}'
            print(l)

@command
class ComEnv:
    name = 'env'
    num_arg = 2
    help = 'set environment parameter'
    
    def invoke(self, api, args):
        api.add_env(args[0], args[1])

@command
class ComParam:
    name = 'param'
    num_arg = 2
    help = 'set ROS node parameter'
    
    def invoke(self, api, args):
        api.add_param(args[0], args[1])

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
