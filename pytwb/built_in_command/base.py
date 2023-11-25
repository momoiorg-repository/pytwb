import os
import subprocess

import py_trees

from ..common import command
from . import Com

def print_multi(src, align=4):
    l = ''
    count = 0
    for b in sorted(src):
        l += b
        bc = len(b)
        sc = align-bc%align
        for _ in range(sc):
            l += ' '
        count += bc + sc
        if count > 64:
            print(l)
            count = 0
            l = ''
    if len(l) > 0:
        print(l)

def copy_file(source, destination):
    with open(source, 'rb') as inf:
        with open(destination, 'wb') as ouf:
            ouf.write(inf.read())

# native commands
@command
class ComRun(Com):
    name = 'run'
    num_arg = 1
    help = 'run tree_name (without .xml)'

    def invoke(self, api, args):
        api.exec(args[0])

@command
class ComPackage:
    name = 'package'
    num_arg = None
    help = 'package [package_name] :: change current package'
    
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
    help = 'pls :: list registered packages'
    
    def invoke(self, api, args):
        for p in api.get_config('packages').values():
            print(p)

@command
class ComDel:
    name = 'del'
    num_arg = None
    help = 'del package_name :: delete registered packages'
    
    def invoke(self, api, args):
        for name in args:
            package = api.search_package(os.getcwd(), name)
            api.delete_package(package)

@command
class ComCreate:
    name = 'create'
    num_arg = 1
    help = 'create new_package_name :: create ROS package'
    
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

@command
class ComBb:
    name = 'bb'
    num_arg = None
    help = 'bb key [format]* :: get bloackboard value'

    def __init__(self) -> None:
        self.bb = py_trees.blackboard.Blackboard()
    
    def invoke(self, api, args):
        key = args.pop(0)
        if not self.bb.exists(key):
            print("key does not exist")
            return
        val = self.bb.get(key)
        if len(args) < 1:
            print(val)
            return
        for a in args:
            print(f'{a}= {eval(a)}')        

@command
class ComConfig:
    name = 'config'
    num_arg = 0
    help = 'config :: print pip3 and apt records'
    
    def invoke(self, api, args):
        print(f'pip3: {api.get_config("pip3")}')
        print(f'apt: {api.get_config("apt")}')

@command
class ComBehaviors:
    name = 'behaviors'
    num_arg = None
    help = 'behaviors [-l] :: print registered behaviors'
    
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
            blist = []
            for b in api.get_behaviors():
                blist.append(b.name)
            print_multi(blist)

@command
class ComTrees:
    name = 'trees'
    num_arg = 0
    help = 'trees :: print name of registered XML files'

    def invoke(self, api, args):
        tlist = []
        for t in api.get_trees():
            tlist.append(t.split('/')[-1])
        print_multi(tlist)

@command
class ComEnv:
    name = 'env'
    num_arg = 2
    help = 'env key value :: set environment parameter'
    
    def invoke(self, api, args):
        os.environ[args[0]] = args[1]

@command
class ComParam:
    name = 'param'
    num_arg = 2
    help = 'param key value :: set ROS node parameter'
    
    def invoke(self, api, args):
        api.add_param(args[0], args[1])

@command
class ComLog:
    name = 'log'
    num_arg = None
    help = 'log [dir_name] :: start/stop log. log dir_name or log -'
    
    def invoke(self, api, args):
        if len(args) == 0:
            if hasattr(os.environ, 'ROS_LOG_DIR'):
                print(os.environ['ROS_LOG_DIR'])
            else:
                print('log off')
            return
        dir = args[0]
        if dir == '-':
            os.environ.pop('ROS_LOG_DIR')
        else:
            os.environ['ROS_LOG_DIR'] = dir

@command
class ComComLog:
    name = 'com'
    num_arg = None
    help = 'print command log'
    
    def invoke(self, api, args):
        api.print_com_log()

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

@command
class ComGit(CmdLinux):
    name = 'git'
    num_arg = None
    help = 'git command'

@command
class ComPip3(CmdLinux):
    name = 'pip3'
    num_arg = None
    help = 'install Python libraries and record them'

@command
class ComApt(CmdLinux):
    name = 'apt'
    num_arg = None
    help = 'install apt modules and record them'
