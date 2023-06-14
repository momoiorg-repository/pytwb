import os
import sys
import glob
import inspect
import ast
import importlib
import subprocess
import traceback
import yaml
from dataclasses import dataclass

import rclpy
import py_trees_ros

from .py_tree_loader import TreeLoader
from . import built_in_command

# global variable
config = Config()

#
## Config: unique to each docker
#
class Config:
    def __init__(self) -> None:
        dir = os.path.expanduser('~/.pytwb')
        self.dir = dir
        if os.path.isfile(dir):
            with open(dir) as f:
                config = yaml.safe_load(f)
        else:
            config = {
                "current" : None,
                "pip3" : [],
                "apt" : [],
                "packages": {}
            }
            with open(dir, 'w') as f:
                yaml.dump(config, f)
        self.config = config
    
    def get(self, key):
        return self.config.get(key)

    def set(self, key, value):
        if key == 'current':
            self.config['current'] = value
        if key == 'pip3' or key == 'apt':
            if key in self.config[key]: return
            self.config[key].append(value)
        with open(self.dir, 'w') as f:
            yaml.dump(self.config, f)

    def set_current(self, package):
        self.set('current', package)  

    def add_package(self, package):
        self.packages[(package.ws, package.name)] = package
    
    def get_package(self, ws, name):
        return self.packages.get((ws, name))

@dataclass
class Package:
    ws: str
    name: str
    path: str

    def __str__(self) -> str:
        return f'name: {self.name}\n' + \
        f'work space: {self.ws}\n' + \
        f'Python class path: {self.base_dir}'
    
    @classmethod
    def get(cls, ws, name):
        path = os.path.join(ws, f'src/{name}/{name}')
        return Package(ws, name, path)

    
#
## Env: allocated to each ROS package
#
class Env:
    def __init__(self, package) -> None:
        self.tree_table = {}
        self.behavior_module_table = {}
        self.tree = []
        self.behavior = []
        self.path = []
        self.package = package
        if not package: return
        work_dir = package.path
        tree_dir = os.path.join(work_dir, 'trees')
        if os.path.isdir(tree_dir):
            self.tree.append(tree_dir)
        behavior_dir = os.path.join(work_dir, 'behavior')
        if os.path.isdir(behavior_dir):
            self.behavior.append(behavior_dir)
        path_dir = os.path.join(work_dir, 'command')
        if os.path.isdir(path_dir):
            self.path.append(path_dir)
  
    def get_path(self, name):
        name += '.py'
        for p in self.path:
            fn = os.path.join(p, name)
            if os.path.isfile(fn): return fn
        else:
            return None
        
    def get_tree_list(self):
        ret = []
        for t in self.tree:
            ret.extend(glob.glob(os.path.join(t, '*.xml')))
        return ret
    
    def get_tree_file(self, name):
        if name.startswith('/'): # absolute path
            if os.path.isfile(name): return name
            else: return None
        if not name.endswith('.xml'):
            name += '.xml'
        for t in self.tree:
            fname = os.path.join(t, name)
            if os.path.isfile(fname): return fname
        else: return None
            
    def get_behavior_module_list(self):
        ret = []
        for t in self.behavior:
            for f in glob.glob(os.path.join(t, '*.py')):
                if f.startswith('_'): continue
                ret.append(f)
        return ret

class BTFactoryAPI:
    def __init__(self, work_dir=None) -> None:
        self.env = Env(work_dir)
    
    def run(self, src, node_name='behavior_tree', period=1):
        rclpy.init()
        try:
            ros_node = rclpy.node.Node(node_name)
            tloader = TreeLoader(self.env)
            tree = tloader.load_tree(src, ros_node)
            root = py_trees_ros.trees.BehaviourTree(tree, unicode_tree_debug=False)
            root.setup(timeout=15.0, node=ros_node)
            self.root = root
            root.tick_tock(period_ms=period*1000.0)
            try:
                rclpy.spin(ros_node)
            except SystemExit:
                print('done')
        except Exception as e:
            rclpy.shutdown()
            raise e
        rclpy.shutdown()
    
    def shutdown(self):
        rclpy.shutdown()
    
    def set_current_package(self, ws, name):
        global config
        package = Package.get(ws, name)
        sys.path.append(package.path)
        self.env = Env(package)
        config.add_package(package)
        config.set_current(package)
        print(f'current package:\n{package}')
        return package
    
    # create package
    def create(self, ws, name):
        # run ros2 pkg create
        dir = os.path.join(ws, 'src')
        subprocess.run(f'cd {dir};ros2 pkg create --build-type ament_python {name}',
                       shell=True)
        
        # create directories
        package = self.set_current_package(ws, name)
        work_dir = package.path
        os.mkdir(os.path.join(work_dir, 'behavior'))
        os.mkdir(os.path.join(work_dir, 'trees'))

        # create main routines
        dbg_main = \
            'from pytwb.lib_main import initialize, do_command\n' + \
            f'initialize({ws}, {name})\n' + \
            'do_command()\n'
        dbg_file = os.path.join(work_dir, 'dbg_main.py')
        with open(dbg_file,'w') as f:
            f.write(dbg_main)

        main = \
            'from pytwb.lib_main import initialize, run\n' + \
            f'initialize({ws}, {name})\n' + \
            '#run(XML file name)\n'
        main_file = os.path.join(work_dir, 'main.py')
        with open(main_file,'w') as f:
            f.write(main)  
    
    def get_config(self, key):
        global config
        return config.get(key)
    
    def set_config(self, key, value):
        global config
        config.set(key, value)

class CommandInterpreter:
    def __init__(self) -> None:
        self.commands = {}
        for name, m in built_in_command.__dict__.items():
            if name.startswith('_'): continue
            self.get_builtin_command(m)
    
    def get_builtin_command(self, module):
        source = inspect.getsource(module)
        a = ast.parse(source)
        for elm in a.body:
            if not isinstance(elm, ast.ClassDef): continue
            if not hasattr(elm, 'decorator_list'): continue
            dec = elm.decorator_list
            if len(dec) <= 0: continue
            for d in dec:
                if d.id != 'command': continue
                b_name = elm.name
                c_obj = getattr(module, b_name)()
                self.commands[c_obj.name] = c_obj

    def exec_command(self, name, args):
        global bt_factory_api
        if name == help or name == '?':
            for c in self.commands.values():
                print(f'{c.name}: {c.help}')
            return
        if name.startswith('!'):
            if name != '!':
                com = [name[1:]] + args
            else:
                com = args
            subprocess.run(com)
            return
        c_obj = self.commands.get(name)
        if c_obj: # execute built in command
            an = c_obj.num_arg
            if an and an > len(args):
                print('arg count: ' + c_obj.help)
                return
            c_obj.invoke(bt_factory_api, args)
        else: # import from base dir
            fn = self.env.get_path(name)
            if fn:
                loader = importlib.machinery.SourceFileLoader( name, fn )
                spec = importlib.util.spec_from_loader( name, loader )
                mod = importlib.util.module_from_spec( spec )
                loader.exec_module( mod )
                cmd_func = getattr(mod, 'main')
                if cmd_func: 
                    cmd_func(bt_factory_api, *args)
                    return
            print('command not found')

# entry point for application program
def initialize(ws=None, name=None):
    global bt_factory_api, config
    package = None
    if ws:
        package = config.get_package((ws, name))
        if not package:
            package = Package.get(ws, name)
            config.set_package(package)
        sys.path.append(package.path)
        config.set_current(package)
    else:
        package = config.get_current()
    bt_factory_api = BTFactoryAPI(package)

def create_package(ws, name):
    global bt_factory_api
    bt_factory_api = BTFactoryAPI()
    bt_factory_api.create(ws, name)

def run(script, work_dir=None):
    global bt_factory_api
    if work_dir:
        bt_factory_api.change_directory(work_dir)
    bt_factory_api.run(script)

def do_command():
    command_interpreter = CommandInterpreter()

    while True:
        line = input('> ')
        elms = line.split()
        if len(elms) <= 0: continue
        command = elms[0]
        args = elms[1:]
        if command == 'exit': break
        try:
            command_interpreter.exec_command(command, args)
        except Exception as e:
            print(f'error: {e}')
            traceback.print_exc()

# entry point for command line interface
def cli():
    create = None
    work_dir = None
    if len(sys.argv) > 1:
        arg0 = sys.argv[1]
        print(arg0)
        if arg0 == '-c':
            create = sys.argv[2]
        else:
            work_dir = sys.argv[1]
    if not create:
        initialize(work_dir)
        do_command()
    else:
        create_package(create)
