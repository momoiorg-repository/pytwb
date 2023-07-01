import os
import sys
import glob
import inspect
import ast
import importlib
import subprocess
import traceback
from dataclasses import dataclass, field
import pickle
import shutil

import rclpy
import py_trees_ros

from .py_tree_loader import TreeLoader
from .behavior_loader import BehaviorClassLoader
from . import built_in_command

# package descriptor
@dataclass
class Package:
    ws: str
    name: str
    path: str
    env: dict = field(default_factory=dict)
    param: dict = field(default_factory=dict)

    def __str__(self) -> str:
        return f'name: {self.name}\n' + \
        f'work space: {self.ws}\n' + \
        f'Python class path: {self.path}'
    
    @classmethod
    def get(cls, ws, name):
        path = os.path.join(ws, f'src/{name}/{name}')
        return Package(ws, name, path)

#
## Config: unique to each docker
#
@dataclass
class Config:
    _current: Package = None
    pip3: list = field(default_factory=list)
    apt: list = field(default_factory=list)
    packages: dict = field(default_factory=dict)

    @classmethod
    def load(cls) -> None:
        base = os.path.expanduser('~/.pytwb')
        if not os.path.isdir(base):
            os.mkdir(base)
        dir = os.path.expanduser('~/.pytwb/config')
        if os.path.isfile(dir):
            with open(dir, 'rb') as f:
                data = f.read()
            return pickle.loads(data)
        else:
            config = Config()
            with open(dir, 'wb') as f:
                f.write(pickle.dumps(config))
            return config

    def set(self, key, value):
        setattr(self,key,value)
        self.dump()
    
    @property
    def current(self):
        return self._current
    
    @current.setter
    def current(self, package):
        self._current = package
        self.dump()  

    def add_pip3(self, value):
        if value in self.pip3: return
        self.pip3.append(value)
        self.dump()
    
    def add_apt(self, value):
        if value in self.apt: return
        self.apt.append(value)
        self.dump()

    def add_package(self, package):
        self.packages[(package.ws, package.name)] = package
        self.dump()
    
    def update_current(self, package):
        self.current = package
        self.packages[(package.ws, package.name)] = package
        self.dump()
    
    def dump(self, loc='~/.pytwb/config'):
        dir = os.path.expanduser(loc)
        with open(dir, 'wb') as f:
            f.write(pickle.dumps(self))
    
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
        self.dep_behaviors = set()
        self.dep_trees = set()
        self.package = package
        if not package: return
        work_dir = package.path
        tree_dir = os.path.join(work_dir, 'trees')
        if os.path.isdir(tree_dir):
            self.tree.append(tree_dir)
        behavior_dir = os.path.join(work_dir, 'behavior')
        if os.path.isdir(behavior_dir):
            self.behavior.append(behavior_dir)
        work_dir = os.path.expanduser('~/.pytwb')
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
    def __init__(self, package=None) -> None:
        self.env = Env(package)
        self.current = package
    
    def run(self, src, node_name='behavior_tree', period=1):
        rclpy.init()
        try:
            ros_node = rclpy.node.Node(node_name)
            if len(self.current.param) > 0:
                for k, v in self.current.param.items():
                    ros_node.declare_parameter(k, value=v)
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
    
    def exec(self, trees):
        global config
        fname = os.path.join(config.current.path, 'app.py')
        loader = importlib.machinery.SourceFileLoader('app', fname)
        module = loader.load_module()
        module.app_main(trees)
    
    def shutdown(self):
        rclpy.shutdown()
    
    def search_package(self, ws, name):
        global config
        return config.packages.get((ws, name))

    def register_package(self, ws, name):
        global config
        package = Package.get(ws, name)
        config.add_package(package)
        return package
    
    def set_current_package(self, package):
        global config
        self.env = Env(package)
        sys.path.append(package.path)
        os.chdir(package.ws)
        config.current = package
        self.current = package
        for k, v in package.env.items():
            os.environ[k] = v

    def get_current_package(self):
        return self.current
    
    def delete_package(self, package):
        global config
        config.packages.pop((package.ws, package.name))
        if self.current == package:
            self.current = None 
            config.current = None
    
    # create package
    def create(self, ws, name):
        package = self.search_package(ws, name)
        if package:
            return False # already exists
        
        # run ros2 pkg create
        dir = os.path.join(ws, 'src')
        subprocess.run(f'cd {dir};ros2 pkg create --build-type ament_python {name}',
                       shell=True)
    
        # build package object and register
        package = self.register_package(ws, name)
        
        # create directories
        work_dir = package.path
        os.mkdir(os.path.join(work_dir, 'behavior'))
        os.mkdir(os.path.join(work_dir, 'trees'))

        # create app_main routine
        app = \
f'''
from pytwb.lib_main import run
def app_main(trees):
# insert application specific initialization routine here
    run(trees)
'''
        main_file = os.path.join(work_dir, 'app.py')
        with open(main_file,'w') as f:
            f.write(app)
        
        self.set_current_package(package) # all done
        return True
    
    # register existing package
    def register(self, ws, name):
        package = self.search_package(ws, name)
        if package:
            return False # already exists
        package = self.register_package(ws, name)
        self.set_current_package(package)
        return True
           
    def get_config(self, key):
        global config
        return getattr(config, key)
    
    def set_config(self, key, value):
        global config
        config.set(key, value)
    
    def add_pip3(self, val):
        global config
        config.add_pip3(val)

    def add_apt(self, val):
        global config
        config.add_apt(val)
    
    def add_env(self, key, val):
        self.current.env[key] = val
        config.update_current(self.current)
        os.environ[key] = val
    
    def add_param(self, key, val):
        self.current.param[key] = val
        config.update_current(self.current)
    
    def gen_dockerfile(self):
        global config
        if not self.current:
            print('set current package')
            return
        envs = {}
        params = {}
        for p in config.packages.values():
            envs.update(p.env)
            params.update(p.param) 
        dockerfile = gen_dockerfile(self.current.ws, config.apt, config.pip3, envs)
        ofile = os.path.join(self.current.ws, '_Dockerfile')
        with open(ofile, 'w') as f:
            f.write(dockerfile)
    
    def save_config(self):
        ofile = os.path.join(self.current.ws, '_conifg')
        config.dump(ofile)
    
    def get_behaviors(self):
        BehaviorClassLoader(self.env)
        behaviors = []
        for m in self.env.behavior_module_table.values():
            behaviors += m.behaviors
        return behaviors
    
    def get_trees(self):
        return self.env.get_tree_list()
    
    def get_base(self):
        return os.path.expanduser('~/.pytwb')
    
    def depend(self, target):
        root = self.env.get_tree_file(target)
        self.env.dep_behaviors = set()
        self.env.dep_trees = set()
        tloader = TreeLoader(self.env)
        tloader.scan_tree(target)
        behaviors = {}
        for b in self.env.dep_behaviors:
            behaviors[b.name] = b
        trees = {}
        for t in self.env.dep_trees:
            trees[t.name] = t
        return root, behaviors, trees
    
    def gen_main(self, tree_name):
        ws = self.current.ws
        name = self.current.name
        code = \
f'''
from pytwb.lib_main import initialize
import app

if __name__ == "__main__":
    initialize("{ws}", "{name}")
    app.app_main("{tree_name}")
'''
        main_file = os.path.join(self.current.path, 'main.py')
        with open(main_file,'w') as f:
            f.write(code)
    
    def clear(self):
        global config
        ws = self.current.ws
        conf_file = os.path.expanduser('~/.pytwb/config')
        os.remove(conf_file)
        config = Config.load()
        self.current = None
        self.env = Env(None)
        src_dir = os.path.join(ws, 'src')
        shutil.rmtree(src_dir)
        os.mkdir(src_dir)
        self.env = Env(None)

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
        if name == 'help' or name == '?':
            for c in self.commands.values():
                print(f'{c.name}: {c.help}')
            return
        if name.startswith('!'):
            if name != '!':
                com = [name[1:]] + args
            else:
                com = args
            line = ' '.join(com)
            subprocess.run(line, shell=True)
            return
        c_obj = self.commands.get(name)
        if c_obj: # execute built in command
            an = c_obj.num_arg
            if an and an > len(args):
                print('arg count: ' + c_obj.help)
                return
            c_obj.invoke(bt_factory_api, args)
        else: # import from base dir
            fn = bt_factory_api.env.get_path(name)
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
    base = os.path.expanduser('~/.pytwb')
    sys.path.append(base)
    package = None
    if ws:
        package = config.packages.get((ws, name))
        if not package:
            package = Package.get(ws, name)
            config.add_package(package)
        sys.path.append(package.path)
        config.current = package
    else:
        package = config.current
    bt_factory_api = BTFactoryAPI(package)
    if package:
        bt_factory_api.set_current_package(package)
        print(f'current package:\n{package}')
    else:
        print('no current package')

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
    if len(sys.argv) > 1:
        arg0 = sys.argv[1]
        if arg0 == '-c':
            ws = os.path.abspath(sys.argv[2])
            create_package(ws, sys.argv[3])
            return
        else:
            if len(sys.argv) == 2:
                ws = os.getcwd()
                name = sys.argv[1]
            else:
                ws = os.path.abspath(sys.argv[1])
                name = sys.argv[2]
            initialize(ws, name)
    else:
        initialize()
    do_command()

apt_init = [
        'vim', 'xterm', 'less', 'git', 'python3-pip', 
        'ros-humble-navigation2', 'ros-humble-py-trees',
        'ros-humble-py-trees-ros'
    ]

def gen_dockerfile(ws, apts, pip3s, envs):
    ws_name = ws.split('/')[-1]
    apt_list = 'RUN apt-get update && apt-get install -y --no-install-recommends '
    apt_list += ' '.join(apt_init + apts)
    
    if len(pip3s) > 0:
        pip3_list = 'RUN pip3 install '
        pip3_list += ' '.join(pip3s)
    else:
        pip3_list = ''

    env_list = ''
    for key, value in envs.items():
        env_list += f'ENV {key}={value}\n'

    return \
f'''
FROM ros:humble
SHELL ["/bin/bash", "-c"]

{apt_list}

{pip3_list}

{env_list}

WORKDIR /usr/local/lib
RUN git clone https://github.com/momoiorg-repository/pytwb.git
WORKDIR /usr/local/lib/pytwb
RUN source /opt/ros/humble/setup.bash && pip3 install -e .

WORKDIR /root
COPY ./src {ws_name}
COPY _.pytwb .pytwb
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
'''

# global variable
config = Config.load()
