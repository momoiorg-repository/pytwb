import os
import glob
import inspect
import ast
import importlib
import subprocess
import traceback

import rclpy
import py_trees_ros

from .py_tree_loader import TreeLoader
from . import built_in_command

class Env:
    def __init__(self, work_dir=None) -> None:
        if work_dir:
            self.work_dir = work_dir
        else:
            self.work_dir = os.getcwd()
        self.tree = []
        tree_dir = os.path.join(self.work_dir, 'trees')
        if os.path.isdir(tree_dir):
            self.tree.append(tree_dir)
        self.behavior = []
        behavior_dir = os.path.join(self.work_dir, 'behavior')
        if os.path.isdir(behavior_dir):
            self.behavior.append(behavior_dir)
        self.path = []
        path_dir = os.path.join(self.work_dir, 'command')
        if os.path.isdir(path_dir):
            self.path.append(path_dir)
        self.tree_table = {}
        self.behavior_module_table = {}
  
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
#            ret.extend(glob.glob(os.path.join(t, '[^_]*.py')))
            for f in glob.glob(os.path.join(t, '*.py')):
                if f.startswith('_'): continue
                ret.append(f)
        return ret

class BTFactoryAPI:
    def __init__(self, work_dir=None) -> None:
        if not work_dir:
            work_dir = os.getcwd()
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
    
    def change_directory(self, work_dir):
        self.env = Env(work_dir)
    
    def create(self, name):
        subprocess.run(f'ros2 pkg create --build-type ament_python {name}',
                       shell=True)
        work_dir = os.path.join(os.getcwd(), f'{name}/{name}')
        os.mkdir(os.path.join(work_dir, 'behavior'))
        os.mkdir(os.path.join(work_dir, 'trees'))

        dbg_main = '''
from pybt.lib_main import initialize, do_command
initialize()
do_command()
'''
        dbg_file = os.path.join(work_dir, 'dbg_main.py')
        with open(dbg_file,'w') as f:
            f.write(dbg_main)

        main = '''
from pybt.lib_main import initialize, run
initialize()
#run(XML file name)
'''
        main_file = os.path.join(work_dir, 'main.py')
        with open(main_file,'w') as f:
            f.write(main)

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
        c_obj = self.commands.get(name)
        if c_obj: # execute built in command
            an = c_obj.num_arg
            if an and an > len(args):
                print('arg count: ' + c_obj.help)
                return
            c_obj.invoke(bt_factory_api, args)
        else: # import from work dir
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

def initialize(work_dir=None):
    global bt_factory_api
    bt_factory_api = BTFactoryAPI(work_dir)

def create_project(name):
    global bt_factory_api
    bt_factory_api = BTFactoryAPI()
    bt_factory_api.create(name)

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

