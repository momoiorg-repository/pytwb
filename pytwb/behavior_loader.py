import os
import importlib
import inspect
import ast
from enum import Enum

import py_trees

from . import built_in_bt

#
## important variables
#
# BehaviorNodeDescriptor: b_desc  allocated to each behavior definition
#                         definition may come from:
#                               "behavior" directory, py_trees, other tree file
#                         table of BehaviorNodeDescriptor is kept by
#                               "behavior_talbe" of BehaviorClassLoader object
#                               
# ModuleDescriptor: m_desc  allocated to each ".py" module
#                           which may contain multiple behavior class definitions
#                           m_desc is identified by module file name
#                           table of ModuleDescriptor is kept by Env object
#
# TreeDescriptor: t_desc allocated to eahc ".xml" file under "trees" dir
#                           this may contain multiple BehaviorNodeDescriptor
#                           and is identified by file name
#                           table of TreeDescriptor is kept by Env object

built_in = {}

class BehaviorNodeDescriptor:
    def __init__(self, name) -> None:
        self.name = name

class BehaviorClassDescriptor(BehaviorNodeDescriptor):
    def __init__(self, name, cls) -> None:
        super().__init__(name)
        self.bt_class = cls

def initialize_built_in():
    global built_in
    for b in built_in_bt.builtin_behaviors:
        name = b.name
        b_desc = BehaviorClassDescriptor(name, b.cls)
        b_desc.type = b.type
        built_in[name] = b_desc
        cons_args = list(b.cls.__init__.__code__.co_varnames)
        cons_args.remove('self')
        b_desc.cons_args = cons_args
        b_desc.default_args = b.arg

# ModuleDescriptor is allocated to each Python module
class ModuleDescriptor:
    def __init__(self, name, file_name, mtime) -> None:
        self.name = name
        self.file_name = file_name
        self.mtime = mtime
        self.behaviors = []

class BehaviorClassLoader:
    def __init__(self, env) -> None:
        self.env = env
        # self.behavior_table keeps BehaviorDescripor
        self.behavior_table = {}
        # update behavior module table
        new_behavior_module_table = {} # source file(=module) list of behaviors
        for f in env.get_behavior_module_list():
            mtime = os.stat(f).st_mtime
            mname = f.split('/')[-1][:-3]
            m_desc = env.behavior_module_table.get(f)
            if (not m_desc) or (m_desc.mtime < mtime): # need recompilation
                m_desc = self.load_behavior_module(f)
            else:
                for b in m_desc.behaviors:
                    self.behavior_table[b.name] = b
            new_behavior_module_table[f] = m_desc
        env.behavior_module_table = new_behavior_module_table
            # module table is kept in Env object
        self.bb = py_trees.blackboard.Blackboard()

    def load_behavior_module(self, file_name) -> ModuleDescriptor:
        m_desc = self.env.behavior_module_table.get(file_name)
        modtime = os.stat(file_name).st_mtime
        if m_desc and modtime <= m_desc.mtime:
            for b in m_desc.behaviors:
                self.behavior_table[b.name] = b
            return m_desc
        m_desc = self.read_module_file(file_name)
        self.env.behavior_module_table[m_desc.name] = m_desc
        return m_desc
    
    # read in module directly from source file
    def read_module_file(self, file_name) -> ModuleDescriptor:
        name = file_name.split('/')[-1][:-3] 
        loader = importlib.machinery.SourceFileLoader( name, file_name )
        spec = importlib.util.spec_from_loader( name, loader )
        mod = importlib.util.module_from_spec( spec )
        loader.exec_module( mod )
        m_desc = ModuleDescriptor(mod.__name__, file_name, os.stat(file_name).st_mtime)
        m_desc.body = mod
        self.extract_behavior(m_desc)
        return m_desc

    # extract behavior definitions from module
    def extract_behavior(self, m_desc) -> None:
        module = m_desc.body
        behaviors = []
        source = inspect.getsource(module)
        a = ast.parse(source)
        for elm in a.body:
            if not isinstance(elm, ast.ClassDef): continue
            if not hasattr(elm, 'decorator_list'): continue
            dec = elm.decorator_list
            if len(dec) <= 0: continue
            for d in dec:
                if d.id != 'behavior': continue
                b_name = elm.name
                cls = getattr(module, b_name)
                b_desc = BehaviorClassDescriptor(b_name, cls)
                cons_args = list(cls.__init__.__code__.co_varnames)
                cons_args.remove('self')
                b_desc.cons_args = cons_args # set constructor arguments
                behaviors.append(b_desc)
                self.behavior_table[b_name] = b_desc
        m_desc.behaviors = behaviors # keep behavior descriptor in module descriptor
        
class TreeBehaviorDescriptor(BehaviorNodeDescriptor):
    def __init__(self, name, tree) -> None:
        super().__init__(name)
        self.behavior_tree = tree

class BehaviorTable:
    def __init__(self, env) -> None:
        self.env = env
        self.tree_behavior = {}
        self.class_loader = BehaviorClassLoader(env)

    def append_behavior_from_tree(self, b):
        desc = TreeBehaviorDescriptor(b.name, b)
        self.tree_behavior[b.name] = desc

    def get_behavior(self, name) -> BehaviorNodeDescriptor:
        # try external behavior class
        ret = self.class_loader.behavior_table.get(name)
        if ret: return ret
        # try xml file originated behavior definition
        ret = self.tree_behavior.get(name)
        if ret: return ret
        # try built in behavior class
        return built_in.get(name)

# initialization of this module
initialize_built_in()

    
