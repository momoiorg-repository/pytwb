# initialzation sequence
from . import common
from . import behavior_loader
from . import py_tree_loader

config = None
def set_config(value):
    global config
    config = value
