import os
import subprocess

from ..common import command
from .base import Com

def copy_file(source, destination):
    with open(source, 'rb') as inf:
        with open(destination, 'wb') as ouf:
            ouf.write(inf.read())

@command
class ComUse(Com):
    name = 'run'
    num_arg = 1
    help = 'introduce an external work space to the base'

    def invoke(self, api, args):
        ws = os.path.abspath(args[0])
        dest = api.get_base()
        packages = os.path.join(ws, 'src')
        for p in os.listdir(packages):
            source = os.path.join(ws, p)
            source = os.path.join(source, p)
            if not os.path.isdir(source): continue
            for dir in ('behavior', 'trees', 'lib'):
                s_dir = os.path.join(source, dir)
                if not os.path.isdir(s_dir): continue
                d_dir = os.path.join(dest, dir)
                for fname in os.listdir(s_dir):
                    source = os.path.join(s_dir, fname)
                    destination = os.path.join(d_dir, fname)
                    if not os.path.isfile(destination):
                            copy_file(source, destination)
                    else:                    
                        print(f'file {dir}/{source} already exists')
                        ans = input('overwrite, skip or rename [o/s/r]').upper()
                        if ans == 'O':
                            copy_file(source, destination)
                        elif ans == 'S':
                            continue
                        else:
                            fname = input('input new name')
                            destination = os.path.join(d_dir, fname)
                            copy_file(source, destination)

@command
class ComSave(Com):
    name = 'save'
    num_arg = None
    help = 'copy current work space to base for future reuse'

    def invoke(self, api, args):
        api.run(args[0])

@command
class ComImport(Com):
    name = 'import'
    num_arg = 1
    help = 'copy needed components from base to current work space'

    def invoke(self, api, args):
        api.run(args[0])