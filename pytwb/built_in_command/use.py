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
    name = 'use'
    num_arg = 1
    help = 'introduce an external work space to the base'

    def invoke(self, api, args):
        ws = os.path.abspath(args[0])
        dest = api.get_base()
        packages = os.path.join(ws, 'src')
        for p in os.listdir(packages):
            source = os.path.join(packages, p)
            source = os.path.join(source, p)
            if not os.path.isdir(source): continue
            for dir in ('behavior', 'trees', 'lib'):
                s_dir = os.path.join(source, dir)
                if not os.path.isdir(s_dir): continue
                d_dir = os.path.join(dest, dir)
                os.makedirs(d_dir, exist_ok=True)
                for fname in os.listdir(s_dir):
                    s_file = os.path.join(s_dir, fname)
                    if os.path.isdir(s_file): continue
                    d_file = os.path.join(d_dir, fname)
                    if not os.path.isfile(d_file):
                        copy_file(s_file, d_file)
                    else:                    
                        print(f'file {dir}/{fname} already exists')
                        ans = input('overwrite, skip or rename [o/s/r]').upper()
                        if ans == 'O':
                            copy_file(s_file, d_file)
                        elif ans == 'S':
                            continue
                        else:
                            fname = input('input new name')
                            d_file = os.path.join(d_dir, fname)
                            copy_file(source, d_file)
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