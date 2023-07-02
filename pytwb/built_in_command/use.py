import os
import subprocess

from ..common import command
from .base import Com

def copy_file(source, destination):
    with open(source, 'rb') as inf:
        with open(destination, 'wb') as ouf:
            ouf.write(inf.read())

def do_fetch(api, tree):
    root, behaviors, trees = api.depend(tree)
    path = api.get_current_package().path
    b_files = set()
    for b in behaviors.values():
        f = b.module.__file__
        if f.startswith(path): continue
        b_files.add(f)
    b_dir = os.path.join(path, 'behavior')
    for bf_src in b_files:
        short = bf_src.split('/')[-1]
        bf_dest = os.path.join(b_dir, short)
        copy_file(bf_src, bf_dest)
    t_files = set()
    if not root.startswith(path): t_files.add(root)
    for t in trees.values():
        f = t.t_desc.file_name
        if f.startswith(path): continue
        t_files.add(f)
    t_dir = os.path.join(path, 'trees')
    for tf_src in t_files:
        short = tf_src.split('/')[-1]
        tf_dest = os.path.join(t_dir, short)
        copy_file(tf_src, tf_dest)


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
                    if fname.startswith('_'): continue
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
class ComFetch:
    name = 'fetch'
    num_arg = 1
    help = 'fetch package_name :: fetch code from base to current ws'
    
    def invoke(self, api, args):
        do_fetch(api, args[0])

@command
class ComShowDepend:
    name = 'show'
    num_arg = 1
    help = 'show package_name :: show module files of current package'
    
    def invoke(self, api, args):
        root, behaviors, trees = api.depend(args[0])
        print(f'root:\n\t{root}')
        print('behaviors:')
        for b in behaviors.values():
            f = b.module.__file__
            print(f'\t{b.name}: {f}')
        print('depending trees:')
        for t in trees.values():
            f = t.t_desc.file_name
            print(f'\t{t.name}: {f}')

@command
class ComDockerfile:
    name = 'dockerfile'
    num_arg = 0
    help = 'dockerfile :: generate dockerfile template to _Dockerfile'
    
    def invoke(self, api, args):
        api.gen_dockerfile()

@command
class ComFinalize:
    name = 'finalize'
    num_arg = 1
    help = 'finalize :: generate files to complete ws image'
    
    def invoke(self, api, args):
        do_fetch(api, args[0])
        api.gen_dockerfile()
        api.save_config()
        api.gen_main(args[0])

@command
class ComReset:
    name = 'reset'
    num_arg = 0
    help = 'remove ws contents'
    
    def invoke(self, api, args):
        print('All contents of the current ws will be removed.')
        ans = input('Are you shure?[Y/n]').upper()
        if ans != 'Y':
            print('not removed')
            return
        api.clear()
