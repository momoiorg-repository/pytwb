import sys
import os

from .lib_main import initialize, do_command, create_project

def run():
    create = None
    if len(sys.argv) > 1:
        arg0 = sys.argv[1]
        print(arg0)
        if arg0 == '-c':
            create = sys.argv[2]
        else:
            work_dir = sys.argv[1]
    else:
        work_dir = os.getcwd()
    if not create:
        initialize(work_dir)
        do_command()
    else:
        create_project(create)
