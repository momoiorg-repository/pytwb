# pytwb: A workbench for creating ROS applications based on py_trees and py_trees_ros frameworks (beta).
pytwb is assumed to be used with VSCode under docker environment. It generates and runs ROS packages on a command line basis. Although based on py_trees, it is possible to define behavior trees using XML, because pytwb is equipped with XML parser.  With writing XML and Python codes by VsCode, it becomes possible to collectively execute them interactively as a ROS application.

## Getting Started
### Prerequirement
pytwb is a normal Python application and can be installed locally, but it is recommended to use docker to build the ROS application and create an execution environment by using pytwb. Development is done by first generating a docker and attaching VSCode to that docker. Therefore, the docker environment and VSCode must be available in the Ubuntu22 environment. It's standard to do it in WSL. VSCode must have a plugin installed for docker.

### Installation
First, do a docker build in your local environment. Get the git repository as a first step.
```
git clone https://github.com/momoiorg-repository/pytwb.git
```
Then run docker build.
```
cd pytwb  
docker image build -t pytwb:latest .
```
creates a docker image.

### Create a ROS package using pytwb
First, in your local environment, create a workspace as usual.
```
mkdir -p <workspace name>/src
```
For the rest of the work, start docker instead of the local environment and proceed within it.
```
docker run -–name <docker name> -v `pwd`/<workspace name>:/root/<workspace name>:rw -it pytwb
```
Then from VSCode, attach to this docker. Set the working directory to  
/root/\<workspace name\>/src  
Next, launch pytwb to create a ROS package. Open Terminal of VSCode, and
```
pytwb -c <package name>
```
will do. The package is now created.

### Coding and running
Use pytwb to run your code.
First, create the behavior code in Python in  
 \<package name\>/\<package name\>/behavior  
under the current directory, and create the XML code of the behavior tree in  
\<package name\>/\<package name\>/trees.

　Next, when you execute  
\<package name\>/\<package name\>/dbg_main.py  
from VSCode, the prompt '>' will be displayed. Therefore,

\> run \<behavior tree name\>

will do. \<behavior tree name\> is the file name of the XML behavior tree you created under the 'trees' directory, or it without the '.XML'.
　The behavior tree you created will now start running as a ROS application. After that, please debug using VSCode in the same way as usual.
Finally, the package is completed by building a ROS package with  
\<package name\>/\<package name\>/main.py  
as the main.

## Document
- [overview](doc/overview.md)
## Sample code
- [pytwb_demo](https://github.com/momoiorg-repository/pytwb_demo)
