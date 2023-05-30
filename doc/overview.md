# pytwb documentation

pytwb is a tool for supporting ROS behavior tree implementation using Python, and operates using py_trees and py_trees_ros. It has an XML parser and can execute a behavior tree described in XML. There are two ways to use it: from the API and from the command line.

## Usage from the command line
When used from the command line, pytwb is intended to be used to build a ROS workspace from scratch or to add packages to an existing workspace. It is also assumed to use docker and VSCode as the underlying mechanism. Install according to the [Readme](../README.md), create a workspace, switch to docker + VSCode execution, create a package, start the command line, and develop and run the application.

The current directory of VSCode is assumed to be set to the workspace location.
Create a package with pytwb -c <package name>. Inside the pytwb, the ros2 pkg command is executed internally, and in addition to generated standard ROS2 package directories, under the ./src/\<package name\>/\<package name\> directory, the behavior, trees directories, and dbg_main.py , main.py files are generated.

In pytwb, the base directory is set as a reference during work, in addition to the current directory (base directory). In this example, the initialization code will set it to ./src/\<package name\>/\<package name\>. Names of referenced files below are relative to this base directory.  Also, at runtime, the base directory location is added to Python's classpath.

In this state, create behavior code written in Python under "behavior" directory. The file name body is arbitrary and the extension should be ".py". Also, create behavior tree code written in XML under "trees" directory. The file extension of the behavior tree must be ".xml", but the file name body is arbitrary. The local file name without .xml is referred to as the name when specifying the behavior tree to be executed, so it is desirable to use a name that is easy to refer to.
Under the base directory, you can also create other directories to place the necessary files.

When the entire code is written, start debugging. Run dbg_main.py from VSCode to get the whole thing working. At this time, if it is executed in debug mode, it is possible to execute the entire application in debug mode.

The command prompt is displayed by executing dbg_main.py. The command you enter from the command prompt is:  
\> run <behavior tree name>  
\<behavior tree name\> is the file name body of XML file describing the behavior tree, omitting ".xml", as described above. This initiates the interpretive execution of the behavior tree. If you need to invoke some initialization codes before that, do it by adding them to dbg_main.py.

After completing one execution of "run", even if dbg_main.py is still running, the code under behavior and trees directories can be changed at any time for debugging. The next time you enter the run command, it will automatically reload the Python module and parse the XML file. This shortens the turnaround time during debugging.

In the future, we plan to make it possible to install external Python packages and applications using apt and pip as if they are pytwb commands. By recording installed packages and application names through these commands, build files will be able to be generated semi-automatically.

Pytwb is a tool that supports the development of ROS behavior tree applications using Python with such functions.

## Description of behavior tree in XML
The behavior tree description is similar to the description in ROS standard BehaviorTree.CPP. However, the behavior it uses is the one supplied by py_trees. Actual code takes the form of the followings:

```
<root>
    <BehaviorTree id=<behavior tree id>>
--- body description
    </BehaviorTree>
</root>
```

Normal behaviors inside the "body description" are:  
\<\<behavior type\> name=\<behavior name\> args>  
Specifying name is essential. 'args' corresponds to the arguments of the constructor in the behavior class description in Python, each of them takes the form of:  
arg_name = arg_value  
For arg_value, you can specify a normal string or a string surrounded by {} or []. A character string enclosed in { } is the blackboard tag name, and the value retreived from the blackboard is the actual argument. A string enclosed in [ ] is evaluated as a Python expression. Therefore, for example, [True] is the Python boolean value True, and [1] is the Python integer value 1 as the actual arguments.

The behaviors that can be used by default are currently the following: 
- Composites  
     Selector, Sequence, Parallel
- Decorators  
     Condition, Count, EternalGuard, Inverter, OneShot, Repeat, Retry, StatusToBloackboard, Timeout
     FailureIsRunning, FailureIsSuccess, RunningIsFailure, RunningIsSuccess, SuccessIsFailure, SuccessIsRunning

For details, refer to the py_trees documentation (https://py-trees.readthedocs.io/en/devel/decorators.html).

In addition, behavior tree files under trees directory are all parsed at the same time, forming a kind of database. This makes it possible to hierarchically call and use descriptions of other behavior trees as components.

## Writing behavior in Python
A new behavior written in Python should be placed under the "behavior" directory. Also, the type name of behavior uses the class name as it is. Furthermore, it is necessary to add an annotation that indicates that it is a description of behavior. Therefore, the description is as follows.

```
import py_trees
from pytwb.common import behavior

@behavior
class Commander(py_trees.behaviour.Behaviour):
    pass
```

The description method of the class body is the same as the policy required by py_trees and py_trees_ros so far.
