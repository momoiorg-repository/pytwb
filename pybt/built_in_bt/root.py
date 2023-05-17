import rclpy
import py_trees
import py_trees_ros
from py_trees.common import OneShotPolicy

class root(py_trees.decorators.OneShot):
    def __init__(self, child):
        super().__init__("OneShot", child, OneShotPolicy.ON_COMPLETION)
    

    
    def update(self):
        if self.status == py_trees.common.Status.SUCCESS:
            print('mission complete')
#            self.tree.node.destroy_node()
            raise SystemExit
        elif self.status == py_trees.common.Status.FAILURE:
            print('mission failure')
#            self.tree.node.destroy_node()
            raise SystemExit
        return super().update()

class BehaviorTree(py_trees.behaviour.Behaviour):
    def __init__(self, ID):
        super().__init__(ID)
        print(f'register tree: {ID}')
    def update(self):
        for c in self.children:
            c.update()

class KeepRunningUntilFailure(py_trees.behaviour.Behaviour):
    def __init__(self, ID):
        super().__init__('keep_running_until_failure')
    def update(self):
        print('update')
