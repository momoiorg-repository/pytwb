import py_trees

class Parallel(py_trees.composites.Parallel):
    def __init__(self, name: str, policy: str, children):
        if policy=='SuccessOnAll': policy_obj = py_trees.common.ParallelPolicy.SuccessOnAll()
        elif policy=='SuccessOnOne': policy_obj = py_trees.common.ParallelPolicy.SuccessOnOne()
        elif policy=='SuccessOnSelected': policy_obj = py_trees.common.ParallelPolicy.SuccessOnSelected()
        else: raise Exception('Parallel policy error')
        super().__init__(name, policy_obj)
        for c in children:
            self.add_child(c)
    
    def tick(self):
        return super().tick()