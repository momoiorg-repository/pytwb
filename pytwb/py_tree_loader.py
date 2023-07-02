import os
import xml.etree.ElementTree as ET

from .behavior_loader import BehaviorTable, TreeBehaviorDescriptor

class ParseNode:
    def __init__(self, element, desc) -> None:
        self.element = element
        self.desc = desc
        self.type_name = element.tag
        name = element.attrib.get('name')
        if name: self.name = name
        else: self.name = element.tag
        self.children = []

    def generate(self, ros_node, bt_loader):
        bnode_name = self.type_name
        bnode = bt_loader.get_behavior(bnode_name)
        if not bnode:
            print(f'{bnode_name}: not found')
            raise Exception('behavior not found')
        if isinstance(bnode, TreeBehaviorDescriptor):
            # this node is created by another behavior tree
            loader = TreeLoader(bt_loader.env)
            return loader.load_tree(bnode.name,ros_node)
        # regular behavior class
        bnode_cls = bnode.bt_class
        child_nodes = []
        for c in self.children:
            child_nodes.append(c.generate(ros_node, bt_loader))
        rarg = {}
        for an in bnode.cons_args: # constructor args
            if an == 'children': rarg['children'] = child_nodes
            elif an == 'child': rarg['child'] = child_nodes[0]
            elif an == 'node': rarg['node'] = ros_node
            else:
                value = self.element.attrib.get(an)
                if not value and hasattr(bnode,'default_args'):
                    value = bnode.default_args.get(an)
                if value: rarg[an] = self.eval_arg(value, bt_loader)
        bnode_body = bnode_cls(**rarg)
#        if hasattr(bnode_body, 'add_children'):
#            bnode_body.add_children(child_nodes)
        return bnode_body

    def eval_arg(self, expression, bt_loader):
        if expression.startswith('{') and expression.endswith('}'):
            return bt_loader.bb.get(expression[1:-1])
        elif expression.startswith('[') and expression.endswith(']'):
            return eval(expression[1:-1])
        return expression
    
    def scan(self, bt_loader):
        bnode_name = self.type_name
        bnode = bt_loader.get_behavior(bnode_name)
        if not bnode:
            print(f'{bnode_name}: not found')
            raise Exception('behavior not found')
        if isinstance(bnode, TreeBehaviorDescriptor):
            # this node is created by another behavior tree
            loader = TreeLoader(bt_loader.env)
            return loader.scan_tree(bnode.name)
        # regular behavior class
        for c in self.children:
            c.scan(bt_loader)

class RootParseNode(ParseNode):
    def __init__(self, element, desc) -> None:
        super().__init__(element, desc)
        desc.root = self

    def generate(self, ros_node, bt_loader):
        bnode = bt_loader.get_behavior('root')
        bnode_cls = bnode.bt_class
        child_nodes = []
        for c in self.children:
            child_nodes.append(c.generate(ros_node, bt_loader))
        bnode_body = bnode_cls(child_nodes[0])
#        bnode_body.add_children(child_nodes)
        return bnode_body
    
    def scan(self, bt_loader):
        for c in self.children:
            c.scan(bt_loader)

class BehaviorTreeParseNode(ParseNode):
    def __init__(self, element, desc) -> None:
        super().__init__(element, desc)
        self.name = element.attrib['ID']
        desc.behaviors.append(self)

    def generate(self, ros_node, bt_loader):
        return self.children[0].generate(ros_node, bt_loader) # go through

parse_node_table = {}
parse_node_table['root'] = RootParseNode
parse_node_table['BehaviorTree'] = BehaviorTreeParseNode

#
## behavior tree source file descriptor
#

# TreeDescriptor class
#  its instance keeps info. on each behavior tree source file(.xml)
class TreeDescriptor:
    def __init__(self, file_name, mtime) -> None:
        self.file_name = file_name
        self.mtime = mtime
        self.behaviors = []

# build behavior tree database
# env.tree_table will be updated by instantiation of this class
class TreeLoader:
    def __init__(self, env) -> None:
#        self.cls_loader = ClsLoader(env)
        self.env = env
        new_tree_table = {}
        # scan all behavior tree files and updates env
        for f in self.env.get_tree_list():
            mtime = os.stat(f).st_mtime
            t_desc = self.env.tree_table.get(f)
            if (not t_desc) or (t_desc.mtime < mtime): # need recompilation
                t_desc = TreeDescriptor(f, mtime)
                t_desc.parsed_tree = self.parse_tree(t_desc)
            if t_desc.parsed_tree: new_tree_table[f] = t_desc
        self.env.tree_table = new_tree_table

    def parse_tree(self, t_desc) -> ParseNode:
        try:
            with open(t_desc.file_name, 'r') as f:
                src = f.read()
            xml_element = ET.fromstring(src)
    #        print(f'load behavior {node_name}')
            return self.parse_one_element(xml_element, t_desc)
        except Exception as e:
            print(f'{t_desc.file_name}: parse error({e})')
            return None

    def parse_one_element(self, xml_element, t_desc) -> ParseNode:
        pnode_cls = parse_node_table.get(xml_element.tag)
        if pnode_cls:
            pnode = pnode_cls(xml_element, t_desc)
        else:
            pnode = ParseNode(xml_element, t_desc)
        for n in xml_element:
            pnode.children.append(self.parse_one_element(n, t_desc))
        return pnode
    
    def load_tree(self, name, ros_node) -> TreeDescriptor:
        b_loader = BehaviorTable(self.env)
        for t in self.env.tree_table.values():         
            for b in t.behaviors:
                b_loader.append_behavior_from_tree(b)
        tf_name = self.env.get_tree_file(name)
        if not tf_name:
            raise Exception('unknown tree file name')
        t_desc = self.env.tree_table.get(tf_name)
        if not t_desc:
            t_desc = TreeDescriptor(tf_name, None)
            t_desc.body = self.parse_tree(t_desc)
            for b in t_desc.behavior:
                b_loader.append_behavior_from_tree(b)
        return t_desc.root.generate(ros_node, b_loader)
        
    def scan_tree(self, name) -> None:
        b_loader = BehaviorTable(self.env)
        for t in self.env.tree_table.values():
            for b in t.behaviors:
                b_loader.append_behavior_from_tree(b)
        tf_name = self.env.get_tree_file(name)
        if not tf_name:
            raise Exception('unknown tree file name')
        t_desc = self.env.tree_table.get(tf_name)
        if not t_desc:
            t_desc = TreeDescriptor(tf_name, None)
            t_desc.body = self.parse_tree(t_desc)
            for b in t_desc.behavior:
                b_loader.append_behavior_from_tree(b)
        t_desc.root.scan(b_loader)
            
    