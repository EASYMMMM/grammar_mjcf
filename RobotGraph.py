import networkx as nx
from networkx import DiGraph

class RobotLink():
    '''
    机器人的link
    '''
    def __init__(self,
                 name,
                 length,
                 size,
                 link_type = 'capsule'
                ):
        self.name = name
        self.length = length
        self.size = size
        self.link_type = link_type

class RobotJoint():
    '''
    机器人的关节
    '''
    def __init__(self,
                 name='abdomen_z',
                 joint_type='hinge',
                 armature=0.02,
                 axis=[0,0,1],
                 pos=[0,0,0],
                 joint_range=[-45,45],
                 stiffness=20  ):
        self.name = name
        self.length = joint_type
        self.armature = armature
        self.axis = axis
        self.pos = pos
        self.joint_range = joint_range
        self.stiffness = 20


class RobotGraph(DiGraph):

    def __init__(self, name = 'RobotGraph'):
        super().__init__(self,name=name)
    
    def add_node(self, node_name :str = None, Node_info = None):
        if not node_name:
            raise  ValueError("The node need a name.")
        super().add_node(node_name, info = Node_info)
    
    def add_edge(self, started_node, ended_node, **attr):
        
        return super().add_edge(started_node, ended_node, **attr)
    
    

if __name__ == '__main__':

    R = RobotGraph(name='xmlrobot')
    print(R.graph['name'])