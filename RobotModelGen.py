from RobotGraph import RobotGraph,RobotJoint,RobotLink
from mjcf import elements as e

class ModelGenerator():
    '''
    基于图结构的模型生成器
    对于所有设定和参数,只会生成非默认值
    '''
    def __init__(self,
                 robot:RobotGraph , 
                 path:str = 'mjcf_model/'  , # 存储路径
                 ):
        self.robot = robot # 机器人图
        self.model = e.Mujoco(model=self.robot.graph['name']) # mjcf生成器
        self.path = path
        # 初始化xml文档结构
        self.compiler = e.Compiler()
        self.option = e.Option()
        self.size = e.Size()
        self.custom = e.Custom()
        self.default = e.Default()
        self.asset = e.Asset()
        self.worldbody = e.Worldbody()
        self.actuator = e.Actuator()
        self.model.add_children([self.compiler,
                                 self.option,
                                 self.size,
                                 self.custom,
                                 self.default,
                                 self.asset,
                                 self.worldbody,
                                 self.actuator])
    def set_basic_assets(self,):
        '''
        添加基本的素材：纹理，地面
        TODO 
        '''
        tex1 = e.Texture(
            builtin="gradient",
            height=100,
            rgb1=[1, 1, 1],
            rgb2=[0, 0, 0],
            type="skybox",
            width=100
        )
        tex2 = e.Texture(
            builtin="flat",
            height=1278,
            mark="cross",
            markrgb=[1, 1, 1],
            name="texgeom",
            random=0.01,
            rgb1=[0.8, 0.6, 0.4],
            rgb2=[0.8, 0.6, 0.4],
            type="cube",
            width=127
        )
        tex3 = e.Texture(
            builtin="checker",
            height=[100],
            name="texplane",
            rgb1=[0, 0, 0],
            rgb2=[0.8, 0.8, 0.8],
            type="2d",
            width=100
        )
        mat1 = e.Material(
            name="MatPlane",
            reflectance=0.5,
            shininess=1,
            specular=1,
            texrepeat=[60, 60],
            texture="texplane"
        )
        mat2 = e.Material(
            name="geom",
            texture="texgeom",
            texuniform=True
        )
        self.asset.add_children([
            tex1,
            tex2,
            tex3,
            mat1,
            mat2,
        ])
    
    def set_compiler(self, angle = 'radian', eulerseq = 'xyz',**kwargs):
        self.compiler.angle = angle
        self.compiler.eulerseq = eulerseq

    def set_size(self, njmax = 1000, nconmax = 500 ):
        self.size.njmax = njmax
        self.size.nconmax = nconmax

    def set_option(self, gravity = -9.81):
        self.option.gravity = [0 ,0, gravity]

    def set_ground(self):
        '''
        添加灯光 场地
        '''
        light = e.Light(
            cutoff=100,
            diffuse=[1, 1, 1],
            dir=[-0, 0, -1.3],
            directional=True,
            exponent=1,
            pos=[0, 0, 1.3],
            specular=[.1, .1, .1]
        )
        floor_geom = e.Geom(
            conaffinity=1,
            condim=3,
            material="MatPlane",
            name="floor",
            pos=[0, 0, 0],
            rgba=[0.8, 0.9, 0.8, 1],
            size=[40, 40, 40],
            type="plane"
        )
        self.worldbody.add_children([
                    light,
                    floor_geom
                ])
    def get_link(self, robot_part : RobotLink):
        '''
        添加一个机器人部件的几何体
        return: 该部件的body和geom
        '''
        body =  e.Body(
                name=robot_part.name,
                pos=robot_part.body_pos,
                euler=robot_part.euler
                )
        start_point = list(robot_part.start_point)
        end_point = [start_point[0]+robot_part.length, start_point[1]+0,start_point[2]+0]
        start_point.extend(end_point)
        if robot_part.link_type == 'capsule': # 如果是胶囊形状
            geom =  e.Geom(
                    fromto = start_point,
                    name   = "geom"+robot_part.name,
                    size   = robot_part.size,
                    type   = robot_part.link_type
                        )
        if robot_part.link_type == 'sphere': # 如果是胶囊形状
            geom =  e.Geom(
                    pos = robot_part.geom_pos,
                    name   = "geom"+robot_part.name,
                    size   = robot_part.size,
                    type   = robot_part.link_type
                        )
        return body,geom
        
    def get_joint(self, robot_joint: RobotJoint) :
        '''
        添加一个机器人部件的关节
        return: 该部件的joint
        '''
        joint = e.Joint(
                        axis=robot_joint.axis,
                        name="joint_"+robot_joint.name,
                        pos=robot_joint.pos,
                        range=robot_joint.joint_range,
                        type=robot_joint.joint_type
                            )
        if robot_joint.armature != None:
            joint.armature = robot_joint.armature
        if robot_joint.stiffness != None:
            joint.stiffness = robot_joint.stiffness

        actuator  = e.Motor(
                        ctrllimited=robot_joint.ctrllimited,
                        ctrlrange= robot_joint.ctrlrange,
                        joint="joint_"+robot_joint.name,
                        
                    )
        return joint,actuator


    def get_robot(self,robot_graph:RobotGraph):
        '''
        从机器人图生成一个机器人
        '''     
        if 'root' not in robot_graph.nodes :
            raise ValueError('The robot graph does not have root node.') 
        rootbody, rootgeom = self.get_link(robot_graph.nodes['root']['info'])
        rootbody.add_child(rootgeom)
        if len(list(robot_graph.successors('root'))) == 0:
            raise ValueError(f'The robot graph is empty.')
        
        for n in robot_graph.successors('root'):
            # 遍历root的每一个分支
            first_body = True # 该分支最上层的body
            current_node = n # 每个node为一个dict
            has_joint = False # 标记是否遍历到joint
            last_body = []
            while True:
                if robot_graph.nodes[current_node]['type'] == 'link':
                    body,geom = self.get_link(robot_graph.nodes[current_node]['info'])
                    body.add_child(geom)
                    if has_joint:
                        # 如果有关节需要添加,则遍历该节点的父节点，依次添加关节
                        for p in robot_graph.predecessors(current_node):
                            joint,actuator = self.get_joint(robot_graph.nodes[p]['info'])
                            body.add_child(joint) 
                            self.actuator.add_child(actuator) # 添加驱动
                        has_joint = False
                                      
                    if first_body: # 如果当前分支最上层body为空，则该body为最上层
                        last_body = body
                        rootbody.add_child(last_body)
                        first_body = False
                    else:
                        last_body.add_child(body)
                        last_body = body
                    # 继续下一个节点
                    if len(list(robot_graph.successors(current_node))) == 0:
                        break # 当前分支无子节点，退出
                    next_node = list(robot_graph.successors(current_node))[0]
                    current_node = next_node 
                    continue
                if robot_graph.nodes[current_node]['type'] == 'joint':
                    # 如果当前节点是joint类型节点，继续到下一个节点
                    next_node = list(robot_graph.successors(current_node))[0]
                    has_joint = True
                    current_node = next_node 
                    continue
        self.worldbody.add_child(rootbody)

    def generate(self):
        '''
        输出xml文档
        '''
        model_xml = self.model.xml()
        save_path = self.path + self.robot.graph['name'] + '.xml'
        # Output
        with open(save_path, 'w') as fh:
            fh.write(model_xml)


if __name__ == '__main__':

    R = RobotGraph(name='antrobot')

    root = RobotLink('root',link_type = 'sphere',size=0.25,body_pos=[0,0,2],geom_pos=[0,0,0])
    R.add_node( node_type='link',node_info = root)

    # 添加一条腿
    part1 = RobotLink('part1',length=0.5,size=0.1,body_pos=[0.25,0,0],euler=[0,0,0])    
    R.add_node( node_type='link', node_info=part1)
    R.add_edge(started_node='root',ended_node='part1')
    # 添加joint11和12，part1的子节点，为关节
    joint11 = RobotJoint('joint11',axis=[0,1,0],)
    R.add_node( node_type='joint', node_info=joint11)
    R.add_edge(started_node='part1',ended_node='joint11')
    # 添加小腿
    part2 = RobotLink('part2',length=0.4,size=0.1,body_pos=[0.5,0,0],euler=[0,90,0])    
    R.add_node( node_type='link', node_info=part2)
    R.add_edge(started_node='joint11',ended_node='part2')

    # 添加第二条腿
    part3 = RobotLink('part3',length=0.5,size=0.1,body_pos=[0,0.25,0],euler=[0,0,90])    
    R.add_node( node_type='link', node_info=part3)
    R.add_edge(started_node='root',ended_node='part3')
    # 添加joint11和12，part1的子节点，为关节
    joint31 = RobotJoint('joint31',axis=[0,1,0],)
    R.add_node( node_type='joint', node_info=joint31)
    R.add_edge(started_node='part3',ended_node='joint31')
    # 添加小腿
    part4 = RobotLink('part4',length=0.4,size=0.1,body_pos=[0.5,0,0],euler=[0,90,0])    
    R.add_node( node_type='link', node_info=part4)
    R.add_edge(started_node='joint31',ended_node='part4')


    M = ModelGenerator(R)
    M.set_compiler(angle='degree')
    M.set_basic_assets()
    M.set_size()
    M.set_option(gravity=0)
    M.set_ground()
    M.get_robot(R)

    M.generate()
    print(M.compiler.angle)


