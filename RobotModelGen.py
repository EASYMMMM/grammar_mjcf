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
    
    def set_compiler(self, angle = 'radian',**kwargs):
        self.compiler.angle = angle

    def set_size(self, njmax = 1000, nconmax = 500 ):
        self.size.njmax = njmax
        self.size.nconmax = nconmax

    def set_option(self, gravity = -9.81):
        self.option.gravity = [ 0 ,0,  gravity]

    def get_linkage(self, robot_part : RobotLink):
        geom =  e.Body(
                name=robot_part.name,
                length=robot_part.lengh,
                euler=[0, 0, ]
            )
     


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

    R = RobotGraph(name='xmlrobot')
    M = ModelGenerator(R)
    M.set_compiler()
    M.set_basic_assets()
    M.set_size()
    M.set_option(gravity=0)


    M.generate()
    print(M.compiler.angle)


