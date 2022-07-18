import getopt
import numpy as np
import matplotlib
from scipy.spatial.transform import Rotation as Rot
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy
import time
import math
import sys


class node:
    
    def __init__(self, name, isroot, isend):
        self.name = name
        self.isroot = isroot
        self.isend = isend
        self.childs = []     # 子节点的列表，从前向后
        self.offset = []     # 当前节点和父节点的 offset
        self.coordinate = [] # 当前节点自己的坐标
        self.joint = []      # 当前关节在绘图板上的信息
        self.bone = []       # 当前关节和父关节连接的骨骼在绘图板上的信息
        self.channels = []   # 自己的输入通道信息
                             # 除了根节点外，坐标会由父节点赋值。
        self.rot = []
        
    
def do_op(str):
    # 这里我们知道第一行必然是 HIERARCHY
    line = 0
    str = str[1:]
    nodelist = []
    root = []
    for i in str:
        fline = i.replace('\n','').replace('\t','')
        if fline == 'MOTION' :
            # 后续均为动作信息
            break
        fline = fline.split(' ')
        print(fline)
        if fline[0] == 'ROOT' :
            # 根节点
            tnode = node(fline[1],True,False)
            nodelist.append(tnode)
        if fline[0] == 'JOINT' or fline[0] == 'End':
            if fline[0] == 'JOINT':
                # 非终止节点/关节
                tnode = node(fline[1],False,False)
            else:
                # 终止节点
                tnode = node(fline[0],False,True)
            fnode = nodelist[-1]
            fnode.childs.append(tnode)
            nodelist.append(tnode)
        if fline[0] == 'OFFSET':
            # 线段长度信息
            fnode = nodelist[-1]
            fnode.offset = np.array(list(map(float,fline[1:])))
        if fline[0] == 'CHANNELS':
            # 节点通道信息
            fnode = nodelist[-1]
            fnode.channels = fline[2:]
        if fline[0] == '}' :
            root = nodelist[-1]
            nodelist.pop()
        line = line + 1
    return str[line: ], root


class Myqueue:
    ## 手写一个队列
    ## 该队列用于输入时顺次信息的读入
    def __init__(self,queue):
        self.queue = queue
        self.len = len(self.queue)
        self.pnt = 0
    
    def pop_front(self):
        # 弹栈
        if self.pnt >= self.len:
            raise ValueError("队列为空 !!")
        value = self.queue[self.pnt]
        self.pnt = self.pnt + 1
        return value


def TransMat(rotX, rotY, rotZ):
    AngX = rotX * np.pi / 180.0
    AngY = rotY * np.pi / 180.0
    AngZ = rotZ * np.pi / 180.0
    MatX = np.array([[1, 0           , 0            ],
                     [0, np.cos(AngX), -np.sin(AngX)],
                     [0, np.sin(AngX), np.cos(AngX) ]])
    MatY = np.array([[np.cos(AngY) , 0, np.sin(AngY)],
                     [0            , 1, 0           ],
                     [-np.sin(AngY), 0, np.cos(AngY)]])
    MatZ = np.array([[np.cos(AngZ), -np.sin(AngZ), 0],
                     [np.sin(AngZ), np.cos(AngZ) , 0],
                     [0           , 0            , 1]])
    return MatX @ MatY @ MatZ

def draw(root, par, queue, frame):
    
    mp = {}
    for i in root.channels:
        # print(i)
        mp[i] = queue.pop_front()
    if root.isroot == True:
        ## 是根节点，根据输入确定坐标
        root.coordinate = np.array([mp['Xposition'],mp['Yposition'],mp['Zposition']])
        root.rot = TransMat(mp['Xrotation'],mp['Yrotation'],mp['Zrotation'])
    else:
        ## 不是根节点，根据和父节点的关系
        offset = np.dot(root.offset, par.rot.T)
        root.coordinate = par.coordinate + offset
        arrayx = np.array([root.coordinate[0],par.coordinate[0]])
        arrayy = np.array([root.coordinate[1],par.coordinate[1]])
        arrayz = np.array([root.coordinate[2],par.coordinate[2]])
        root.bone, = frame.plot(arrayx,arrayz,arrayy,color='red')
        if root.isend == False:
            root.rot = par.rot @ TransMat(mp['Xrotation'],mp['Yrotation'],mp['Zrotation'])
    root.joint, = frame.plot(root.coordinate[0],root.coordinate[2],root.coordinate[1],'bo')
                                 
    for child in root.childs:
        draw(child, root, queue, frame)

def update(root, par, queue):

    mp = {}
    for i in root.channels:
        mp[i] = queue.pop_front()
    if root.isroot == True:
        ## 是根节点，根据输入确定坐标
        root.coordinate = np.array([mp['Xposition'],mp['Yposition'],mp['Zposition']])
        root.rot = TransMat(mp['Xrotation'],mp['Yrotation'],mp['Zrotation'])
    else:
        offset = np.dot(root.offset, par.rot.T)
        ## 不是根节点，根据和父节点的关系
        root.coordinate = par.coordinate + offset
        root.bone.set_data(np.array([[root.coordinate[0],par.coordinate[0]],
                                     [root.coordinate[2],par.coordinate[2]]]))
        root.bone.set_3d_properties(np.array([root.coordinate[1],par.coordinate[1]]))
        if root.isend == False:
            root.rot = par.rot @ TransMat(mp['Xrotation'],mp['Yrotation'],mp['Zrotation'])
    root.joint.set_data(np.array([[root.coordinate[0]],
                                  [root.coordinate[2]]]))
    root.joint.set_3d_properties(np.array([root.coordinate[1]]))
                                 
    for child in root.childs:
        update(child, root, queue)
        
if __name__ == "__main__":
    BVH_file = "0008_Walking002.bvh"
    fin = open(BVH_file, "r")
    str = fin.readlines()
    str, root = do_op(str)
    
    fig = plt.figure()
    frame = p3.Axes3D(fig)
    frame.set_xlim3d([-40,40])
    frame.set_ylim3d([-40,40])
    frame.set_zlim3d([0,80])
    frame.set_xlabel('X')
    frame.set_ylabel('Y')
    frame.set_zlabel('Z')
    
    framecnt = int(str[1].replace('\n','').replace('\t','').split(' ')[1]) ## 帧数
    fps = int(1.0 / float(str[2].replace('\n','').replace('\t','').split(' ')[2])) ## fps
    str = str[3:]
    print(framecnt)
    
    def update_dots(info):
        queue, idx = info
        update(root,root,queue)
        print("Frame Done",idx,time.time())
    
    def init_dots():
        channels = list(map(float,str[0][1:].replace('\n','').replace('\t','').split(' ')))
        channels = Myqueue(channels)
        draw(root,root,channels,frame)
        
    def gen_dots():
        idx = 1
        for i in str[1:]:
            channels = list(map(float,i[1:].replace('\n','').replace('\t','').split(' ')))
            channels = Myqueue(channels)
            yield (channels, idx)
            idx = idx + 1
    
    ani = animation.FuncAnimation(fig, update_dots, frames = gen_dots,
                                  init_func=init_dots, save_count = 300, interval = 5, repeat = False)
    fout = BVH_file.split('.')[0]
    ani.save(fout + '.gif', writer='imagemagick', fps=fps)

    plt.show()
