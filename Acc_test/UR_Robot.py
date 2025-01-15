#coding=utf8
import os
import time
import socket
import struct
import numpy as np
import math
from realsenseD435 import RealsenseD435 as Camera 


class UR_Robot:
    def __init__(self, tcp_host_ip="192.168.3.6", tcp_port=30003, workspace_limits=None,
                 is_use_camera=True):
        # 初始化变量
        if workspace_limits is None:
            workspace_limits = [[-0.7, 0.7], [-0.7, 0.7], [0.00, 0.6]]
        self.workspace_limits = workspace_limits
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        self.is_use_camera = is_use_camera


        # UR5 机器人配置
        # 默认关节/工具速度配置
        self.joint_acc = 1.4  # 安全值: 1.4   8
        self.joint_vel = 1.05  # 安全值: 1.05  3

        # 阻塞调用的关节容差
        self.joint_tolerance = 0.01

        # 默认工具速度配置
        self.tool_acc = 0.5 # 安全值: 0.5
        self.tool_vel = 0.2  # 安全值: 0.2

        # 阻塞调用的工具姿态容差
        self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        # 默认机器人回到初始关节配置（机器人悬空状态）
        self.home_joint_config = [-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             (-90 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             (90 / 360.0) * 2 * np.pi, 0.0]
    
    # 关节控制
    '''
    输入: joint_configuration = 关节角度
    '''
    def move_j(self, joint_configuration, k_acc=1, k_vel=1, t=0, r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "movej([%f" % joint_configuration[0]
        for joint_idx in range(1,6):
            tcp_command += ",%f" % joint_configuration[joint_idx]
        tcp_command += "],a=%f,v=%f,t=%f,r=%f)\n" % (k_acc*self.joint_acc, k_vel*self.joint_vel, t, r)
        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态
        state_data = self.tcp_socket.recv(1500)
        actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
        while not all([np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
            time.sleep(0.01)
        self.tcp_socket.close()
    
    # 工具坐标控制
    '''
    move_j_p(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0)
    输入: tool_configuration = [x, y, z, r, p, y]
    其中 x, y, z 为三个轴的目标位置坐标，单位为米
    r, p, y 为旋转角度，单位为弧度
    '''
    def move_j_p(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movej_p([{tool_configuration}])")
        # 命令: movej([joint_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command += " array = rpy2rotvec([%f,%f,%f])\n" % (tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += "movej(get_inverse_kin(p[%f,%f,%f,array[0],array[1],array[2]]),a=%f,v=%f,t=%f,r=%f)\n" % (
            tool_configuration[0], tool_configuration[1], tool_configuration[2],
            k_acc * self.joint_acc, k_vel * self.joint_vel, t, r)
        tcp_command += "end\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        time.sleep(1.5)
        self.tcp_socket.close()
    
    # 直线运动控制
    def move_l(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        print(f"movel([{tool_configuration}])")
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # 命令: movel([tool_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command += " array = rpy2rotvec([%f,%f,%f])\n" % (tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += "movel(p[%f,%f,%f,array[0],array[1],array[2]],a=%f,v=%f,t=%f,r=%f)\n" % (
            tool_configuration[0], tool_configuration[1], tool_configuration[2],
            k_acc * self.joint_acc, k_vel * self.joint_vel, t, r)
        tcp_command += "end\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        time.sleep(1.5)
        self.tcp_socket.close()
    
    # 圆弧运动控制（通常不使用）
    # mode 0: 无约束模式。插值当前姿态到目标姿态（pose_to）的方向
    # mode 1: 固定模式。保持相对于圆弧切线的姿态不变（从当前姿态开始）
    def move_c(self, pose_via, tool_configuration, k_acc=1, k_vel=1, r=0, mode=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movec([{pose_via},{tool_configuration}])")
        # 命令: movec([pose_via, tool_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command += " via_pose = rpy2rotvec([%f,%f,%f])\n" % (pose_via[3], pose_via[4], pose_via[5])
        tcp_command += " tool_pose = rpy2rotvec([%f,%f,%f])\n" % (tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command = f" movec([{pose_via[0]},{pose_via[1]},{pose_via[2]},via_pose[0],via_pose[1],via_pose[2]], \
                [{tool_configuration[0]},{tool_configuration[1]},{tool_configuration[2]},tool_pose[0],tool_pose[1],tool_pose[2]], \
                a={k_acc * self.tool_acc},v={k_vel * self.tool_vel},r={r})\n"
        tcp_command += "end\n"

        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = self.tcp_socket.recv(1500)
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        self.tcp_socket.close()
        time.sleep(1.5)
    
    def go_home(self):
        self.move_j(self.home_joint_config)

    # 获取机器人当前状态和信息
    def get_state(self):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        state_data = self.tcp_socket.recv(1500)
        self.tcp_socket.close()
        return state_data
    
    # 获取机器人当前关节角度和笛卡尔姿态
    def parse_tcp_state_data(self, data, subpackage):
        dic = {'MessageSize': 'i', 'Time': 'd', 'q target': '6d', 'qd target': '6d', 'qdd target': '6d',
               'I target': '6d',
               'M target': '6d', 'q actual': '6d', 'qd actual': '6d', 'I actual': '6d', 'I control': '6d',
               'Tool vector actual': '6d', 'TCP speed actual': '6d', 'TCP force': '6d', 'Tool vector target': '6d',
               'TCP speed target': '6d', 'Digital input bits': 'd', 'Motor temperatures': '6d', 'Controller Timer': 'd',
               'Test value': 'd', 'Robot Mode': 'd', 'Joint Modes': '6d', 'Safety Mode': 'd', 'empty1': '6d',
               'Tool Accelerometer values': '3d',
               'empty2': '6d', 'Speed scaling': 'd', 'Linear momentum norm': 'd', 'SoftwareOnly': 'd',
               'softwareOnly2': 'd',
               'V main': 'd',
               'V robot': 'd', 'I robot': 'd', 'V actual': '6d', 'Digital outputs': 'd', 'Program state': 'd',
               'Elbow position': 'd', 'Elbow velocity': '3d'}
        ii = range(len(dic))
        for key, i in zip(dic, ii):
            fmtsize = struct.calcsize(dic[key])
            data1, data = data[0:fmtsize], data[fmtsize:]
            fmt = "!" + dic[key]
            dic[key] = dic[key], struct.unpack(fmt, data1)

        if subpackage == 'joint_data':  # 获取关节数据
            q_actual_tuple = dic["q actual"]
            joint_data = np.array(q_actual_tuple[1])
            return joint_data
        elif subpackage == 'cartesian_info':
            Tool_vector_actual = dic["Tool vector actual"]  # 获取 x y z rx ry rz
            cartesian_info = np.array(Tool_vector_actual[1])
            return cartesian_info

    def rpy2rotating_vector(self, rpy):
        # RPY 转旋转向量
        R = self.rpy2R(rpy)
        return self.R2rotating_vector(R)

    def rpy2R(self, rpy):  # [r, p, y] 单位 rad
        rot_x = np.array([[1, 0, 0],
                          [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                          [0, math.sin(rpy[0]), math.cos(rpy[0])]])
        rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                          [0, 1, 0],
                          [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
        rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                          [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                          [0, 0, 1]])
        R = np.dot(rot_z, np.dot(rot_y, rot_x))
        return R

    def R2rotating_vector(self, R):
        theta = math.acos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
        print(f"theta:{theta}")
        rx = (R[2, 1] - R[1, 2]) / (2 * math.sin(theta))
        ry = (R[0, 2] - R[2, 0]) / (2 * math.sin(theta))
        rz = (R[1, 0] - R[0, 1]) / (2 * math.sin(theta))
        return np.array([rx, ry, rz]) * theta

    def R2rpy(self, R):
        # assert (isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    ## 获取相机数据 
    def get_camera_data(self):
        color_img, depth_img = self.camera.get_data()
        return color_img, depth_img

    # 测试机器人控制
    def testRobot(self):
        try:
            print("测试机器人中...")
            self.move_j([-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             -(0 / 360.0) * 2 * np.pi, 0.0])
            time.sleep(1)
            # self.move_j([(57.04 / 360.0) * 2 * np.pi, (-65.26/ 360.0) * 2 * np.pi,
            #                  (73.52/ 360.0) * 2 * np.pi, (-100.89/ 360.0) * 2 * np.pi,
            #                  (-86.93/ 360.0) * 2 * np.pi, (-0.29/360)*2*np.pi])
            # time.sleep(1)
            # self.move_j([(57.03 / 360.0) * 2 * np.pi, (-56.67 / 360.0) * 2 * np.pi,
            #                   (88.72 / 360.0) * 2 * np.pi, (-124.68 / 360.0) * 2 * np.pi,
            #                   (-86.96/ 360.0) * 2 * np.pi, (-0.3/ 360) * 2 * np.pi])
            # time.sleep(1)
            # self.move_j([(57.04 / 360.0) * 2 * np.pi, (-65.26 / 360.0) * 2 * np.pi,
            #                   (73.52 / 360.0) * 2 * np.pi, (-100.89 / 360.0) * 2 * np.pi,
            #                   (-86.93 / 360.0) * 2 * np.pi, (-0.29 / 360) * 2 * np.pi])
            # time.sleep(1)
            # self.move_j([-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
            #                  -(0 / 360.0) * 2 * np.pi, 0.0])
            # time.sleep(1)
            # self.move_j_p([0.3,0,0.3,np.pi/2,0,0],0.5,0.5)
            # time.sleep(1)
            # for i in range(10):
            #      self.move_j_p([0.3, 0, 0.3, np.pi, 0, i*0.1], 0.5, 0.5)
            #     #  time.sleep(1)
            # self.move_j_p([0.3, 0, 0.3, -np.pi, 0, 0],0.5,0.5)
            # time.sleep(1)
            # self.move_p([0.3, 0.3, 0.3, -np.pi, 0, 0],0.5,0.5)
            # time.sleep(1)
            # self.move_l([0.2, 0.2, 0.3, -np.pi, 0, 0],0.5,0.5)
            # time.sleep(1)
            # self.plane_grasp([0.3, 0.3, 0.1])
            # time.sleep(1)
            # self.plane_push([0.3, 0.3, 0.1])
            # time.sleep(1)
        except:
            print("测试失败！")

if __name__ =="__main__":
    ur_robot = UR_Robot()
    ur_robot.testRobot()
