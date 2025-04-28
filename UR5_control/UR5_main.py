#coding=utf8
import os
import time
import socket
import struct
import numpy as np
import math

"""
在main函数中修改ur5地址即可
"""

class UR_Robot:
    def __init__(self, tcp_host_ip="192.168.1.15", tcp_port=30003, workspace_limits=None):
        # 初始化变量
        if workspace_limits is None:
            workspace_limits = [[-0.7, 0.7], [-0.7, 0.7], [0.00, 0.6]]
        self.workspace_limits = workspace_limits
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

        # UR5 机器人配置
        # 默认关节/工具速度配置
        self.joint_acc = 0.7  # 安全值: 1.4   8
        self.joint_vel = 0.7  # 安全值: 1.05  3

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
            try: # 添加 try-except 块以处理可能的套接字错误
                state_data = self.tcp_socket.recv(1500)
                if not state_data:
                    print("警告: 套接字连接已关闭，可能未到达目标位置。")
                    break
                actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
                time.sleep(0.01)
            except socket.error as e:
                print(f"套接字错误: {e}")
                break
            except Exception as e:
                print(f"解析状态数据时出错: {e}")
                # 可以选择继续或中断，这里选择中断
                break
        self.tcp_socket.close()

    # 工具坐标控制
    '''
    move_j_p(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0)
    输入: tool_configuration = [x, y, z, r, p, y]
    其中 x, y, z 为三个轴的目标位置坐标，单位为米
    r, p, y 为旋转角度，单位为弧度 (注意：URScript 使用旋转向量，代码中做了转换)
    '''
    def move_j_p(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movej_p([{tool_configuration}])")
        # 命令: movej(pose, a, v, t, r)\n, pose 通过 get_inverse_kin(p[...]) 获得
        tcp_command = "def process():\n"
        # 注意： URScript 的 p[...] 需要的是 旋转向量 (rx, ry, rz) 而不是 RPY (r, p, y)
        # 代码中使用了 URScript 内置函数 rpy2rotvec 进行转换
        tcp_command += " target_pose = p[%f,%f,%f,0,0,0]\n" % (tool_configuration[0], tool_configuration[1], tool_configuration[2])
        tcp_command += " target_rpy = [%f,%f,%f]\n" % (tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += " target_rotvec = rpy2rotvec(target_rpy)\n"
        tcp_command += " target_pose[3] = target_rotvec[0]\n"
        tcp_command += " target_pose[4] = target_rotvec[1]\n"
        tcp_command += " target_pose[5] = target_rotvec[2]\n"
        tcp_command += " movej(get_inverse_kin(target_pose), a=%f, v=%f, t=%f, r=%f)\n" % (
            k_acc * self.joint_acc, k_vel * self.joint_vel, t, r)
        tcp_command += "end\n"
        # print(f"Sending command:\n{tcp_command}") # 用于调试
        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态 (只检查位置 x, y, z)
        start_time = time.time()
        timeout = 30 # 设置一个超时时间，例如30秒
        while time.time() - start_time < timeout:
            try:
                state_data = self.tcp_socket.recv(1500)
                if not state_data:
                    print("警告: 套接字连接在 move_j_p 期间关闭，可能未到达目标位置。")
                    break
                actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
                # 检查位置是否足够接近
                position_reached = all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)])
                if position_reached:
                    # print("目标位置已到达。") # 用于调试
                    break # 位置到达，退出循环
                time.sleep(0.01)
            except socket.error as e:
                print(f"move_j_p 中套接字错误: {e}")
                break
            except Exception as e:
                print(f"move_j_p 中解析状态数据时出错: {e}")
                break
        else: # 如果循环是因为超时而结束
             print("警告: move_j_p 超时，机器人可能未到达目标位置。")

        # time.sleep(1.5) # 这个延时可能不需要，或者可以缩短
        self.tcp_socket.close()

    # 直线运动控制
    def move_l(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        print(f"movel([{tool_configuration}])")
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        # 命令: movel(pose,a,v,t,r)\n
        tcp_command = "def process():\n"
        # 同样，URScript 的 p[...] 需要旋转向量
        tcp_command += " target_pose = p[%f,%f,%f,0,0,0]\n" % (tool_configuration[0], tool_configuration[1], tool_configuration[2])
        tcp_command += " target_rpy = [%f,%f,%f]\n" % (tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += " target_rotvec = rpy2rotvec(target_rpy)\n"
        tcp_command += " target_pose[3] = target_rotvec[0]\n"
        tcp_command += " target_pose[4] = target_rotvec[1]\n"
        tcp_command += " target_pose[5] = target_rotvec[2]\n"
        tcp_command += " movel(target_pose, a=%f, v=%f, t=%f, r=%f)\n" % (
            k_acc * self.tool_acc, k_vel * self.tool_vel, t, r) # 注意这里用了 tool_acc, tool_vel
        tcp_command += "end\n"
        # print(f"Sending command:\n{tcp_command}") # 用于调试
        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态 (只检查位置 x, y, z)
        start_time = time.time()
        timeout = 30 # 设置一个超时时间，例如30秒
        while time.time() - start_time < timeout:
            try:
                state_data = self.tcp_socket.recv(1500)
                if not state_data:
                    print("警告: 套接字连接在 move_l 期间关闭，可能未到达目标位置。")
                    break
                actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
                # 检查位置是否足够接近
                position_reached = all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)])
                if position_reached:
                    # print("目标位置已到达。") # 用于调试
                    break # 位置到达，退出循环
                time.sleep(0.01)
            except socket.error as e:
                print(f"move_l 中套接字错误: {e}")
                break
            except Exception as e:
                print(f"move_l 中解析状态数据时出错: {e}")
                break
        else: # 如果循环是因为超时而结束
            print("警告: move_l 超时，机器人可能未到达目标位置。")

        # time.sleep(1.5) # 这个延时可能不需要，或者可以缩短
        self.tcp_socket.close()

    # 圆弧运动控制（通常不使用）
    # mode 0: 无约束模式。插值当前姿态到目标姿态（pose_to）的方向
    # mode 1: 固定模式。保持相对于圆弧切线的姿态不变（从当前姿态开始）
    def move_c(self, pose_via, tool_configuration, k_acc=1, k_vel=1, r=0, mode=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movec([{pose_via},{tool_configuration}])")
        # 命令: movec(pose_via, pose_to, a, v, r, mode)\n
        tcp_command = "def process():\n"
        # 转换 pose_via 的 RPY 为旋转向量
        tcp_command += " via_pose = p[%f,%f,%f,0,0,0]\n" % (pose_via[0], pose_via[1], pose_via[2])
        tcp_command += " via_rpy = [%f,%f,%f]\n" % (pose_via[3], pose_via[4], pose_via[5])
        tcp_command += " via_rotvec = rpy2rotvec(via_rpy)\n"
        tcp_command += " via_pose[3] = via_rotvec[0]\n"
        tcp_command += " via_pose[4] = via_rotvec[1]\n"
        tcp_command += " via_pose[5] = via_rotvec[2]\n"
        # 转换 tool_configuration (pose_to) 的 RPY 为旋转向量
        tcp_command += " tool_pose = p[%f,%f,%f,0,0,0]\n" % (tool_configuration[0], tool_configuration[1], tool_configuration[2])
        tcp_command += " tool_rpy = [%f,%f,%f]\n" % (tool_configuration[3], tool_configuration[4], tool_configuration[5])
        tcp_command += " tool_rotvec = rpy2rotvec(tool_rpy)\n"
        tcp_command += " tool_pose[3] = tool_rotvec[0]\n"
        tcp_command += " tool_pose[4] = tool_rotvec[1]\n"
        tcp_command += " tool_pose[5] = tool_rotvec[2]\n"
        # 构建 movec 命令
        tcp_command += f" movec(via_pose, tool_pose, a={k_acc * self.tool_acc}, v={k_vel * self.tool_vel}, r={r}, mode={mode})\n" # 注意这里用了 tool_acc, tool_vel
        tcp_command += "end\n"
        # print(f"Sending command:\n{tcp_command}") # 用于调试
        self.tcp_socket.send(str.encode(tcp_command))

        # 阻塞直到机器人达到目标状态 (只检查位置 x, y, z)
        start_time = time.time()
        timeout = 45 # 圆弧运动可能需要更长时间
        while time.time() - start_time < timeout:
            try:
                state_data = self.tcp_socket.recv(1500)
                if not state_data:
                     print("警告: 套接字连接在 move_c 期间关闭，可能未到达目标位置。")
                     break
                actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
                # 检查位置是否足够接近
                position_reached = all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)])
                if position_reached:
                    # print("目标位置已到达。") # 用于调试
                    break # 位置到达，退出循环
                time.sleep(0.01)
            except socket.error as e:
                print(f"move_c 中套接字错误: {e}")
                break
            except Exception as e:
                print(f"move_c 中解析状态数据时出错: {e}")
                break
        else: # 如果循环是因为超时而结束
            print("警告: move_c 超时，机器人可能未到达目标位置。")

        # time.sleep(1.5) # 这个延时可能不需要，或者可以缩短
        self.tcp_socket.close()

    def go_home(self):
        print("回到初始位置...")
        self.move_j(self.home_joint_config)
        print("已回到初始位置。")

    # 获取机器人当前状态和信息
    def get_state(self):
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5) # 设置超时避免无限等待
            self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
            state_data = self.tcp_socket.recv(1500) # UR E系列通常是1116字节，但1500足够
            self.tcp_socket.close()
            return state_data
        except socket.timeout:
            print("获取机器人状态超时！")
            return None
        except socket.error as e:
            print(f"获取机器人状态时发生套接字错误: {e}")
            return None
        except Exception as e:
            print(f"获取机器人状态时发生未知错误: {e}")
            return None


    # 获取机器人当前关节角度和笛卡尔姿态
    def parse_tcp_state_data(self, data, subpackage):
        if data is None or len(data) < 1044: # 检查数据是否有效 (CB3: 1044, E-series: 1116)
            print(f"错误: 接收到的状态数据无效或过短 (长度: {len(data) if data else 0})")
            # 返回默认值或引发异常，这里返回None或默认零数组
            if subpackage == 'joint_data':
                return np.zeros(6) # 或者 None
            elif subpackage == 'cartesian_info':
                return np.zeros(6) # 或者 None
            else:
                return None # 或者空的字典
        dic = {}
        offset = 0
        # Message Size (int)
        fmtsize = struct.calcsize("!i")
        val = struct.unpack("!i", data[offset:offset+fmtsize])[0]
        dic['MessageSize'] = val
        offset += fmtsize
        # Time (double)
        fmtsize = struct.calcsize("!d")
        val = struct.unpack("!d", data[offset:offset+fmtsize])[0]
        dic['Time'] = val
        offset += fmtsize
        # ... (跳过一些目标值)
        offset += 6 * 8 # q target
        offset += 6 * 8 # qd target
        offset += 6 * 8 # qdd target
        offset += 6 * 8 # I target
        offset += 6 * 8 # M target
        # q actual (double[6]) - Actual joint positions
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['q actual'] = val
        offset += fmtsize
        # qd actual (double[6]) - Actual joint velocities
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['qd actual'] = val
        offset += fmtsize
        # I actual (double[6]) - Actual joint currents
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['I actual'] = val
        offset += fmtsize
         # I control (double[6]) - Joint control currents
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['I control'] = val
        offset += fmtsize
        # Tool vector actual (double[6]) - Actual Cartesian coordinates of TCP: (x,y,z,rx,ry,rz)
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['Tool vector actual'] = val
        offset += fmtsize
        # TCP speed actual (double[6]) - Actual speed of TCP given in base frame
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['TCP speed actual'] = val
        offset += fmtsize
        # TCP force (double[6]) - Generalized forces in the TCP
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['TCP force'] = val
        offset += fmtsize
        # Tool vector target (double[6]) - Target Cartesian coordinates of TCP: (x,y,z,rx,ry,rz)
        fmtsize = struct.calcsize("!6d")
        val = struct.unpack("!6d", data[offset:offset+fmtsize])
        dic['Tool vector target'] = val
        offset += fmtsize
        # ... (可以根据需要继续解析其他字段)

        # 根据请求返回数据
        if subpackage == 'joint_data':  # 获取关节数据
            joint_data = np.array(dic.get("q actual", np.zeros(6))) # 使用 get 提供默认值
            return joint_data
        elif subpackage == 'cartesian_info':
            cartesian_info = np.array(dic.get("Tool vector actual", np.zeros(6))) # 获取 x y z rx ry rz
            return cartesian_info
        elif subpackage == 'all': # 如果需要所有解析的数据
             return dic
        else:
             print(f"警告: 未知的子包请求 '{subpackage}'")
             return None

    # --- 姿态转换辅助函数 ---
    # 注意: URScript 的 rpy2rotvec 和 rotvec2rpy 可能与常见的 ZYX 欧拉角定义不同。
    # 这些 Python 辅助函数是基于标准 ZYX 欧拉角计算的，如果需要严格匹配 URScript 内部转换，
    # 最好的方法是在 URScript 端完成转换，如此处 move_j_p, move_l, move_c 中所示。
    # 如果要在 Python 端计算旋转向量给 URScript 使用，需要确保旋转定义一致。

    def rpy2rotating_vector(self, rpy):
        # RPY (roll, pitch, yaw) ZYX 欧拉角转旋转向量
        R = self.rpy2R(rpy)
        return self.R2rotating_vector(R)

    def rpy2R(self, rpy):  # [roll, pitch, yaw] 单位 rad, ZYX顺序
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        cos_r, sin_r = math.cos(roll), math.sin(roll)
        cos_p, sin_p = math.cos(pitch), math.sin(pitch)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        # ZYX 顺序对应的旋转矩阵
        R_z = np.array([[cos_y, -sin_y, 0],
                        [sin_y, cos_y,  0],
                        [0,     0,      1]])
        R_y = np.array([[cos_p,  0, sin_p],
                        [0,      1, 0],
                        [-sin_p, 0, cos_p]])
        R_x = np.array([[1, 0,     0],
                        [0, cos_r, -sin_r],
                        [0, sin_r, cos_r]])

        # R = Rz * Ry * Rx
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def R2rotating_vector(self, R):
        # 旋转矩阵转旋转向量 (轴角表示)
        angle = math.acos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0)) # 使用 clip 防止 acos 域错误

        if abs(angle) < 1e-6: # 接近零旋转
            return np.array([0.0, 0.0, 0.0])
        if abs(angle - np.pi) < 1e-6: # 接近 180 度旋转
            # 此时 sin(angle) 接近 0，需要特殊处理
            # 可以从 R + I 矩阵中找到旋转轴的方向向量
            # R = np.array(R) # 确保是 numpy 数组
            # M = (R + np.identity(3)) / 2.0
            # # 寻找 M 中绝对值最大的对角线元素对应的轴
            # diag_abs = np.abs(np.diag(M))
            # axis_idx = np.argmax(diag_abs)
            # axis_vec = np.sqrt(M[axis_idx, axis_idx]) * np.sign(R[...]) # 需要更鲁棒的方法
            # 简化处理：尝试几种可能的轴组合，但这并不完全通用
            # 更可靠的方法是通过解 R*v = v 来找特征向量，但这比较复杂
            # 或者使用四元数作为中间表示
            # 为了简单起见，这里可以返回一个近似值或标记错误
            # print("警告: 接近 180 度旋转，旋转向量计算可能不精确")
            # 尝试从反对称部分计算（不精确但有时可用）
            rx = np.sqrt(np.clip((R[0, 0] + 1) / 2.0, 0, 1)) * np.sign(R[2, 1] - R[1, 2]) if abs(R[2, 1] - R[1, 2]) > 1e-5 else np.sqrt(np.clip((R[0, 0] + 1) / 2.0, 0, 1))
            ry = np.sqrt(np.clip((R[1, 1] + 1) / 2.0, 0, 1)) * np.sign(R[0, 2] - R[2, 0]) if abs(R[0, 2] - R[2, 0]) > 1e-5 else np.sqrt(np.clip((R[1, 1] + 1) / 2.0, 0, 1))
            rz = np.sqrt(np.clip((R[2, 2] + 1) / 2.0, 0, 1)) * np.sign(R[1, 0] - R[0, 1]) if abs(R[1, 0] - R[0, 1]) > 1e-5 else np.sqrt(np.clip((R[2, 2] + 1) / 2.0, 0, 1))
            vec = np.array([rx, ry, rz])
            # 归一化
            norm = np.linalg.norm(vec)
            if norm < 1e-6: # 如果向量接近零，则无法确定轴
                 # 这种情况理论上不应发生，除非 R 不是有效的旋转矩阵
                 print("错误: 无法计算 180 度旋转的轴")
                 return np.array([angle, 0.0, 0.0]) # 返回一个默认轴，例如绕X轴旋转pi
            axis = vec / norm
            return axis * angle


        # 一般情况
        sin_angle = math.sin(angle)
        rx = (R[2, 1] - R[1, 2]) / (2 * sin_angle)
        ry = (R[0, 2] - R[2, 0]) / (2 * sin_angle)
        rz = (R[1, 0] - R[0, 1]) / (2 * sin_angle)
        return np.array([rx, ry, rz]) * angle

    def R2rpy(self, R):
        # 旋转矩阵转 RPY (roll, pitch, yaw) ZYX 欧拉角
        # assert (isRotationMatrix(R)) # 可以添加一个检查函数 isRotationMatrix
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            # ZYX: x=roll, y=pitch, z=yaw
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            # 万向锁情况 (Gimbal lock), 当 pitch 为 +/- 90度时
            pitch = math.atan2(-R[2, 0], sy) # pitch is +/- pi/2
            # 此时 roll 和 yaw 耦合，通常将其中一个设为 0
            roll = math.atan2(-R[1, 2], R[1, 1]) # 计算 roll+yaw 或 roll-yaw
            yaw = 0 # 设定 yaw 为 0

        return np.array([roll, pitch, yaw])

    # 移除了: get_camera_data(self)

    # 测试机器人控制
    def testRobot(self):
        try:
            print("测试机器人中...")

            # 1. 获取初始状态
            # print("获取初始状态...")
            # initial_state_data = self.get_state()
            # if initial_state_data:
            #     initial_joints = self.parse_tcp_state_data(initial_state_data, 'joint_data')
            #     initial_pose = self.parse_tcp_state_data(initial_state_data, 'cartesian_info')
            #     print(f"初始关节角度: {np.degrees(initial_joints)}")
            #     print(f"初始工具姿态 (m, rad): {initial_pose}")
            # else:
            #      print("无法获取初始状态，测试中止。")
            #      return

            # 2. 移动到预备位置 (使用关节控制)
            print("\n移动到预备关节位置...")
            prep_joint_config = [-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                                 (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                                 (90 / 360.0) * 2 * np.pi, 0.0] # 稍微抬起点
            self.move_j(prep_joint_config, k_acc=0.8, k_vel=0.8) # 使用稍慢的速度和加速度
            time.sleep(1)

            # 3. 获取当前姿态，并基于此进行测试移动
            print("\n获取当前姿态用于后续测试...")
            current_state_data = self.get_state()
            if not current_state_data:
                 print("无法获取当前状态，测试中止。")
                 return
            current_cartesian_pose = self.parse_tcp_state_data(current_state_data, 'cartesian_info')
            print(f"当前工具姿态 (m, rad): {current_cartesian_pose}")

            # 提取当前位置和 RPY 姿态 (需要将旋转向量转为 RPY)
            current_position = current_cartesian_pose[:3]
            current_rotvec = current_cartesian_pose[3:]
            # URScript 的 rotvec2rpy 可能有不同的定义，这里我们使用 Python 实现
            # R = self.rotating_vector2R(current_rotvec) # 需要实现 rotating_vector2R
            # current_rpy = self.R2rpy(R)
            # 为了简单，我们设定一个已知的目标RPY
            target_rpy_test = np.array([np.pi, 0, np.pi/2]) # 例如：翻转手腕，Z轴指向侧面


            # 4. 测试 move_j_p (关节空间移动到目标笛卡尔姿态)
            print(f"\n测试 move_j_p: 移动到 X={current_position[0]+0.1}, Y={current_position[1]}, Z={current_position[2]+0.05}，姿态={np.degrees(target_rpy_test)} deg")
            target_pose_jp = np.concatenate((current_position + [0.1, 0, 0.05], target_rpy_test))
            self.move_j_p(target_pose_jp, k_acc=0.5, k_vel=0.5)
            time.sleep(1)

            # 5. 测试 move_l (直线移动到目标笛卡尔姿态)
            print(f"\n测试 move_l: 直线移动到 X={target_pose_jp[0]}, Y={target_pose_jp[1]-0.1}, Z={target_pose_jp[2]-0.05}，保持姿态")
            target_pose_l = target_pose_jp.copy() # 复制上一目标姿态
            target_pose_l[1] -= 0.1 # Y 轴移动 -0.1m
            target_pose_l[2] -= 0.05 # Z 轴移动 -0.05m
            self.move_l(target_pose_l, k_acc=0.3, k_vel=0.15) # 直线运动通常更慢
            time.sleep(1)

            # 6. 测试 move_c (圆弧运动) - 需要谨慎测试
            # print("\n测试 move_c (圆弧运动)...")
            # via_pose_c = target_pose_l.copy()
            # via_pose_c[0] -= 0.05 # 设定一个中间点 X - 0.05
            # via_pose_c[1] -= 0.05 # 设定一个中间点 Y - 0.05
            # target_pose_c = target_pose_jp.copy() # 回到之前的某个点
            # print(f"  经由点 (approx): {via_pose_c[:3]}")
            # print(f"  目标点 (approx): {target_pose_c[:3]}")
            # self.move_c(via_pose_c, target_pose_c, k_acc=0.2, k_vel=0.1)
            # time.sleep(1)


            # 7. 回到 Home 位置
            # print("\n测试完成，回到初始位置...")
            # self.go_home()
            # time.sleep(1)

            print("\n机器人测试序列完成。")

        except socket.error as e:
            print(f"测试过程中发生套接字连接错误: {e}")
        except Exception as e:
            print(f"测试过程中发生未知错误: {e}")
            import traceback
            traceback.print_exc() # 打印详细错误信息
        finally:
            print("测试结束。")


if __name__ == "__main__":
    # --- 配置 ---
    robot_ip = "192.168.1.15" # !!! 修改为你的机器人 IP 地址 !!!
    robot_port = 30003      # UR 实时数据端口 (RTDE 30004, 二次接口 30002)
    # 工作空间限制 (可选, 目前未使用)
    ws_limits = [[-0.8, 0.8], [-0.8, 0.8], [0.05, 0.7]]

    print(f"尝试连接机器人: {robot_ip}:{robot_port}")
    ur_robot = UR_Robot(tcp_host_ip=robot_ip, tcp_port=robot_port, workspace_limits=ws_limits)

    # --- 执行测试 ---
    ur_robot.testRobot()