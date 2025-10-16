"""
本代码适用于windows下懒得配置rtde环境的用户，通过tcp直接控制UR5
并新增自动检测连接功能
使用前请注意当前TCP设置！！！

三种运动方式说明：
1. move_j (关节运动): 每个关节独立运动到目标角度，末端轨迹不可预测
2. move_l (直线运动): 末端TCP沿直线从当前位置移动到目标位置
3. move_c (圆弧运动): 末端TCP沿圆弧从当前位置经过中间点到达目标位置，尽量别用！！！

6个关节含义 [单位：度]:
- Joint 1 (基座): 整个机械臂绕底座旋转 (±360°)
- Joint 2 (肩部): 第一臂段的抬升角度 (±360°)
- Joint 3 (肘部): 第二臂段相对第一臂段的弯曲角度 (±360°)
- Joint 4 (手腕1): 手腕绕臂轴旋转 (±360°)
- Joint 5 (手腕2): 手腕俯仰 (±360°)
- Joint 6 (手腕3): 末端执行器旋转 (±360°)

TCP位姿含义 [位置单位：米，姿态单位：弧度]:
- x, y, z: 末端相对基座的空间位置
- rx, ry, rz: 绕x/y/z轴的旋转角度(轴角表示法)
"""

# coding=utf8
import time
import socket
import struct
import numpy as np
import math

robot_ip = "192.168.1.99"  # 替换为实际的机器人IP地址


class UR_Robot:
    def __init__(self, tcp_host_ip=robot_ip, tcp_port=30003, workspace_limits=None):
        # 初始化变量
        if workspace_limits is None:
            workspace_limits = workspace_limits = [
                [-0.85, 0.85],  # X: Full theoretical reach
                [-0.85, 0.85],  # Y: Full theoretical reach
                [-0.85, 0.85],  # Z: Extended vertical range
            ]
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        self.workspace_limits = workspace_limits

        # UR5机器人配置参数
        self.joint_acc = 1.4
        self.joint_vel = 1.05
        self.joint_tolerance = 0.01
        self.tool_acc = 1.2
        self.tool_vel = 0.25
        self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        # 测试机器人连接
        print("UR5 Robot initialized successfully!")
        self.testRobot()

    def testRobot(self):
        """测试机器人连接"""
        try:
            print("Testing robot connection...")
            # 先尝试获取状态，而不是移动
            state_data = self.get_state()
            if state_data and len(state_data) > 0:
                print(f"Received data length: {len(state_data)} bytes")
                joint_angles = self.get_current_joint_angles()
                print(f"Current joint angles (degrees): {joint_angles}")
                print("Robot connection test passed!\n")
            else:
                print("Warning: No data received from robot\n")
        except Exception as e:
            print(
                f"Test failed! Please check the IP address or robot connection: {e}\n"
            )
            import traceback

            traceback.print_exc()

    def get_state(self):
        """获取机器人当前状态数据 - 改进版本"""
        try:
            self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_socket.settimeout(5.0)  # 设置5秒超时
            self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))

            # 接收完整的数据包
            data = b""
            while len(data) < 1060:  # UR机器人标准数据包大小约1060字节
                packet = self.tcp_socket.recv(2048)
                if not packet:
                    break
                data += packet
                if len(data) >= 1060:
                    break

            self.tcp_socket.close()

            if len(data) < 100:  # 数据太短，可能有问题
                print(f"Warning: Received data is too short ({len(data)} bytes)")
                return None

            return data

        except socket.timeout:
            print("Error: Connection timeout")
            return None
        except Exception as e:
            print(f"Error getting state: {e}")
            return None

    def parse_tcp_state_data(self, data, subpackage):
        """
        解析来自机器人的TCP状态数据 - 改进版本
        """
        if data is None or len(data) == 0:
            print("Error: No data to parse")
            return None

        try:
            # UR机器人实时数据接口的数据包结构
            dic = {
                "MessageSize": "i",
                "Time": "d",
                "q target": "6d",
                "qd target": "6d",
                "qdd target": "6d",
                "I target": "6d",
                "M target": "6d",
                "q actual": "6d",
                "qd actual": "6d",
                "I actual": "6d",
                "I control": "6d",
                "Tool vector actual": "6d",
                "TCP speed actual": "6d",
                "TCP force": "6d",
                "Tool vector target": "6d",
                "TCP speed target": "6d",
                "Digital input bits": "d",
                "Motor temperatures": "6d",
                "Controller Timer": "d",
                "Test value": "d",
                "Robot Mode": "d",
                "Joint Modes": "6d",
                "Safety Mode": "d",
                "empty1": "6d",
                "Tool Accelerometer values": "3d",
                "empty2": "6d",
                "Speed scaling": "d",
                "Linear momentum norm": "d",
                "SoftwareOnly": "d",
                "softwareOnly2": "d",
                "V main": "d",
                "V robot": "d",
                "I robot": "d",
                "V actual": "6d",
                "Digital outputs": "d",
                "Program state": "d",
            }

            for key in dic:
                fmtsize = struct.calcsize(dic[key])

                # 检查是否有足够的数据
                if len(data) < fmtsize:
                    print(
                        f"Warning: Not enough data for {key}. Need {fmtsize}, have {len(data)}"
                    )
                    break

                data1, data = data[0:fmtsize], data[fmtsize:]
                fmt = "!" + dic[key]

                try:
                    dic[key] = dic[key], struct.unpack(fmt, data1)
                except struct.error as e:
                    print(f"Error unpacking {key}: {e}")
                    print(f"Expected {fmtsize} bytes, got {len(data1)} bytes")
                    return None

            if subpackage == "joint_data":
                if "q actual" in dic and isinstance(dic["q actual"], tuple):
                    q_actual_tuple = dic["q actual"]
                    joint_data = np.array(q_actual_tuple[1])
                    # 检查NaN值
                    if np.any(np.isnan(joint_data)):
                        print("Warning: NaN values detected in joint data")
                        return None
                    return joint_data
                else:
                    print("Error: Could not extract joint data")
                    return None

            elif subpackage == "cartesian_info":
                if "Tool vector actual" in dic and isinstance(
                    dic["Tool vector actual"], tuple
                ):
                    Tool_vector_actual = dic["Tool vector actual"]
                    cartesian_info = np.array(Tool_vector_actual[1])
                    # 检查NaN值
                    if np.any(np.isnan(cartesian_info)):
                        print("Warning: NaN values detected in cartesian info")
                        return None
                    return cartesian_info
                else:
                    print("Error: Could not extract cartesian info")
                    return None

        except Exception as e:
            print(f"Error parsing TCP state data: {e}")
            import traceback

            traceback.print_exc()
            return None

    def get_current_joint_angles(self):
        """获取当前关节角度（角度制）"""
        state_data = self.get_state()
        if state_data is None:
            return None
        joint_angles_radians = self.parse_tcp_state_data(state_data, "joint_data")
        if joint_angles_radians is None:
            return None
        return [angle * 180.0 / np.pi for angle in joint_angles_radians]

    def get_current_joint_angles_rad(self):
        """获取当前关节角度（弧度制）"""
        state_data = self.get_state()
        if state_data is None:
            return None
        return self.parse_tcp_state_data(state_data, "joint_data")

    def get_current_tool_pose(self):
        """获取当前工具位姿 [x, y, z, rx, ry, rz]"""
        state_data = self.get_state()
        if state_data is None:
            return None
        return self.parse_tcp_state_data(state_data, "cartesian_info")

    def move_j(self, joint_configuration_degrees, k_acc=1, k_vel=1, t=0, r=0):
        """通过指定关节角度移动机器人关节（角度制输入）"""
        joint_configuration_radians = [
            deg * np.pi / 180.0 for deg in joint_configuration_degrees
        ]

        # 建立一个专门用于发送指令的socket
        cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_socket.connect((self.tcp_host_ip, self.tcp_port))

        tcp_command = "movej([%f" % joint_configuration_radians[0]
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + (",%f" % joint_configuration_radians[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (
            k_acc * self.joint_acc,
            k_vel * self.joint_vel,
        )
        cmd_socket.send(str.encode(tcp_command))
        cmd_socket.close()  # 发送后即可关闭

        # 循环检查机器人是否到达目标位置
        while True:
            actual_joint_positions_rad = self.get_current_joint_angles_rad()

            if actual_joint_positions_rad is None:
                print("Warning: Could not get actual joint positions, retrying...")
                time.sleep(0.1)
                continue

            # 检查是否所有关节都已到达目标位置
            all_joints_close = True
            for j in range(6):
                if not (
                    np.abs(
                        actual_joint_positions_rad[j] - joint_configuration_radians[j]
                    )
                    < self.joint_tolerance
                ):
                    all_joints_close = False
                    break

            if all_joints_close:
                break

            time.sleep(0.01)

    def move_l(self, tool_configuration, k_acc=1, k_vel=1, t=0, r=0):
        """直线运动到指定工具位姿 - 修复NaN值问题"""
        # 建立socket
        cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_socket.connect((self.tcp_host_ip, self.tcp_port))

        tcp_command = "movel(p[%f" % tool_configuration[0]
        for joint_idx in range(1, 6):
            tcp_command = tcp_command + (",%f" % tool_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (
            k_acc * self.tool_acc,
            k_vel * self.tool_vel,
        )
        cmd_socket.send(str.encode(tcp_command))
        cmd_socket.close()  # 发送后即可关闭

        while True:
            # 获取最新状态
            actual_tool_positions = self.get_current_tool_pose()

            if actual_tool_positions is None:
                print("Warning: Could not get actual tool positions, retrying...")
                time.sleep(0.1)  # 如果获取失败，稍等后重试
                continue

            # 检查是否有NaN值
            if np.any(np.isnan(actual_tool_positions)) or np.any(
                np.isnan(tool_configuration)
            ):
                print(
                    "Warning: NaN values detected in position comparison, retrying..."
                )
                time.sleep(0.1)
                continue

            # 检查是否到达目标位置
            position_differences = []
            all_axes_close = True
            for j in range(6):
                diff = np.abs(actual_tool_positions[j] - tool_configuration[j])
                if not (diff < self.tool_pose_tolerance[j]):
                    all_axes_close = False
                    break

            if all_axes_close:
                break

            time.sleep(0.01)

    def move_z(self, distance, k_acc=1, k_vel=1):
        """沿Z轴移动指定距离"""
        current_pose = self.get_current_tool_pose()
        if current_pose is None:
            print("Error: Could not get current pose")
            return False

        if np.any(np.isnan(current_pose)):
            print("Error: Current pose contains NaN values")
            return False

        target_pose = current_pose.copy()
        target_pose[2] += distance

        if not self._check_workspace_limits(target_pose):
            print(
                f"Warning: Target position [{target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f}] is outside workspace limits!"
            )
            return False

        self.move_l(target_pose, k_acc, k_vel)
        return True

    def move_x(self, distance, k_acc=1, k_vel=1):
        """沿X轴移动指定距离"""
        current_pose = self.get_current_tool_pose()
        if current_pose is None:
            print("Error: Could not get current pose")
            return False

        if np.any(np.isnan(current_pose)):
            print("Error: Current pose contains NaN values")
            return False

        target_pose = current_pose.copy()
        target_pose[0] += distance

        if not self._check_workspace_limits(target_pose):
            print(
                f"Warning: Target position [{target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f}] is outside workspace limits!"
            )
            return False

        self.move_l(target_pose, k_acc, k_vel)
        return True

    def move_y(self, distance, k_acc=1, k_vel=1):
        """沿Y轴移动指定距离"""
        current_pose = self.get_current_tool_pose()
        if current_pose is None:
            print("Error: Could not get current pose")
            return False

        if np.any(np.isnan(current_pose)):
            print("Error: Current pose contains NaN values")
            return False

        target_pose = current_pose.copy()
        target_pose[1] += distance

        if not self._check_workspace_limits(target_pose):
            print(
                f"Warning: Target position [{target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f}] is outside workspace limits!"
            )
            return False

        self.move_l(target_pose, k_acc, k_vel)
        return True

    def _check_workspace_limits(self, pose):
        """检查目标位姿是否在工作空间限制内"""
        # 先检查是否有NaN值
        if np.any(np.isnan(pose)):
            print("Error: Pose contains NaN values")
            return False

        x, y, z = pose[0], pose[1], pose[2]

        if x < self.workspace_limits[0][0] or x > self.workspace_limits[0][1]:
            return False
        if y < self.workspace_limits[1][0] or y > self.workspace_limits[1][1]:
            return False
        if z < self.workspace_limits[2][0] or z > self.workspace_limits[2][1]:
            return False

        return True


if __name__ == "__main__":
    print("Initializing UR Robot...")
    robot = UR_Robot()
    # robot.move_j([70, -90, 90, 0, 90, 0])
    # robot.move_j([90, -90, 90, 0, 90, 0])
    robot.move_z(0.05)
    robot.move_z(-0.05)
    robot.move_x(0.05)
    robot.move_x(-0.05)
    robot.move_y(0.05)
    robot.move_y(-0.05)
    print("Test completed.")
