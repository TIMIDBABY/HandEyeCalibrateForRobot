"""
UR机械臂控制(基于UR5官方库：rtde) - 在环境中输入：pip install ur-rtde

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

import rtde_control
import rtde_receive
import numpy as np
import time
import atexit

ur_ip = "192.168.1.43"     #在这里替换为你的UR5 IP地址
speed = 0.5                #安全值：0.5
acc   = 0.5                #安全值：0.5

class UR_Robot_Enhanced:
    def __init__(self, robot_ip = ur_ip):
        self.robot_ip = robot_ip
        self.rtde_c = None
        self.rtde_r = None
        self.is_connected = False
        
        # 注册退出时的清理函数
        atexit.register(self.cleanup)
        
        self.connect()
    
    def connect(self):
        """连接机器人"""
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            
            if self.rtde_c.isConnected() and self.rtde_r.isConnected():
                self.is_connected = True
                print(f"✅ 成功连接到机器人: {self.robot_ip}")
            else:
                raise Exception("连接验证失败")
                
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            self.cleanup()
            raise
    
    def cleanup(self):
        """清理资源并断开连接"""
        try:
            if self.is_connected:
                print("🧹 正在清理资源...")
                if self.rtde_c:
                    self.rtde_c.stopJ(2.0)  # 停止机器人
                    self.rtde_c.disconnect()
                if self.rtde_r:
                    self.rtde_r.disconnect()
                self.is_connected = False
                print("✅ 资源清理完成")
        except Exception as e:
            print(f"⚠️ 清理资源时出现异常: {e}")
    
    def _safe_execute(self, func, *args, **kwargs):
        """安全执行函数，出错时自动清理资源"""
        try:
            if not self.is_connected:
                raise Exception("机器人未连接")
            return func(*args, **kwargs)
        except Exception as e:
            print(f"❌ 执行异常: {e}")
            self.cleanup()
            raise
    
    # ==================== 位置获取方法 ====================
    
    def get_current_joints(self):
        """获取当前关节角度（度）"""
        def _get_joints():
            joint_radians = self.rtde_r.getActualQ()
            joint_degrees = [np.rad2deg(rad) for rad in joint_radians]
            print(f"🎯 当前关节角度: {[f'{deg:.2f}°' for deg in joint_degrees]}")
            return joint_degrees
        
        return self._safe_execute(_get_joints)
    
    def get_current_pose(self):
        """获取当前TCP位姿"""
        def _get_pose():
            current_pose = self.rtde_r.getActualTCPPose()
            print(f"📍 当前TCP位姿: {[f'{pos:.3f}' for pos in current_pose]}")
            return current_pose
        
        return self._safe_execute(_get_pose)
    
    def get_current_position(self):
        """获取当前位置信息（关节+TCP）"""
        joints = self.get_current_joints()
        pose = self.get_current_pose()
        return {
            'joints': joints,
            'tcp_pose': pose,
            'position': pose[:3],  # x, y, z
            'orientation': pose[3:]  # rx, ry, rz
        }
    
    # ==================== 基础运动方法 ====================
    
    def move_j(self, joint_degrees, speed=speed, acceleration=acc):
        """
        关节空间运动 - 直接输入度数
        joint_degrees: [基座, 肩部, 肘部, 手腕1, 手腕2, 手腕3] 6个关节角度(度)
        """
        def _move_j():
            # 将度数转换为弧度
            joint_radians = [np.deg2rad(deg) for deg in joint_degrees]
            print(f"🎯 关节运动到: {joint_degrees} 度")
            result = self.rtde_c.moveJ(joint_radians, speed, acceleration)
            
            if result:
                print("✅ 运动命令发送成功")
                return True
            else:
                print("❌ 运动命令失败")
                return False
        
        return self._safe_execute(_move_j)
    
    def move_l(self, tcp_pose, speed=speed, acceleration=acc):
        """
        直线运动 - TCP末端沿直线移动
        tcp_pose: [x, y, z, rx, ry, rz] 位置(米)和姿态(弧度)
        """
        def _move_l():
            print(f"📍 直线运动到: {[f'{pos:.3f}' for pos in tcp_pose[:3]]}")
            result = self.rtde_c.moveL(tcp_pose, speed, acceleration)
            
            if result:
                print("✅ 运动命令发送成功")
                return True
            else:
                print("❌ 运动命令失败")
                return False
        
        return self._safe_execute(_move_l)
    
    def move_c(self, via_pose, to_pose, speed=speed, acceleration=acc):
        """
        圆弧运动 - TCP末端沿圆弧移动
        via_pose: [x, y, z, rx, ry, rz] 中间点位置和姿态
        to_pose: [x, y, z, rx, ry, rz] 终点位置和姿态
        """
        def _move_c():
            print(f"🌀 圆弧运动: 经过 {[f'{pos:.3f}' for pos in via_pose[:3]]} 到达 {[f'{pos:.3f}' for pos in to_pose[:3]]}")
            result = self.rtde_c.moveC(via_pose, to_pose, speed, acceleration)
            
            if result:
                print("✅ 运动命令发送成功")
                return True
            else:
                print("❌ 运动命令失败")
                return False
        
        return self._safe_execute(_move_c)
    
    # ==================== 单轴控制方法 ====================
    
    def move_x(self, delta_x, speed=speed, acceleration=acc):
        """沿X轴移动指定距离（米）"""
        def _move_x():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            target_pose[0] += delta_x
            print(f"➡️ 沿X轴移动 {delta_x:.3f}m")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_move_x)
    
    def move_y(self, delta_y, speed=speed, acceleration=acc):
        """沿Y轴移动指定距离（米）"""
        def _move_y():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            target_pose[1] += delta_y
            print(f"⬆️ 沿Y轴移动 {delta_y:.3f}m")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_move_y)
    
    def move_z(self, delta_z, speed=speed, acceleration=acc):
        """沿Z轴移动指定距离（米）"""
        def _move_z():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            target_pose[2] += delta_z
            print(f"⬆️ 沿Z轴移动 {delta_z:.3f}m")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_move_z)
    
    def rotate_rx(self, delta_rx, speed=speed, acceleration=acc):
        """绕X轴旋转指定角度（弧度）"""
        def _rotate_rx():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            target_pose[3] += delta_rx
            print(f"🔄 绕X轴旋转 {np.rad2deg(delta_rx):.2f}°")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_rotate_rx)
    
    def rotate_ry(self, delta_ry, speed=speed, acceleration=acc):
        """绕Y轴旋转指定角度（弧度）"""
        def _rotate_ry():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            target_pose[4] += delta_ry
            print(f"🔄 绕Y轴旋转 {np.rad2deg(delta_ry):.2f}°")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_rotate_ry)
    
    def rotate_rz(self, delta_rz, speed=speed, acceleration=acc):
        """绕Z轴旋转指定角度（弧度）"""
        def _rotate_rz():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            target_pose[5] += delta_rz
            print(f"🔄 绕Z轴旋转 {np.rad2deg(delta_rz):.2f}°")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_rotate_rz)
    
    # ==================== 便捷方法 ====================
    
    def move_to_position(self, x=None, y=None, z=None, rx=None, ry=None, rz=None, speed=speed, acceleration=acc):
        """移动到指定位置（只更新指定的轴）"""
        def _move_to_position():
            current_pose = self.rtde_r.getActualTCPPose()
            target_pose = current_pose.copy()
            
            if x is not None: target_pose[0] = x
            if y is not None: target_pose[1] = y
            if z is not None: target_pose[2] = z
            if rx is not None: target_pose[3] = rx
            if ry is not None: target_pose[4] = ry
            if rz is not None: target_pose[5] = rz
            
            changes = []
            if x is not None: changes.append(f"x={x:.3f}")
            if y is not None: changes.append(f"y={y:.3f}")
            if z is not None: changes.append(f"z={z:.3f}")
            if rx is not None: changes.append(f"rx={np.rad2deg(rx):.2f}°")
            if ry is not None: changes.append(f"ry={np.rad2deg(ry):.2f}°")
            if rz is not None: changes.append(f"rz={np.rad2deg(rz):.2f}°")
            
            print(f"🎯 移动到指定位置: {', '.join(changes)}")
            return self.rtde_c.moveL(target_pose, speed, acceleration)
        
        return self._safe_execute(_move_to_position)
    
    def go_home(self):
        """回到初始位置"""
        home_joints = [0, -70, 70, -90, -90, 0]
        print("🏠 回到初始位置...")
        return self.move_j(home_joints, speed=speed, acceleration=acc)
    
    def stop(self):
        """停止机器人"""
        def _stop():
            print("🛑 停止机器人")
            return self.rtde_c.stopJ(5.0)
        
        return self._safe_execute(_stop)
    
    def disconnect(self):
        """手动断开连接"""
        self.cleanup()

# 使用示例
def main():
    robot = None
    try:
        # 创建机器人实例并连接
        robot = UR_Robot_Enhanced(ur_ip)  # 在最顶部修改机器人ip
        
        while True:
            # 获取当前位置信息
            # current_info = robot.get_current_position()
            # print(f"当前位置信息: {current_info}")
            
            # 回到初始位置
            robot.go_home()

            # 1. 关节运动示例
            robot.move_j([-45, -70, 70, -90, -90, 0])
            
            # # 2. 单轴控制示例
            # robot.move_x(0.1)     # 沿X轴移动10cm
            # time.sleep(0.5)
            # robot.move_x(-0.1)    # 沿X轴移动10cm
            # time.sleep(0.5)
            # robot.move_y(0.1)     # 沿Y轴移动-10cm
            # time.sleep(0.5)
            # robot.move_y(-0.2)    # 沿Y轴移动-20cm
            # time.sleep(0.5)
            # robot.move_z(0.1)     # 沿Z轴移动10cm
            # time.sleep(0.5)
            # robot.move_z(-0.2)    # 沿Z轴移动-20cm
            # time.sleep(0.5)
            # robot.go_home()
            # time.sleep(3)
            
            # # 3. 旋转控制示例
            # robot.rotate_rz(np.deg2rad(45))  # 绕Z轴旋转45度
            # time.sleep(1)
            
            # # 4. 移动到指定位置示例
            # robot.move_to_position(x=0.3, z=0.4)  # 只更新x和z坐标
            # time.sleep(2)
            
            # # 5. 获取当前关节角度
            # joints = robot.get_current_joints()
            # print(f"当前关节角度: {joints}")
            
            # # 6. 获取当前TCP位姿
            pose = robot.get_current_pose()
            print(f"当前TCP位姿: {pose}")

    except Exception as e:
        print(f"❌ 程序执行失败: {e}")
        
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断程序")
    except Exception as e:
        print(f"❌ 程序执行失败: {e}")
    finally:
        if robot:
            robot.disconnect()

if __name__ == "__main__":
    main()