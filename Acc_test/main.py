# CameraControl.py
import sys
import cv2
import os
import numpy as np
from realsenseD435 import RealsenseD435
from Acc_test.UR_Robot import UR_Robot
import time

class CameraControl:
    def __init__(self):
        self.camera_started = False
        try:
            ##################### 修改以下参数 #############################################            
            # 初始化相机内参和畸变系数
            self.camera_matrix = np.array([
                [904.76290219, 0, 645.31269502],
                [0, 904.70943893, 359.75810675],
                [0, 0, 1]
            ])
            
            self.dist_coeffs = np.array([[0.09775705, -0.06515347, -0.00125012, 0.00001913, -0.38918392]])
            
            # 旋转矩阵
            self.R = np.array([
                [-0.01331479, 0.54377433, -0.83912585],
                [0.99927821, -0.02262319, -0.03051639],
                [-0.03557773, -0.8389265, -0.54308062]
            ])
            # 平移向量,xyz任意方向不准的话在这里进行具体的修改
            self.t = np.array([[1.01345134], [0.009400499], [0.60140605]]) 
            
            self.depth_scale = 0.0010000000474974513
            ##################### 修改完成 ################################################ 
        
            # 初始化相机和机器人
            self.camera = RealsenseD435()
            self.camera_started = True  # 标记相机已启动
            self.robot = UR_Robot()
            
            # 验证机器人连接
            if not self.is_robot_connected():
                raise Exception("机器人连接失败")
                
            # 验证相机连接
            if not self.is_camera_connected():
                raise Exception("相机连接失败")
            
            # 存储当前图像和深度信息
            self.current_color = None
            self.current_depth = None
            
            print("初始化成功完成")
            
        except Exception as e:
            print(f"初始化错误: {str(e)}")
            self.cleanup()
            raise

    def cleanup(self):
        """清理资源"""
        if self.camera_started:
            try:
                self.camera.stop()
                print("相机已停止")
                self.camera_started = False  # 标记相机已停止
            except Exception as e:
                print(f"相机关闭错误: {str(e)}")
        if hasattr(self, 'robot'):
            try:
                self.robot.go_home()
                print("机器人已复位")
            except Exception as e:
                print(f"机器人复位失败: {str(e)}")


    def is_robot_connected(self):
        """检查机器人是否正确连接"""
        try:
            # 使用更基础的连接测试
            state_data = self.robot.get_state()
            return state_data is not None
        except Exception as e:
            print(f"机器人连接检查错误: {str(e)}")
            return False
            
    def is_camera_connected(self):
        """检查相机是否正确连接"""
        try:
            # 获取几帧数据确保相机稳定
            for i in range(3):
                color, depth = self.camera.get_data()
                if color is not None and depth is not None:
                    # 打印图像尺寸以验证
                    print(f"尝试 {i+1}: 彩色图像尺寸: {color.shape}, 深度图像尺寸: {depth.shape}")
                    return True
                else:
                    print(f"尝试 {i+1}: 未获取到有效数据")
                time.sleep(0.2)
            return False
        except Exception as e:
            print(f"相机连接错误: {str(e)}")
            return False
            
    def check_position_reached(self, target_pos, actual_pos, tolerance=0.001):
        """检查是否到达目标位置"""
        try:
            # 输入验证
            if actual_pos is None or target_pos is None:
                print("位置数据为空")
                return False
                
            # 确保数据长度正确
            if len(actual_pos) < 3 or len(target_pos) < 3:
                print("位置数据长度不足")
                return False
                
            # 转换为numpy数组并确保是float类型
            target = np.array(target_pos[:3], dtype=np.float64)
            actual = np.array(actual_pos[:3], dtype=np.float64)
            
            # 检查无效值
            if np.any(np.isnan(target)) or np.any(np.isnan(actual)):
                print("检测到无效位置值")
                return False
                
            if np.any(np.isinf(target)) or np.any(np.isinf(actual)):
                print("检测到无限位置值")
                return False
                
            # 计算欧氏距离
            distance = np.linalg.norm(target - actual)
            
            # 添加调试信息
            print(f"目标位置: {target}")
            print(f"当前位置: {actual}")
            print(f"位置误差: {distance*1000:.2f}mm")
            
            return distance < tolerance
            
        except Exception as e:
            print(f"位置检查错误: {str(e)}")
            return False

    def pixel_to_world(self, pixel_x, pixel_y, depth):
        """将像素坐标转换为世界坐标"""
        try:
            # 像素坐标转换为相机坐标
            x = (pixel_x - self.camera_matrix[0,2]) * depth / self.camera_matrix[0,0]
            y = (pixel_y - self.camera_matrix[1,2]) * depth / self.camera_matrix[1,1]
            z = depth
            
            # 相机坐标转世界坐标
            camera_point = np.array([[x], [y], [z]])
            world_point = np.dot(self.R, camera_point) + self.t
            
            return world_point.flatten()
        except Exception as e:
            print(f"坐标转换错误: {str(e)}")
            return None
    
    def safe_robot_move(self, target_pose, movement_type='l', k_acc=1, k_vel=1):
        """安全的机器人运动控制"""
        try:
            # 确保target_pose是正确的格式
            target_pose = [float(x) for x in target_pose]
            print("目标位置:", target_pose)
            
            # 如果带有夹爪
            target_pose[2] = target_pose[2] + 0.15
            # 执行移动
            if movement_type == 'l':
                self.robot.move_l(target_pose, k_acc=k_acc, k_vel=k_vel)
            elif movement_type == 'j':
                # 直接传递位姿，不做任何嵌套
                self.robot.move_j_p(target_pose, k_acc=k_acc, k_vel=k_vel)
            
            # 等待移动完成
            time.sleep(1.0)
            
            # 验证移动结果
            current_pos = self.get_robot_position()
            if current_pos is not None:
                # 计算位置误差
                target_array = np.array(target_pose[:3])
                current_array = np.array(current_pos[:3])
                distance = np.linalg.norm(target_array - current_array)
                print(f"位置误差: {distance*1000:.2f}mm")
                
                if distance > 0.001:
                    print("警告: 未达到期望精度")
                
            return True
            
        except Exception as e:
            print(f"机器人移动错误: {str(e)}")
            return False

    def get_robot_position(self):
        """获取当前机器人位置"""
        try:
            state_data = self.robot.get_state()
            if not state_data:
                print("无法获取机器人状态数据")
                return None
                
            # 添加调试信息
            print("状态数据大小:", len(state_data) if state_data else "None")
            
            if state_data and len(state_data) >= 48:  # 确保数据长度足够
                tool_positions = self.robot.parse_tcp_state_data(state_data, 'cartesian_info')
                if tool_positions is not None:
                    return [float(x) for x in tool_positions]
            return None
        except Exception as e:
            print(f"获取位置错误: {str(e)}")
            return None

    def emergency_stop(self):
        """紧急停止机器人"""
        try:
            self.robot.stop()
            time.sleep(0.5)  # 等待停止命令执行
            print("机器人已紧急停止")
        except Exception as e:
            print(f"紧急停止失败: {str(e)}")

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标点击事件处理"""
        if event == cv2.EVENT_LBUTTONDOWN:
            try:
                if self.current_depth is not None:
                    depth = self.current_depth[y, x] * self.depth_scale
                    if depth > 0:
                        # 计算目标世界坐标
                        target_coord = self.pixel_to_world(x, y, depth)
                        if target_coord is None:
                            print("无法计算世界坐标")
                            return
                            
                        print(f"\n目标坐标: X={target_coord[0]:.4f}, Y={target_coord[1]:.4f}, Z={target_coord[2]:.4f}")
                        
                        # 构造目标姿态 - 以列表形式而不是嵌套列表
                        target_pose = [target_coord[0], target_coord[1], target_coord[2], -np.pi, 0, 0]
                        pre_pose = [target_coord[0], target_coord[1], target_coord[2] + 0.1, -np.pi, 0, 0]
                        
                        # 移动顺序
                        time.sleep(0.5)
                        print("移动到预位置...")
                        if not self.safe_robot_move(pre_pose, 'l', k_acc=0.4, k_vel=0.4):
                            print("移动到预位置失败")
                            return
                            
                        time.sleep(2)
                        print("移动到目标位置...")
                        if not self.safe_robot_move(target_pose, 'l', k_acc=0.4, k_vel=0.4):
                            print("移动到目标位置失败")
                            return
                            
                        time.sleep(2)
                        print("返回预位置...")
                        if not self.safe_robot_move(pre_pose, 'l', k_acc=0.4, k_vel=0.4):
                            print("返回预位置失败")
                            return
                            
                        print("动作完成，可以选取下一个位置点")
                        
            except Exception as e:
                print(f"操作失败: {str(e)}")
    
    def run(self):
        """主运行循环"""
        cv2.namedWindow('Camera View')
        cv2.setMouseCallback('Camera View', self.mouse_callback)
        
        print("开始相机控制。点击图像以移动机器人到目标位置。")
        print("按'q'退出。")
        
        try:
            while True:
                # 获取相机图像
                try:
                    self.current_color, self.current_depth = self.camera.get_data()
                    if self.current_color is None or self.current_depth is None:
                        print("获取相机数据失败")
                        continue
                    
                    # 显示图像
                    cv2.imshow('Camera View', self.current_color)
                    
                except Exception as e:
                    print(f"相机错误: {str(e)}")
                    continue
                
                # 按'q'退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        except Exception as e:
            print(f"运行时错误: {str(e)}")
        finally:
            print("清理...")
            self.cleanup()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    controller = None
    try:
        controller = CameraControl()
        controller.run()
    except Exception as e:
        print(f"程序失败: {str(e)}")
    finally:
        if controller:
            try:
                cv2.destroyAllWindows()
            except Exception as cleanup_e:
                print(f"清理资源时出错: {str(cleanup_e)}")

