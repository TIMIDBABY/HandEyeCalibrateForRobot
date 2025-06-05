"""
注意：标定所用画面大小与测试所用画面大小需相同！否则测试结果不对！
"""
import sys
import cv2
import os
import numpy as np
from realsenseD435 import RealsenseD435
from arm_control import UR_Robot_Enhanced
import time
import threading
from queue import Queue

class EnhancedCameraControl:
    def __init__(self):
        self.camera_started = False
        self.robot_connected = False
        self.operation_queue = Queue()
        self.is_operating = False
        
        try:
            print("🚀 开始...")
            
            ##################### 相机标定参数 #############################################            
            # 相机内参矩阵
            self.camera_matrix = np.array([
                [603.8661115132229, 0.0, 322.25827084613934],
                [0.0, 603.8818543467364, 236.16126531486586],
                [0.0, 0.0, 1.0]
            ])
            
            # 畸变系数
            self.dist_coeffs = np.array([
                [0.034104320460288245, 0.6050822548146084, -0.0006508861079110671, -0.00020757037799296792, -2.4776884755100372]
            ])
            
            # 手眼标定 - 旋转矩阵 (相机坐标系到机器人坐标系)
            self.R = np.array([
                [-0.006171194234242838, -0.8972091938379405, 0.44156265563835473],
                [-0.9998940261561149, -0.00028589296443715106, -0.014555230074707659],
                [0.01318532589805943, -0.4416056846983357, -0.8971123488298867]
            ])
            
            # 平移向量 (相机坐标系到机器人坐标系)
            self.t = np.array([
                [-0.8276189034282344],
                [-0.09056830563701516],
                [0.95025828299934]
            ]) 
            
            # 深度缩放因子
            self.depth_scale = 0.0010000000474974513
            
            # 运动参数配置
            self.approach_height = 0.08  # 接近高度 (8cm)
            self.move_speed = 0.3        # 运动速度
            self.move_acc = 0.3          # 运动加速度
            
            ##################### 系统初始化 ################################################ 
            
            # 初始化相机
            print("📷 正在初始化RealSense相机...")
            self.camera = RealsenseD435()
            self.camera_started = True
            print("✅ 相机初始化成功")
            
            # 初始化机器人
            print("🤖 正在连接UR机器人...")
            self.robot = UR_Robot_Enhanced()
            self.robot_connected = True
            print("✅ 机器人连接成功")
            
            # 验证系统连接
            self._verify_connections()
            
            # 初始化图像存储
            self.current_color = None
            self.current_depth = None
            
            # 控制模式
            self.control_mode = "normal"  # normal, precise, test
            
            print("🎉 系统初始化完成！")
            print("💡 使用说明:")
            print("   - 左键点击: 移动到目标位置")
            print("   - 右键点击: 精确定位模式")
            print("   - 键盘 'h': 回到初始位置")
            print("   - 键盘 's': 紧急停止")
            print("   - 键盘 'q': 退出程序")
            
        except Exception as e:
            print(f"❌ 初始化失败: {str(e)}")
            self.cleanup()
            raise

    def _verify_connections(self):
        """验证所有连接"""
        print("🔍 验证系统连接...")
        
        # 验证相机连接
        if not self._test_camera():
            raise Exception("相机连接验证失败")
        
        # 验证机器人连接
        if not self._test_robot():
            raise Exception("机器人连接验证失败")
        
        print("✅ 所有连接验证通过")

    def _test_camera(self):
        """测试相机连接"""
        try:
            for i in range(3):
                color, depth = self.camera.get_data()
                if color is not None and depth is not None:
                    print(f"📷 相机测试通过 - 图像尺寸: {color.shape}")
                    return True
                time.sleep(0.1)
            return False
        except Exception as e:
            print(f"📷 相机测试失败: {e}")
            return False

    def _test_robot(self):
        """测试机器人连接"""
        try:
            if self.robot.is_connected:
                current_pose = self.robot.get_current_pose()
                if current_pose is not None:
                    print("🤖 机器人测试通过")
                    return True
            return False
        except Exception as e:
            print(f"🤖 机器人测试失败: {e}")
            return False

    def cleanup(self):
        """清理所有资源"""
        print("🧹 正在清理系统资源...")
        
        # 停止相机
        if self.camera_started:
            try:
                self.camera.stop()
                self.camera_started = False
                print("📷 相机已停止")
            except Exception as e:
                print(f"📷 相机停止失败: {e}")
        
        # 断开机器人连接
        if self.robot_connected:
            try:
                # 机器人回到安全位置
                print("🏠 机器人回到安全位置...")
                self.robot.go_home()
                time.sleep(1)
                
                # 断开连接
                self.robot.disconnect()
                self.robot_connected = False
                print("🤖 机器人连接已断开")
            except Exception as e:
                print(f"🤖 机器人断开失败: {e}")
        
        print("✅ 资源清理完成")

    def pixel_to_world(self, pixel_x, pixel_y, depth):
        """像素坐标转世界坐标"""
        try:
            # 像素坐标转相机坐标
            x = (pixel_x - self.camera_matrix[0,2]) * depth / self.camera_matrix[0,0]
            y = (pixel_y - self.camera_matrix[1,2]) * depth / self.camera_matrix[1,1]
            z = depth
            
            # 相机坐标转世界坐标
            camera_point = np.array([[x], [y], [z]])
            world_point = np.dot(self.R, camera_point) + self.t
            
            return world_point.flatten()
        except Exception as e:
            print(f"❌ 坐标转换失败: {e}")
            return None

    def safe_robot_move(self, target_pose, movement_type='l'):
        """安全的机器人运动控制"""
        try:
            if not self.robot_connected:
                print("❌ 机器人未连接")
                return False
            
            print(f"🎯 目标位置: {[f'{pos:.3f}' for pos in target_pose[:3]]}")
            
            # 执行运动
            if movement_type == 'l':
                result = self.robot.move_l(target_pose, self.move_speed, self.move_acc)
            elif movement_type == 'j':
                result = self.robot.move_j_p(target_pose, self.move_speed, self.move_acc)
            else:
                print(f"❌ 不支持的运动类型: {movement_type}")
                return False
            
            if result:
                print("✅ 运动完成")
                # 验证位置精度
                current_pose = self.robot.get_current_pose()
                if current_pose:
                    distance = np.linalg.norm(np.array(target_pose[:3]) - np.array(current_pose[:3]))
                    print(f"📊 位置精度: {distance*1000:.2f}mm")
                return True
            else:
                print("❌ 运动失败")
                return False
                
        except Exception as e:
            print(f"❌ 机器人运动异常: {e}")
            return False

    def execute_pick_and_place(self, world_coord):
        """执行抓取动作序列"""
        try:
            print(f"\n🎯 开始执行抓取序列...")
            print(f"📍 目标坐标: X={world_coord[0]:.3f}, Y={world_coord[1]:.3f}, Z={world_coord[2]:.3f}")
            
            # 构造运动位姿 (保持末端向下的姿态)
            target_pose = [world_coord[0], world_coord[1], world_coord[2], -np.pi, 0, 0]
            approach_pose = [world_coord[0], world_coord[1], world_coord[2] + self.approach_height, -np.pi, 0, 0]
            
            # 步骤1: 移动到接近位置
            print("1️⃣ 移动到接近位置...")
            if not self.safe_robot_move(approach_pose, 'l'):
                print("❌ 移动到接近位置失败")
                return False
            time.sleep(0.5)
            
            # 步骤2: 下降到目标位置
            print("2️⃣ 下降到目标位置...")
            if not self.safe_robot_move(target_pose, 'l'):
                print("❌ 下降到目标位置失败")
                return False
            time.sleep(1.0)
            
            # 步骤3: 在目标位置停留
            print("3️⃣ 在目标位置停留...")
            time.sleep(1.0)
            
            # 步骤4: 抬升到安全位置
            print("4️⃣ 抬升到安全位置...")
            if not self.safe_robot_move(approach_pose, 'l'):
                print("❌ 抬升失败")
                return False
            
            print("✅ 抓取序列完成")
            return True
            
        except Exception as e:
            print(f"❌ 抓取序列执行失败: {e}")
            return False

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        if self.is_operating:
            print("⚠️ 操作进行中，请等待...")
            return
            
        if event == cv2.EVENT_LBUTTONDOWN:
            # 左键 - 普通抓取模式
            self._handle_click(x, y, "normal")
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            # 右键 - 精确模式
            self._handle_click(x, y, "precise")

    def _handle_click(self, x, y, mode="normal"):
        """处理鼠标点击事件"""
        try:
            if self.current_depth is None:
                print("❌ 深度图像不可用")
                return
            
            # 获取深度值
            depth = self.current_depth[y, x] * self.depth_scale
            if depth <= 0:
                print("❌ 无效的深度值")
                return
            
            print(f"\n{'🎯' if mode == 'normal' else '🔍'} {mode.upper()}模式 - 像素位置: ({x}, {y}), 深度: {depth:.3f}m")
            
            # 转换为世界坐标
            world_coord = self.pixel_to_world(x, y, depth)
            if world_coord is None:
                print("❌ 坐标转换失败")
                return
            
            # 启动操作线程
            self.is_operating = True
            operation_thread = threading.Thread(
                target=self._execute_operation,
                args=(world_coord, mode),
                daemon=True
            )
            operation_thread.start()
            
        except Exception as e:
            print(f"❌ 点击处理失败: {e}")
            self.is_operating = False

    def _execute_operation(self, world_coord, mode):
        """在后台线程中执行操作"""
        try:
            if mode == "precise":
                # 精确模式 - 分步移动
                print("🔍 精确模式: 分步移动")
                self._precise_move(world_coord)
            else:
                # 普通模式 - 直接抓取
                print("🎯 普通模式: 直接抓取")
                self.execute_pick_and_place(world_coord)
                
        except Exception as e:
            print(f"❌ 操作执行失败: {e}")
        finally:
            self.is_operating = False
            print("✅ 操作完成，可以继续点击")

    def _precise_move(self, world_coord):
        """精确移动模式"""
        try:
            print("🔍 精确移动序列:")
            current_pose = self.robot.get_current_pose()
            if not current_pose:
                print("❌ 无法获取当前位置")
                return
            
            # 分步移动到目标位置
            steps = [
                ("XY平面移动", [world_coord[0], world_coord[1], current_pose[2], current_pose[3], current_pose[4], current_pose[5]]),
                ("Z轴调整", [world_coord[0], world_coord[1], world_coord[2] + 0.05, current_pose[3], current_pose[4], current_pose[5]]),
                ("精确定位", [world_coord[0], world_coord[1], world_coord[2], current_pose[3], current_pose[4], current_pose[5]])
            ]
            
            for i, (description, target_pose) in enumerate(steps, 1):
                print(f"{i}️⃣ {description}...")
                if not self.safe_robot_move(target_pose, 'l'):
                    print(f"❌ {description}失败")
                    return
                time.sleep(0.5)
            
            print("✅ 精确移动完成")
            
        except Exception as e:
            print(f"❌ 精确移动失败: {e}")

    def handle_keyboard(self, key):
        """处理键盘输入"""
        try:
            if key == ord('h') or key == ord('H'):
                # 回到初始位置
                print("🏠 回到初始位置...")
                if self.robot_connected:
                    self.robot.go_home()
                    print("✅ 已回到初始位置")
                
            elif key == ord('s') or key == ord('S'):
                # 紧急停止
                print("🛑 紧急停止机器人...")
                if self.robot_connected:
                    self.robot.stop()
                    print("✅ 机器人已停止")
                
            elif key == ord('p') or key == ord('P'):
                # 显示当前位置信息
                if self.robot_connected:
                    print("\n📊 当前状态信息:")
                    self.robot.get_current_position()
                    
            elif key == ord('c') or key == ord('C'):
                # 校准模式 (显示十字线)
                print("📐 校准模式开启")
                
        except Exception as e:
            print(f"❌ 键盘处理失败: {e}")

    def draw_interface(self, image):
        """绘制用户界面"""
        try:
            height, width = image.shape[:2]
            
            # 绘制十字线 (图像中心)
            center_x, center_y = width // 2, height // 2
            cv2.line(image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
            cv2.line(image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
            
            # 状态信息
            status_color = (0, 255, 0) if not self.is_operating else (0, 165, 255)
            status_text = "Ready" if not self.is_operating else "Operating..."
            
            # 绘制状态栏
            cv2.rectangle(image, (10, 10), (300, 100), (0, 0, 0), -1)
            cv2.rectangle(image, (10, 10), (300, 100), (255, 255, 255), 2)
            
            cv2.putText(image, f"Status: {status_text}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            cv2.putText(image, f"Mode: {self.control_mode}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(image, "H: Home S: Stop Q: Quit", (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
        except Exception as e:
            print(f"❌ 界面绘制失败: {e}")

    def run(self):
        """主运行循环"""
        cv2.namedWindow('Enhanced Robot Vision Control', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('Enhanced Robot Vision Control', self.mouse_callback)
        
        print("\n🎮 增强版机器人视觉控制系统已启动")
        print("=" * 50)
        
        try:
            while True:
                # 获取相机数据
                try:
                    self.current_color, self.current_depth = self.camera.get_data()
                    if self.current_color is None or self.current_depth is None:
                        print("⚠️ 相机数据获取失败")
                        continue
                    
                    # 绘制界面
                    display_image = self.current_color.copy()
                    self.draw_interface(display_image)
                    
                    # 显示图像
                    cv2.imshow('Enhanced Robot Vision Control', display_image)
                    
                except Exception as e:
                    print(f"❌ 图像处理失败: {e}")
                    continue
                
                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    print("👋 用户退出程序")
                    break
                elif key != 255:  # 有按键按下
                    self.handle_keyboard(key)
                
        except KeyboardInterrupt:
            print("\n⚠️ 程序被用户中断")
        except Exception as e:
            print(f"❌ 运行时错误: {e}")
        finally:
            print("🧹 正在清理资源...")
            self.cleanup()
            cv2.destroyAllWindows()


def main():
    """主函数"""
    controller = None
    try:
        # 创建并运行控制器
        controller = EnhancedCameraControl()
        controller.run()
        
    except Exception as e:
        print(f"❌ 程序启动失败: {e}")
        print("💡 请检查:")
        print("   1. RealSense相机是否正确连接")
        print("   2. UR机器人IP地址是否正确")
        print("   3. 是否安装了所需的依赖库")
        
    finally:
        if controller:
            try:
                controller.cleanup()
            except Exception as cleanup_e:
                print(f"⚠️ 清理过程中出现异常: {cleanup_e}")
        
        try:
            cv2.destroyAllWindows()
        except:
            pass


if __name__ == "__main__":
    main()

# # CameraControl.py
# import sys
# import cv2
# import os
# import numpy as np
# from realsenseD435 import RealsenseD435
# from UR_Robot import UR_Robot
# import time

# class CameraControl:
#     def __init__(self):
#         self.camera_started = False
#         try:
#             ##################### 修改以下参数 #############################################            
#             # 初始化相机内参和畸变系数
#             self.camera_matrix = np.array([
#                 [604.258913282636, 0.0, 323.7801248531704],
#                 [0.0, 604.3289179344752, 235.021488161533],
#                 [0.0, 0.0, 1.0]
#             ])
            
#             self.dist_coeffs = np.array([
#                 [0.015988132441259323, 0.774979417324099, -0.001538144288486602, 0.0008365855434617833, -2.8805300633434845]
#             ])
            
#             # 旋转矩阵
#             self.R = np.array([
#                 [-0.029717059245760735, 0.8542787098593931, 0.5189651069876997],
#                 [0.9964067095840052, -0.01587931741139026, 0.08319565117555144],
#                 [0.07931308521145591, 0.5195726447374986, -0.8507371517423705]
#             ])
#             # 平移向量,xyz任意方向不准的话在这里进行具体的修改
#             self.t = np.array([
#                 [-0.8695994465679254],
#                 [-0.2088276852840516],
#                 [0.9248090186081733]
#             ]) 
            
#             self.depth_scale = 0.0010000000474974513
#             ##################### 修改完成 ################################################ 
        
#             # 初始化相机和机器人
#             self.camera = RealsenseD435()
#             self.camera_started = True  # 标记相机已启动
#             self.robot = UR_Robot()
            
#             # 验证机器人连接
#             if not self.is_robot_connected():
#                 raise Exception("机器人连接失败")
                
#             # 验证相机连接
#             if not self.is_camera_connected():
#                 raise Exception("相机连接失败")
            
#             # 存储当前图像和深度信息
#             self.current_color = None
#             self.current_depth = None
            
#             print("初始化成功完成")
            
#         except Exception as e:
#             print(f"初始化错误: {str(e)}")
#             self.cleanup()
#             raise

#     def cleanup(self):
#         """清理资源"""
#         if self.camera_started:
#             try:
#                 self.camera.stop()
#                 print("相机已停止")
#                 self.camera_started = False  # 标记相机已停止
#             except Exception as e:
#                 print(f"相机关闭错误: {str(e)}")
#         if hasattr(self, 'robot'):
#             try:
#                 self.robot.go_home()
#                 print("机器人已复位")
#             except Exception as e:
#                 print(f"机器人复位失败: {str(e)}")


#     def is_robot_connected(self):
#         """检查机器人是否正确连接"""
#         try:
#             # 使用更基础的连接测试
#             state_data = self.robot.get_state()
#             return state_data is not None
#         except Exception as e:
#             print(f"机器人连接检查错误: {str(e)}")
#             return False
            
#     def is_camera_connected(self):
#         """检查相机是否正确连接"""
#         try:
#             # 获取几帧数据确保相机稳定
#             for i in range(3):
#                 color, depth = self.camera.get_data()
#                 if color is not None and depth is not None:
#                     # 打印图像尺寸以验证
#                     print(f"尝试 {i+1}: 彩色图像尺寸: {color.shape}, 深度图像尺寸: {depth.shape}")
#                     return True
#                 else:
#                     print(f"尝试 {i+1}: 未获取到有效数据")
#                 time.sleep(0.2)
#             return False
#         except Exception as e:
#             print(f"相机连接错误: {str(e)}")
#             return False
            
#     def check_position_reached(self, target_pos, actual_pos, tolerance=0.001):
#         """检查是否到达目标位置"""
#         try:
#             # 输入验证
#             if actual_pos is None or target_pos is None:
#                 print("位置数据为空")
#                 return False
                
#             # 确保数据长度正确
#             if len(actual_pos) < 3 or len(target_pos) < 3:
#                 print("位置数据长度不足")
#                 return False
                
#             # 转换为numpy数组并确保是float类型
#             target = np.array(target_pos[:3], dtype=np.float64)
#             actual = np.array(actual_pos[:3], dtype=np.float64)
            
#             # 检查无效值
#             if np.any(np.isnan(target)) or np.any(np.isnan(actual)):
#                 print("检测到无效位置值")
#                 return False
                
#             if np.any(np.isinf(target)) or np.any(np.isinf(actual)):
#                 print("检测到无限位置值")
#                 return False
                
#             # 计算欧氏距离
#             distance = np.linalg.norm(target - actual)
            
#             # 添加调试信息
#             print(f"目标位置: {target}")
#             print(f"当前位置: {actual}")
#             print(f"位置误差: {distance*1000:.2f}mm")
            
#             return distance < tolerance
            
#         except Exception as e:
#             print(f"位置检查错误: {str(e)}")
#             return False

#     def pixel_to_world(self, pixel_x, pixel_y, depth):
#         """将像素坐标转换为世界坐标"""
#         try:
#             # 像素坐标转换为相机坐标
#             x = (pixel_x - self.camera_matrix[0,2]) * depth / self.camera_matrix[0,0]
#             y = (pixel_y - self.camera_matrix[1,2]) * depth / self.camera_matrix[1,1]
#             z = depth
            
#             # 相机坐标转世界坐标
#             camera_point = np.array([[x], [y], [z]])
#             world_point = np.dot(self.R, camera_point) + self.t
            
#             return world_point.flatten()
#         except Exception as e:
#             print(f"坐标转换错误: {str(e)}")
#             return None
    
#     def safe_robot_move(self, target_pose, movement_type='l', k_acc=1, k_vel=1):
#         """安全的机器人运动控制"""
#         try:
#             # 确保target_pose是正确的格式
#             target_pose = [float(x) for x in target_pose]
#             print("目标位置:", target_pose)
            
#             # 如果带有夹爪
#             target_pose[2] = target_pose[2]
#             # 执行移动
#             if movement_type == 'l':
#                 self.robot.move_l(target_pose, k_acc=k_acc, k_vel=k_vel)
#             elif movement_type == 'j':
#                 # 直接传递位姿，不做任何嵌套
#                 self.robot.move_j_p(target_pose, k_acc=k_acc, k_vel=k_vel)
            
#             # 等待移动完成
#             time.sleep(1.0)
            
#             # 验证移动结果
#             current_pos = self.get_robot_position()
#             if current_pos is not None:
#                 # 计算位置误差
#                 target_array = np.array(target_pose[:3])
#                 current_array = np.array(current_pos[:3])
#                 distance = np.linalg.norm(target_array - current_array)
#                 print(f"位置误差: {distance*1000:.2f}mm")
                
#                 if distance > 0.001:
#                     print("警告: 未达到期望精度")
                
#             return True
            
#         except Exception as e:
#             print(f"机器人移动错误: {str(e)}")
#             return False

#     def get_robot_position(self):
#         """获取当前机器人位置"""
#         try:
#             state_data = self.robot.get_state()
#             if not state_data:
#                 print("无法获取机器人状态数据")
#                 return None
                
#             # 添加调试信息
#             print("状态数据大小:", len(state_data) if state_data else "None")
            
#             if state_data and len(state_data) >= 48:  # 确保数据长度足够
#                 tool_positions = self.robot.parse_tcp_state_data(state_data, 'cartesian_info')
#                 if tool_positions is not None:
#                     return [float(x) for x in tool_positions]
#             return None
#         except Exception as e:
#             print(f"获取位置错误: {str(e)}")
#             return None

#     def emergency_stop(self):
#         """紧急停止机器人"""
#         try:
#             self.robot.stop()
#             time.sleep(0.5)  # 等待停止命令执行
#             print("机器人已紧急停止")
#         except Exception as e:
#             print(f"紧急停止失败: {str(e)}")

#     def mouse_callback(self, event, x, y, flags, param):
#         """鼠标点击事件处理"""
#         if event == cv2.EVENT_LBUTTONDOWN:
#             try:
#                 if self.current_depth is not None:
#                     depth = self.current_depth[y, x] * self.depth_scale
#                     if depth > 0:
#                         # 计算目标世界坐标
#                         target_coord = self.pixel_to_world(x, y, depth)
#                         if target_coord is None:
#                             print("无法计算世界坐标")
#                             return
                            
#                         print(f"\n目标坐标: X={target_coord[0]:.4f}, Y={target_coord[1]:.4f}, Z={target_coord[2]:.4f}")
                        
#                         # 构造目标姿态 - 以列表形式而不是嵌套列表
#                         target_pose = [target_coord[0], target_coord[1], target_coord[2], -np.pi, 0, 0]
#                         pre_pose = [target_coord[0], target_coord[1], target_coord[2] + 0.1, -np.pi, 0, 0]
                        
#                         # 移动顺序
#                         time.sleep(0.5)
#                         print("移动到预位置...")
#                         if not self.safe_robot_move(pre_pose, 'l', k_acc=0.4, k_vel=0.4):
#                             print("移动到预位置失败")
#                             return
                            
#                         time.sleep(2)
#                         print("移动到目标位置...")
#                         if not self.safe_robot_move(target_pose, 'l', k_acc=0.4, k_vel=0.4):
#                             print("移动到目标位置失败")
#                             return
                            
#                         time.sleep(2)
#                         print("返回预位置...")
#                         if not self.safe_robot_move(pre_pose, 'l', k_acc=0.4, k_vel=0.4):
#                             print("返回预位置失败")
#                             return
                            
#                         print("动作完成，可以选取下一个位置点")
                        
#             except Exception as e:
#                 print(f"操作失败: {str(e)}")
    
#     def run(self):
#         """主运行循环"""
#         cv2.namedWindow('Camera View')
#         cv2.setMouseCallback('Camera View', self.mouse_callback)
        
#         print("开始相机控制。点击图像以移动机器人到目标位置。")
#         print("按'q'退出。")
        
#         try:
#             while True:
#                 # 获取相机图像
#                 try:
#                     self.current_color, self.current_depth = self.camera.get_data()
#                     if self.current_color is None or self.current_depth is None:
#                         print("获取相机数据失败")
#                         continue
                    
#                     # 显示图像
#                     cv2.imshow('Camera View', self.current_color)
                    
#                 except Exception as e:
#                     print(f"相机错误: {str(e)}")
#                     continue
                
#                 # 按'q'退出
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     break
                
#         except KeyboardInterrupt:
#             print("\n程序被用户中断")
#         except Exception as e:
#             print(f"运行时错误: {str(e)}")
#         finally:
#             print("清理...")
#             self.cleanup()
#             cv2.destroyAllWindows()


# if __name__ == "__main__":
#     controller = None
#     try:
#         controller = CameraControl()
#         controller.run()
#     except Exception as e:
#         print(f"程序失败: {str(e)}")
#     finally:
#         if controller:
#             try:
#                 cv2.destroyAllWindows()
#             except Exception as cleanup_e:
#                 print(f"清理资源时出错: {str(cleanup_e)}")

