import os
import cv2
import numpy as np
import pyrealsense2 as rs
import socket
import struct
import util as util

# UR5机械臂配置
HOST = "192.168.1.43"
PORT = 30003
tool_acc = 0.4
PI = 3.141592653589

# 创建UR5 socket连接
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.connect((HOST, PORT))

# 相机配置
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

image_save_path = "./collect_data_out/"
count = 0

def get_current_pose():
    """获取UR5当前位姿"""
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    data = tcp_socket.recv(1024)
    position = struct.unpack('!6d', data[444:492])
    tcp = position
    
    # 获取旋转矢量并转换为欧拉角
    rpy = np.asarray(tcp[3:6])
    x, y, z = rpy[0], rpy[1], rpy[2]
    rxyz = util.rv2rpy(x, y, z)
    
    # 组合位置和姿态信息
    pose = [tcp[0], tcp[1], tcp[2], rxyz[0], rxyz[1], rxyz[2]]
    tcp_socket.close()
    return pose

def data_collect():
    global count
    if not os.path.exists(image_save_path):
        os.makedirs(image_save_path)
        
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL |
                                           cv2.WINDOW_KEEPRATIO | 
                                           cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("detection", color_image)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('s'):  # 按s键保存数据
            print(f"采集第{count}组数据...")
            
            # 获取当前机械臂位姿
            pose = get_current_pose()
            print(f"机械臂pose:{pose}")

            # 保存位姿数据
            with open(f'{image_save_path}poses.txt', 'a+') as f:
                pose_str = [str(i) for i in pose]
                new_line = f'{",".join(pose_str)}\n'
                f.write(new_line)

            # 保存图像
            cv2.imwrite(image_save_path + str(count) + '.jpg', color_image)
            count += 1
            
        elif k == ord('q'):  # 按q键退出
            break

    pipeline.stop()
    tcp_socket.close()

if __name__ == "__main__":
    data_collect()