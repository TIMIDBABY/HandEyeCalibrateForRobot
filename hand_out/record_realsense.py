# coding=utf-8
import pyrealsense2 as rs
import numpy as np
import cv2
import os

current_dir = os.path.dirname(os.path.abspath(__file__))

# 创建一个管道
pipeline = rs.pipeline()

# Create a config并配置要流​​式传输的管道
# 颜色和深度流的不同分辨率
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 开始流式传输
profile = pipeline.start(config)
# 获取深度传感器的深度标尺（参见rs - align示例进行说明）
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# 创建measure_data文件夹（如果不存在）
measured_data_out = os.path.join(current_dir, 'measured_data_out')
os.makedirs(measured_data_out, exist_ok=True)

# 保存depth_scale到文件
try:
    # 保存depth_scale到measured_data_out
    depth_scale_path_out = os.path.join(measured_data_out, 'camera_depth_scale.txt')
    with open(depth_scale_path_out, 'w') as f:
        f.write(str(depth_scale))
    print(f"深度标尺已保存至: {depth_scale_path_out}")

except Exception as e:
    print(f"保存深度标尺时发生错误: {str(e)}")
    raise


# 创建对齐对象
# rs.align允许我们执行深度帧与其他帧的对齐
# “align_to”是我们计划对齐深度帧的流类型。
align_to = rs.stream.color
align = rs.align(align_to)
index = 0
# color_path = 'color_img'
# depth_path = 'depth_img'
frame_count=0

# Streaming循环
try:
    while True:
        index += 1
        # 获取颜色和深度的框架集
        frames = pipeline.wait_for_frames()


        # 将深度框与颜色框对齐
        aligned_frames = align.process(frames)

        # 获取对齐的帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame是640x480深度图像
        color_frame = aligned_frames.get_color_frame()

        # 验证两个帧是否有效
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        #转换深度图像为三通道
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))

        # #渲染图像
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_3d, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow('depth_colormap', depth_colormap)
        # # 原图
        cv2.namedWindow('color_image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color_image', color_image)
        # 深度图
        # cv2.namedWindow('depth', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('depth', depth_image)


        RGB_file = os.path.join("./rgb", f"{frame_count:04d}.png")
        cv2.imwrite(RGB_file, color_image)
        depth_file = os.path.join("./depth", f"{frame_count:04d}.png")
        cv2.imwrite(depth_file, depth_image)
        frame_count += 1
        # print(f"写入:{frame_count}")


        # if index % 25 == 0:
        #     color = 'depth_' + str(index) + '.png'
        #     depth = 'depth_' + str(index) + '.png'
        #     cv2.imwrite(os.path.join(color_path, color), color_image)
        #     cv2.imwrite(os.path.join(depth_path, depth), depth_colormap)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()


# import cv2
# import pyrealsense2 as rs
# import numpy as np
# import os
#
# frame_count=0
#
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#     # Start streaming
# pipeline.start(config)
#
# fourcc = cv2.VideoWriter_fourcc(*'MJPG') # 可根据需要选择编解码器 AVI
# RGBout = cv2.VideoWriter('./RGB.avi', fourcc, 30, (640, 480))
# Depth_out = cv2.VideoWriter('./depth.avi', fourcc, 30, (640, 480))
# try:
#     while True:
#         # 等待下一组帧
#         frames = pipeline.wait_for_frames()
#
#         # 获取深度和彩色图像
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()
#
#         if not depth_frame or not color_frame:
#             continue
#
#         # 将深度图像转换为NumPy数组
#         depth_image = np.asanyarray(depth_frame.get_data())
#         # 将彩色图像转换为NumPy数组
#         color_image = np.asanyarray(color_frame.get_data())
#
#         # 显示彩色图像
#
#         cv2.imshow('RealSense Frame', color_image)
#         cv2.imshow(' Frame', depth_image)
#         # if cv2.waitKey(1) & 0xFF == ord('w'):
#         #     output_file = os.path.join("./video/Biaoding", f"frame_{frame_count:04d}.jpg")
#         #     cv2.imwrite(output_file, color_image)
#         #
#         # 将彩色图像写入视频文件
#         # if cv2.waitKey(1) & 0xFF == ord('w'):
#         #     RGB_file = os.path.join("./RGB", f"{frame_count:04d}.png")
#         #     cv2.imwrite(RGB_file, color_image)
#         #     depth_file = os.path.join("./depth", f"{frame_count:04d}.png")
#         #     cv2.imwrite(depth_file, depth_image)
#         #     print(f"写入:{frame_count}")
#         #     frame_count += 1
#
#
#
#         RGB_file = os.path.join("./rgb", f"{frame_count:04d}.png")
#         cv2.imwrite(RGB_file, color_image)
#         depth_file = os.path.join("./depth", f"{frame_count:04d}.png")
#         cv2.imwrite(depth_file, depth_image)
#         frame_count += 1
#         # RGBout.write(color_image)
#         # Depth_out.write(depth_image)
#         print("写入")
#
#         # 按 'q' 键退出循环
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
# finally:
#     # 停止流并释放资源
#     pipeline.stop()
#     RGBout.release()
#     Depth_out.release()
#     cv2.destroyAllWindows()



