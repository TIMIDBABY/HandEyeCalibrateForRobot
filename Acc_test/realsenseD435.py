# realsenseD435.py
import pyrealsense2 as rs
import numpy as np

class RealsenseD435:
    def __init__(self, width=1280, height=720, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # 启用彩色和深度流
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        try:
            self.pipeline_profile = self.pipeline.start(self.config)
            self.depth_scale = self.pipeline_profile.get_device().first_depth_sensor().get_depth_scale()
            print("Realsense D435 相机初始化成功")
        except Exception as e:
            print(f"Realsense D435 相机初始化失败: {e}")
            self.pipeline = None
            raise

    def get_data(self):
        if not self.pipeline:
            print("相机未正确启动")
            return None, None
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                print("未获取到彩色或深度帧")
                return None, None
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            return color_image, depth_image
        except Exception as e:
            print(f"获取相机数据失败: {e}")
            return None, None

    def stop(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
                # print("Realsense D435 相机已停止")
            except Exception as e:
                print(f"停止相机失败: {e}")
