"""
用以检测D435可用分辨率
"""

import pyrealsense2 as rs

def check_supported_resolutions():
    """检查 RealSense 相机支持的分辨率和帧率"""
    try:
        # 创建上下文和设备
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("未发现 RealSense 设备")
            return
        
        device = devices[0]
        print(f"设备: {device.get_info(rs.camera_info.name)}")
        
        # 获取传感器
        sensors = device.query_sensors()
        
        for sensor in sensors:
            sensor_name = sensor.get_info(rs.camera_info.name)
            print(f"\n传感器: {sensor_name}")
            
            # 获取流配置
            stream_profiles = sensor.get_stream_profiles()
            
            color_profiles = []
            depth_profiles = []
            
            for profile in stream_profiles:
                if profile.stream_type() == rs.stream.color:
                    vp = profile.as_video_stream_profile()
                    color_profiles.append((vp.width(), vp.height(), vp.fps()))
                elif profile.stream_type() == rs.stream.depth:
                    vp = profile.as_video_stream_profile()
                    depth_profiles.append((vp.width(), vp.height(), vp.fps()))
            
            if color_profiles:
                print(" 彩色流支持的分辨率:")
                for width, height, fps in sorted(set(color_profiles)):
                    print(f"  {width}×{height} @ {fps}fps")
            
            if depth_profiles:
                print(" 深度流支持的分辨率:")
                for width, height, fps in sorted(set(depth_profiles)):
                    print(f"  {width}×{height} @ {fps}fps")
    
    except Exception as e:
        print(f"检查失败: {e}")

if __name__ == "__main__":
    check_supported_resolutions()