"""
简化版深度信息显示 - 支持鼠标点击获取深度
确保安装必要依赖：pip install opencv-python pyrealsense2 numpy
"""
import cv2
import numpy as np
from realsenseD435 import RealsenseD435

width = 640
height = 480
fps = 30

# 全局变量用于存储当前深度图像
current_depth_image = None
current_color_image = None

def mouse_callback(event, x, y, flags, param):
    """鼠标回调函数，显示点击位置的坐标和深度值"""
    global current_depth_image, current_color_image
    
    if event == cv2.EVENT_LBUTTONDOWN and current_depth_image is not None:
        # 确保点击位置在图像范围内
        if 0 <= x < current_depth_image.shape[1] and 0 <= y < current_depth_image.shape[0]:
            # 获取点击位置的深度值（毫米）
            depth_value = current_depth_image[y, x]
            
            if depth_value > 0:  # 有效深度值
                depth_m = depth_value / 1000.0  # 转换为米
                print(f"📍 位置: ({x}, {y}) | 深度: {depth_value:.0f}mm ({depth_m:.3f}m)")
                
                # 在图像上标记点击位置
                if current_color_image is not None:
                    # 创建带标记的图像副本
                    marked_image = current_color_image.copy()
                    
                    # 画圆标记点击位置
                    cv2.circle(marked_image, (x, y), 5, (0, 255, 0), 2)
                    cv2.circle(marked_image, (x, y), 2, (0, 255, 0), -1)
                    
                    # 添加文本信息
                    text = f"({x},{y}) {depth_value:.0f}mm"
                    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                    
                    # 确定文本位置，避免超出图像边界
                    text_x = max(5, min(x - text_size[0]//2, current_color_image.shape[1] - text_size[0] - 5))
                    text_y = max(20, y - 10) if y > 30 else y + 20
                    
                    # 添加文本背景
                    cv2.rectangle(marked_image, 
                                (text_x - 2, text_y - 15), 
                                (text_x + text_size[0] + 2, text_y + 5), 
                                (0, 0, 0), -1)
                    
                    # 添加文本
                    cv2.putText(marked_image, text, (text_x, text_y), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # 更新显示
                    cv2.imshow('Color Image', marked_image)
                    
            else:
                print(f"❌ 位置: ({x}, {y}) | 无有效深度信息")
        else:
            print(f"⚠️  点击位置超出图像范围: ({x}, {y})")

def main():
    global current_depth_image, current_color_image
    
    # 初始化相机
    try:
        camera = RealsenseD435(width=width, height=height, fps=fps)
    except Exception as e:
        print(f"相机初始化失败: {e}")
        return
    
    print("🎯 程序使用说明:")
    print("  - 按 'q' 键退出程序")
    print("  - 鼠标左键点击彩色图像获取位置和深度信息")
    print("  - 左侧窗口: 彩色图像，右侧窗口: 深度图像")
    print(f"  - 分辨率: {width}×{height} @ {fps}fps")
    print("-" * 50)
    
    # 创建窗口并设置鼠标回调
    cv2.namedWindow('Color Image')
    cv2.setMouseCallback('Color Image', mouse_callback)
    
    try:
        while True:
            # 获取相机数据
            color_image, depth_image = camera.get_data()
            
            if color_image is None or depth_image is None:
                continue
            
            # 更新全局变量
            current_color_image = color_image.copy()
            current_depth_image = depth_image
            
            # 显示彩色图像
            cv2.imshow('Color Image', color_image)
            
            # 处理并显示深度图像
            # 转换为彩色深度图
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            cv2.imshow('Depth Image (Colormap)', depth_colormap)
            
            # 可选：显示原始深度值（灰度图）
            # depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # cv2.imshow('Depth Image (Grayscale)', np.uint8(depth_normalized))
            
            # 检查按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("退出程序")
                break
                
            # 检查窗口是否被关闭
            try:
                if (cv2.getWindowProperty('Color Image', cv2.WND_PROP_VISIBLE) < 1 or 
                    cv2.getWindowProperty('Depth Image (Colormap)', cv2.WND_PROP_VISIBLE) < 1):
                    print("检测到窗口关闭，退出程序")
                    break
            except:
                break
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    
    finally:
        # 清理资源
        camera.stop()
        cv2.destroyAllWindows()
        print("程序结束")

if __name__ == "__main__":
    main()