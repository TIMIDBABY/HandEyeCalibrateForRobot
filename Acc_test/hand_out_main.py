"""
眼在手外验证脚本。

本脚本不会向机械臂发送运动命令，只读取 RealSense 彩色/深度图像，
并在鼠标点击时打印该点从相机坐标系转换到机器人基座坐标系后的坐标：

    P_base = T_base_camera @ P_camera
"""

import cv2
import numpy as np

from realsenseD435 import RealsenseD435

# 相机内参矩阵 camera_matrix / mtx。
CAMERA_MATRIX = np.array(
    [
        [603.8661115132229, 0.0, 322.25827084613934],
        [0.0, 603.8818543467364, 236.16126531486586],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)

# 相机畸变系数 dist_coeffs / dist。
DIST_COEFFS = np.array(
    [
        [
            0.034104320460288245,
            0.6050822548146084,
            -0.0006508861079110671,
            -0.00020757037799296792,
            -2.4776884755100372,
        ]
    ],
    dtype=float,
)

# 眼在手外标定旋转矩阵 R。
R_BASE_CAMERA = np.array(
    [
        [-0.006171194234242838, -0.8972091938379405, 0.44156265563835473],
        [-0.9998940261561149, -0.00028589296443715106, -0.014555230074707659],
        [0.01318532589805943, -0.4416056846983357, -0.8971123488298867],
    ],
    dtype=float,
)

# 眼在手外标定平移向量 t。
T_BASE_CAMERA = np.array(
    [[-0.8276189034282344], [-0.09056830563701516], [0.95025828299934]],
    dtype=float,
)

WINDOW_NAME = "Hand-Out-Eye Verify"

# 深度取值窗口半径。
# 2 表示点击点周围 5x5 像素取有效深度中值，用来减小 RealSense 单点深度噪声。
DEPTH_WINDOW_RADIUS = 2


def make_transform(R, t):
    """由旋转矩阵和平移向量构造 4x4 齐次变换矩阵。"""
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def transform_point(T, point):
    """使用 4x4 齐次变换矩阵变换一个三维点。"""
    point_h = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return (T @ point_h)[:3]


class HandOutEyeVerifier:
    def __init__(self):
        self.camera = RealsenseD435()
        self.depth_scale = getattr(self.camera, "depth_scale", 0.001)
        self.current_color = None
        self.current_depth = None
        self.T_base_camera = make_transform(R_BASE_CAMERA, T_BASE_CAMERA)

        print("Hand-out-eye verification started.")
        print("Left click: print camera/base coordinates.")
        print("Q key: quit. This script will not move the robot.")

    def get_depth_m(self, x, y):
        """取点击点周围小窗口内的有效深度中值，减少单点深度噪声。"""
        h, w = self.current_depth.shape[:2]
        x0 = max(0, x - DEPTH_WINDOW_RADIUS)
        x1 = min(w, x + DEPTH_WINDOW_RADIUS + 1)
        y0 = max(0, y - DEPTH_WINDOW_RADIUS)
        y1 = min(h, y + DEPTH_WINDOW_RADIUS + 1)
        patch = self.current_depth[y0:y1, x0:x1].astype(float) * self.depth_scale
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def pixel_to_camera(self, x, y, depth_m):
        """将像素坐标和深度转换为相机坐标系下的三维点。"""
        pixel = np.array([[[float(x), float(y)]]], dtype=float)
        normalized = cv2.undistortPoints(pixel, CAMERA_MATRIX, DIST_COEFFS)
        xn, yn = normalized[0, 0]
        return np.array([xn * depth_m, yn * depth_m, depth_m], dtype=float)

    def on_mouse(self, event, x, y, flags, param):
        """鼠标左键点击后，只计算并打印坐标，不执行任何机械臂运动。"""
        if event != cv2.EVENT_LBUTTONDOWN or self.current_depth is None:
            return

        depth_m = self.get_depth_m(x, y)
        if depth_m is None:
            print(f"No valid depth around pixel ({x}, {y}).")
            return

        camera_point = self.pixel_to_camera(x, y, depth_m)

        # P_base = T_base_camera @ P_camera
        base_point = transform_point(self.T_base_camera, camera_point)

        print("")
        print(f"Pixel: ({x}, {y}), depth: {depth_m:.4f} m")
        print(
            f"P_camera: X={camera_point[0]:.4f}, Y={camera_point[1]:.4f}, Z={camera_point[2]:.4f} m"
        )
        print(
            f"P_base:   X={base_point[0]:.4f}, Y={base_point[1]:.4f}, Z={base_point[2]:.4f} m"
        )

    def run(self):
        """主循环：持续显示相机画面并等待鼠标点击或键盘退出。"""
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)

        try:
            while True:
                color, depth = self.camera.get_data()
                if color is None or depth is None:
                    continue

                self.current_color = color
                self.current_depth = depth

                display = color.copy()
                cv2.putText(
                    display,
                    "HAND-OUT VERIFY ONLY",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    display,
                    "Click: print coords | Q: quit",
                    (12, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                )
                cv2.imshow(WINDOW_NAME, display)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    break
        finally:
            self.cleanup()

    def cleanup(self):
        """释放相机资源。"""
        try:
            self.camera.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    HandOutEyeVerifier().run()
