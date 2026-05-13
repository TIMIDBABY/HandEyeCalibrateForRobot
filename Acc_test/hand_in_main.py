"""
眼在手上验证脚本。

本脚本不会向机械臂发送运动命令，只读取当前 TCP 位姿和 RealSense
彩色/深度图像，并在鼠标点击时打印该点从相机坐标系转换到机器人
基座坐标系后的坐标：

    P_base = T_base_tcp_current @ T_tcp_camera @ P_camera
"""

import cv2
import numpy as np

from arm_control import UR_Robot_Enhanced
from realsenseD435 import RealsenseD435

# 相机内参矩阵 camera_matrix / mtx。
CAMERA_MATRIX = np.array(
    [[627.817937784, 0, 321.600292039], [0, 629.236078216, 250.749364501], [0, 0, 1]],
    dtype=float,
)

# 相机畸变系数 dist_coeffs / dist。
DIST_COEFFS = np.array(
    [
        [
            0.0816245328824,
            1.0044067567,
            0.00415520055552,
            0.00471501078343,
            -4.28696149467,
        ]
    ],
    dtype=float,
)

# 手眼标定旋转矩阵 R。
R_TCP_CAMERA = np.array(
    [
        [0.6742703399, -0.723591783504, 0.147561646697],
        [0.73681555071, 0.672605914328, -0.0685866477031],
        [-0.0496221015605, 0.154971658234, 0.986671896925],
    ],
    dtype=float,
)

# 手眼标定平移向量 t。
T_TCP_CAMERA = np.array(
    [[-0.0256382540316], [0.0387444160689], [0.28736232244]],
    dtype=float,
)

WINDOW_NAME = "Hand-In-Eye Verify"

# 深度取值窗口半径。
# 2 表示点击点周围 5x5 像素取有效深度中值，用来减小 RealSense 单点深度噪声。
DEPTH_WINDOW_RADIUS = 2


def make_transform(R, t):
    """由旋转矩阵和平移向量构造 4x4 齐次变换矩阵。"""
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def ur_pose_to_transform(pose):
    """将 UR 的 TCP 位姿 [x, y, z, rx, ry, rz] 转换为 4x4 齐次矩阵。"""
    T = np.eye(4, dtype=float)
    T[:3, 3] = np.asarray(pose[:3], dtype=float)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(pose[3:6], dtype=float))
    return T


def transform_point(T, point):
    """使用 4x4 齐次变换矩阵变换一个三维点。"""
    point_h = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return (T @ point_h)[:3]


class HandInEyeVerifier:
    def __init__(self):
        self.camera = RealsenseD435()
        self.robot = UR_Robot_Enhanced()
        self.depth_scale = getattr(self.camera, "depth_scale", 0.001)
        self.current_color = None
        self.current_depth = None
        self.T_tcp_camera = make_transform(R_TCP_CAMERA, T_TCP_CAMERA)

        print("Hand-in-eye verification started.")
        print("Left click: print camera/TCP/base coordinates.")
        print("P key: print current TCP pose.")
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

    def camera_to_base(self, camera_point):
        """眼在手上：相机点先转到 TCP，再结合当前 TCP 位姿转到基座。"""
        tcp_pose = self.robot.get_current_pose()
        if tcp_pose is None:
            return None, None, None

        # 当前机械臂 TCP 位姿，来自机器人实时读取：
        # T_base_tcp 表示 TCP/末端坐标系 -> 机器人基座坐标系。
        T_base_tcp = ur_pose_to_transform(tcp_pose)

        # P_tcp = T_tcp_camera @ P_camera
        tcp_point = transform_point(self.T_tcp_camera, camera_point)

        # P_base = T_base_tcp @ T_tcp_camera @ P_camera
        base_point = transform_point(T_base_tcp @ self.T_tcp_camera, camera_point)
        return tcp_point, base_point, tcp_pose

    def on_mouse(self, event, x, y, flags, param):
        """鼠标左键点击后，只计算并打印坐标，不执行任何机械臂运动。"""
        if event != cv2.EVENT_LBUTTONDOWN or self.current_depth is None:
            return

        depth_m = self.get_depth_m(x, y)
        if depth_m is None:
            print(f"No valid depth around pixel ({x}, {y}).")
            return

        camera_point = self.pixel_to_camera(x, y, depth_m)
        tcp_point, base_point, tcp_pose = self.camera_to_base(camera_point)
        if base_point is None:
            print("Failed to read current TCP pose.")
            return

        print("")
        print(f"Pixel: ({x}, {y}), depth: {depth_m:.4f} m")
        print(
            f"P_camera: X={camera_point[0]:.4f}, Y={camera_point[1]:.4f}, Z={camera_point[2]:.4f} m"
        )
        print(
            f"P_tcp:    X={tcp_point[0]:.4f}, Y={tcp_point[1]:.4f}, Z={tcp_point[2]:.4f} m"
        )
        print(
            f"P_base:   X={base_point[0]:.4f}, Y={base_point[1]:.4f}, Z={base_point[2]:.4f} m"
        )
        print(f"TCP pose: {[round(v, 6) for v in tcp_pose]}")

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
                    "HAND-IN VERIFY ONLY",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    display,
                    "Click: print coords | P: TCP pose | Q: quit",
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
                if key in (ord("p"), ord("P")):
                    print(f"Current TCP pose: {self.robot.get_current_pose()}")
        finally:
            self.cleanup()

    def cleanup(self):
        """释放相机和机器人连接资源；不会让机器人回 home 或移动。"""
        try:
            self.camera.stop()
        except Exception:
            pass
        try:
            self.robot.disconnect()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    HandInEyeVerifier().run()
