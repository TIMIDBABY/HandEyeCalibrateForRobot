# coding=utf-8
"""
眼在手外 用采集到的图片信息和机械臂位姿信息计算 相机坐标系相对于机械臂基座坐标系的 旋转矩阵和平移向量
"""

import os
import cv2
import numpy as np

np.set_printoptions(precision=8, suppress=True)

def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 计算旋转矩阵
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx
    return R

def pose_to_homogeneous_matrix(pose):
    """将位姿转换为齐次变换矩阵"""
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)
    return R, t

def inverse_transformation_matrix(R, t):
    """计算变换矩阵的逆"""
    R_inv = R.T
    t_inv = -np.dot(R_inv, t)
    return R_inv, t_inv

def init_calibration_params(pattern_cols=11, pattern_rows=8, square_size=0.025):
    """
    初始化标定参数
    
    Parameters:
    pattern_cols (int): 标定板角点的列数
    pattern_rows (int): 标定板角点的行数
    square_size (float): 标定板一格的长度 (单位: 米)
    
    Returns:
    tuple: (角点列数, 角点行数, 格子尺寸)
    """
    return pattern_cols, pattern_rows, square_size

def camera_calibrate(images_path, pattern_params):
    """
    相机标定函数
    
    Parameters:
    images_path (str): 图像文件路径
    pattern_params (tuple): 标定板参数 (列数, 行数, 格子尺寸)
    """
    print("++++++++++ 开始相机标定 ++++++++++++++")

    XX, YY, L = pattern_params  # 解包标定板参数
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2) * L

    obj_points = []  # 世界坐标系中的3D点
    img_points = []  # 图像坐标系中的2D点

    # 获取所有jpg文件并按数字顺序排序
    image_files = [os.path.join(images_path, f) for f in os.listdir(images_path) if f.endswith('.jpg')]
    image_files.sort(key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))

    if not image_files:
        print("没有找到图片，退出程序...")
        return None, None, None, None

    print(f"找到 {len(image_files)} 张图片，开始处理...")

    for idx, image_file in enumerate(image_files):
        print(f"正在处理第 {idx + 1} 张图片: {image_file}")

        img = cv2.imread(image_file)
        if img is None:
            print(f"警告: 无法读取图片 {image_file}，跳过。")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

        if ret:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
            img_points.append(corners2)

            cv2.drawChessboardCorners(img, (XX, YY), corners2, ret)
            cv2.imshow("Chessboard", img)
            cv2.waitKey(500)
        else:
            print("未能找到角点！")

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    if ret:
        print("内参矩阵:\n", mtx)
        print("畸变系数:\n", dist)
        print("++++++++++ 相机标定完成 ++++++++++++++")
        return rvecs, tvecs, mtx, dist
    else:
        print("错误: 相机标定失败。")
        return None, None, None, None

def process_arm_pose(arm_pose_file):
    """处理机械臂的pose文件，并计算逆变换"""
    R_arm, t_arm = [], []
    with open(arm_pose_file, "r", encoding="utf-8") as f:
        all_lines = f.readlines()
    for line in all_lines:
        pose = [float(v) for v in line.split(',')]
        R, t = pose_to_homogeneous_matrix(pose=pose)
        # 计算逆变换
        R_inv, t_inv = inverse_transformation_matrix(R, t)
        R_arm.append(R_inv)
        t_arm.append(t_inv)
    return R_arm, t_arm

def save_matrices_to_txt(matrices_dict, filename="all"):
    """
    保存多个矩阵到同一个txt文件
    matrices_dict: 字典，键为矩阵名称，值为矩阵数据
    filename: 保存的文件名
    """
    # 确保目录存在
    save_dir = "./measured_data_out"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    # 完整的文件路径
    filepath = os.path.join(save_dir, f"{filename}.txt")
    
    with open(filepath, "w", encoding="utf-8") as f:
        # 遍历所有矩阵
        for idx, (name, matrix) in enumerate(matrices_dict.items()):
            # 将输入转换为 numpy 数组
            matrix = np.array(matrix)
            
            # 写入矩阵名称
            f.write(f"=== {name} ===\n")
            
            # 处理一维数组
            if matrix.ndim == 1:
                matrix = matrix.reshape(1, -1)
            
            rows, cols = matrix.shape
            f.write("[")  # 整个矩阵的开始中括号
            f.write("\n")
            
            for i in range(rows):
                f.write("[")  # 每行的开始中括号
                for j in range(cols):
                    f.write(repr(matrix[i, j]))  # 使用 repr() 保留所有小数位
                    if j < cols - 1:
                        f.write(", ")  # 数字间用逗号和空格分隔
                f.write("]")  # 每行的结束中括号
                if i < rows - 1:
                    f.write(",\n")  # 行之间用逗号和换行分隔
                else:
                    f.write("\n")
            
            f.write("]")  # 整个矩阵的结束中括号
            
            # 如果不是最后一个矩阵，添加两个换行作为间隔
            if idx < len(matrices_dict) - 1:
                f.write("\n\n")
    
    print(f"已保存所有矩阵数据到：{filepath}")

def hand_eye_calibrate(images_path, arm_pose_file, pattern_params):
    """
    眼在手外标定实现
    
    Parameters:
    images_path (str): 图像文件路径
    arm_pose_file (str): 机械臂位姿文件路径
    pattern_params (tuple): 标定板参数 (列数, 行数, 格子尺寸)
    """
    # 获取相机标定结果
    rvecs, tvecs, camera_matrix, dist_coeffs = camera_calibrate(images_path, pattern_params)
    if rvecs is None:
        return None, None, None, None
    
    # 获取机械臂位姿（包含逆变换）
    R_arm, t_arm = process_arm_pose(arm_pose_file=arm_pose_file)
    
    # 将旋转向量转换为旋转矩阵
    R_pattern = []
    for rvec in rvecs:
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        R_pattern.append(rotation_matrix)

    # 使用TSAI方法进行手眼标定
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_arm, t_arm,
        R_pattern,
        tvecs,
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    print("+++++++++++眼在手外标定完成+++++++++++++++")
    return R_cam2base, t_cam2base, camera_matrix, dist_coeffs

if __name__ == "__main__":
    # 设置标定板参数
    pattern_params = init_calibration_params(
        pattern_cols=11,    # 标定板角点的列数
        pattern_rows=8,     # 标定板角点的行数
        square_size=0.025   # 标定板一格的长度 (单位: 米)
    )
    
    images_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'collect_data_out')
    arm_pose_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), './collect_data_out/poses.txt')

    R, t, mtx, dist = hand_eye_calibrate(images_path, arm_pose_file, pattern_params)

    # 创建齐次变换矩阵
    CamPose = np.eye(4)
    CamPose[:3, :3] = R
    CamPose[:3, 3] = t.flatten()

    # 保存到measured_data文件夹
    os.makedirs('measured_data_out', exist_ok=True)
    np.savetxt('./measured_data_out/mtx.txt', mtx)
    np.savetxt('./measured_data_out/dist.txt', dist)
    np.savetxt('./measured_data_out/rotation_matrix.txt', R)
    np.savetxt('./measured_data_out/translation_vector.txt', t)
    np.savetxt('./measured_data_out/camera_pose.txt', CamPose)

    matrices = {
    "内参矩阵": mtx,
    "畸变系数": dist,
    "旋转矩阵": R,
    "平移向量": t
    }
    save_matrices_to_txt(matrices)
    
    if R is not None and t is not None:
        print("旋转矩阵：")
        print(R)
        print("平移向量：")
        print(t)
        