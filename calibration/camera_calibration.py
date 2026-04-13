
import numpy as np
import cv2
import glob
import pickle
import matplotlib.pyplot as plt


# ===================== 第一步：棋盘格角点检测 =====================
chessboard_size = (10,7)  # 内角点行列数
square_size = 0.02  # 单个棋盘格实际边长（单位：m）
img_path = '/home/wrt/task_7/calibration/include/camera_calibration_images/*.jpg'  # 标定图片路径
test_img_path = '/home/wrt/task_7/calibration/include/camera_calibration_images/img1.jpg'  # 测试图片路径
save_pickle_path = '/home/wrt/task_7/calibration/include/camera_calibration.pkl'  # pickle参数保存路径

# 准备3D物理坐标点
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# 存储物理点和图像点
objpoints = []  # 3D物理点
imgpoints = []  # 2D图像点

# 读取所有标定图片
images = glob.glob(img_path)
if not images:
    raise FileNotFoundError(f"未找到标定图片，请检查路径：{img_path}")

# 遍历图片检测角点
img_size = None
for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    if img is None:
        print(f"跳过无效图片：{fname}")
        continue
    if img_size is None:
        img_size = (img.shape[1], img.shape[0])  # 记录图片尺寸
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        # 亚像素级角点细化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_refined)

        # 绘制并显示角点
        cv2.drawChessboardCorners(img, chessboard_size, corners_refined, ret)
        cv2.imshow('Chessboard Corners', img)
        cv2.waitKey(500)  # 显示500ms后继续
cv2.destroyAllWindows()

# 检查角点检测结果
if not objpoints or not imgpoints:
    raise ValueError("未检测到有效棋盘格角点，请检查标定图片或棋盘格参数")

# ===================== 第二步：相机标定 =====================
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)
print("相机内参矩阵：\n", mtx)
print("畸变系数：\n", dist)

# ===================== 第三步：测试图像畸变校正 =====================
# 读取测试图片
test_img = cv2.imread(test_img_path)
if test_img is None:
    raise FileNotFoundError(f"未找到测试图片，请检查路径：{test_img_path}")
# 执行畸变校正
undist_img = cv2.undistort(test_img, mtx, dist, None, mtx)
# 保存校正后的图片
cv2.imwrite('calibration_wide/test_undist.jpg', undist_img)

# ===================== 第四步：保存标定参数 =====================
dist_pickle = {"mtx": mtx, "dist": dist}
with open(save_pickle_path, "wb") as f:
    pickle.dump(dist_pickle, f)
print(f"标定参数已保存至：{save_pickle_path}")

# ===================== 第五步：可视化校正结果 =====================
# 转换色彩空间（OpenCV BGR → Matplotlib RGB）
test_img_rgb = cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB)
undist_img_rgb = cv2.cvtColor(undist_img, cv2.COLOR_BGR2RGB)

# 绘制对比图
f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
ax1.imshow(test_img_rgb)
ax1.set_title('Original Image', fontsize=30)
ax2.imshow(undist_img_rgb)
ax2.set_title('Undistorted Image', fontsize=30)
plt.tight_layout()
plt.show()
