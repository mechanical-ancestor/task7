# task7

考核七为视觉组最终考核

## 任务描述

**完成一个包含以下功能的视觉项目：**

1.打开相机 (可用海康，迈德或电脑自带的相机，若需要工业相机，来战队借取)

2.对相机进行标定

3.要求对工业相机读取帧里的装甲板进行识别

4.对识别到的装甲板进行卡尔曼滤波，预测装甲板未来的位置

5.将预测结果通过串口从你的电脑发送到另一端 (可以自行购买stm32学习套件，或是来战队借取)

**提交要求：**

1.学会模块化编程，尽量定义与实现分离，各功能块作用明确，高可移植性，高可读性；

2.撰写README.md文件，用来详细描述你的代码，介绍各区块作用，运行方式；

3.不限制语言，但推荐使用c++或rust。

## 学长留言

这是一个非常艰巨的任务，意味着你将完成一个自瞄项目，是对新一届视觉组成员的最终磨练；

考核目的是让大家清楚认识到视觉组最重要的任务“自瞄”的整个工作流程；

要完成该项目你需要了解海康，迈德等工业相机的使用，下载其驱动，对相机进行二次开发 ([海康相机官网](https://www.hikrobotics.com/cn/machinevision/)) ([迈德威视官网](https://www.mindvision.com.cn/))；

由于相机镜头存在畸变，相机内参未知，相机在空间中位姿未知，你需要对相机进行标定 ([ros2相机标定参考](https://github.com/Aubrey-xiang/ros2_camera_calibration))；

对于装甲板的识别可以使用传统opencv识别，也可以yolo模型识别，在之前的考核已经让你们写过类似；

卡尔曼滤波是一种利用“预测 + 观测”来估计系统真实状态的算法，使结果比单纯依赖传感器测量更准确、更平滑 ([卡尔曼b站教程](https://www.bilibili.com/video/BV1Rh41117MT/?spm_id_from=333.337.search-card.all.click)) ([卡尔曼滤波参考文档](https://kalmanfilter.net/CN/default_cn.aspx))；

最后将预测结果通过串口发送到另一端（可以是另一台电脑也可以是单片机），只当你与另一端的通信协议成功对上才能正常通信 ([stm32教程](https://www.bilibili.com/video/BV1th411z7sn/?spm_id_from=333.337.search-card.all.click))；

自瞄算法开源参考 ([同济开源](https://github.com/TongjiSuperPower/sp_vision_25)) ([中南开源](https://github.com/CSU-FYT-Vision/FYT2024_vision))。

```
task7/
├── CMakeLists.txt                 # 构建配置
├── configs/
│   ├── task7.yaml                 # 主配置文件
│   └── camera_intrinsics.yaml     # 相机内参（由标定生成）
├── include/task7/
│   ├── auto_aim/                  # 检测/解算/跟踪
│   ├── calibration/               # 标定模块
│   ├── config/                    # 配置解析
│   └── io/                        # I/O模块
├── src/
│   ├── main.cpp                   # 程序入口
│   ├── auto_aim/                  # 检测+解算+跟踪
│   ├── calibration/               # 标定模块
│   ├── config/                    # 配置解析
│   └── io/                        # I/O模块
├── data/
│   └── calibration_images/        # 棋盘格图像目录
├── output/
│   └── predictions.csv            # 预测结果输出
└── docs/                          # 文档目录
```

## 使用指南

### 相机标定

**Step 1: 采集棋盘格图像**

```bash
./build/task7_main capture --config configs/task7.yaml
```

操作说明：
- 将 9×6 棋盘格（方格大小 25mm）放到相机画面中
- 检测到角点后按 **`c`** 保存当前帧
- 采集 **至少 8 张** 不同角度和距离的图片
- 按 **`q`** 或 **`Esc`** 退出

图片自动保存到 `data/calibration_images/`。

**Step 2: 运行标定**

```bash
./build/task7_main calibrate --config configs/task7.yaml
```

标定完成后输出 `configs/camera_intrinsics.yaml`，包含相机内参矩阵和畸变系数。

> ⚠️ **注意：** 当前 `camera_intrinsics.yaml` 中的是示例数据，上机时需要用自己的相机重新标定。

### 自动瞄准模式

```bash
./build/task7_main autoaim --config configs/task7.yaml
```

**完整流程：**
1. 打开相机 → 实时采集视频流
2. 检测灯条 → 颜色分割 + 轮廓筛选 → 灯条配对
3. 装甲板筛选 → 长度比、角度差、中心距、垂直错位
4. PnP 解算 → 计算 3D 位姿
5. 卡尔曼预测 → 6维状态向量 (x,y,z,vx,vy,vz)
6. 串口发送 → 将预测结果发送给下位机

### 命令行参数

```
./build/task7_main [mode] [options]

模式:
  autoaim         运行自动瞄准（主功能）
  capture         采集棋盘格图像
  calibrate       运行相机标定

选项:
  --config PATH   指定配置文件路径（默认 configs/task7.yaml）
  --camera ID     指定相机设备 ID（覆盖配置文件设置）
  --no-serial     禁用串口发送
  --help, -h      显示帮助信息
```

## 配置说明

主配置文件 `configs/task7.yaml` 包含以下配置项：

### 相机设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device_id` | 0 | 相机设备号 |
| `width` | 1280 | 采集宽度 |
| `height` | 720 | 采集高度 |
| `fps` | 60 | 帧率 |
| `backend` | "auto" | 相机后端 |

### 检测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enemy_color` | "blue" | 敌方颜色 red/blue |
| `binary_threshold` | 150 | 亮度二值化阈值 |
| `color_threshold` | 45 | 颜色阈值 |
| `light_min_ratio` | 2.5 | 灯条最小长宽比 |
| `light_max_ratio` | 20.0 | 灯条最大长宽比 |
| `light_max_angle_deg` | 40.0 | 灯条最大倾斜角 |
| `armor_min_small_center_dist` | 1.0 | 小装甲板中心距下限 |
| `armor_max_small_center_dist` | 4.0 | 小装甲板中心距上限 |
| `armor_min_large_center_dist` | 3.6 | 大装甲板中心距下限 |
| `armor_max_large_center_dist` | 6.6 | 大装甲板中心距上限 |

### PnP 解算设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `calibration_path` | configs/camera_intrinsics.yaml | 标定文件路径 |
| `small_armor_width` | 0.135 | 小装甲板宽度（米） |
| `small_armor_height` | 0.055 | 小装甲板高度（米） |
| `large_armor_width` | 0.230 | 大装甲板宽度（米） |
| `large_armor_height` | 0.055 | 大装甲板高度（米） |

### 卡尔曼跟踪设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `prediction_lead_seconds` | 0.03 | 预测提前时间 |
| `lost_reset_seconds` | 0.40 | 丢失后重置时间 |
| `process_noise_position` | 1e-3 | 位置过程噪声 |
| `process_noise_velocity` | 5e-3 | 速度过程噪声 |
| `measurement_noise` | 1e-2 | 测量噪声 |

### 串口设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enabled` | 0 | 是否启用串口 |
| `port` | /dev/ttyUSB0 | 串口设备路径 |
| `baudrate` | 115200 | 波特率 |

## 输出格式

### 串口数据

```
pred,pred_x,pred_y,pred_z,meas,meas_x,meas_y,meas_z
```

### 本地记录

预测结果同时写入 `output/predictions.csv`，格式与串口数据相同。

## 算法说明

### 装甲板检测流程

```
原始图像 → 红蓝通道差 + 亮度阈值 → 二值图像
    → 查找轮廓 → 拟合椭圆筛选灯条
    → 灯条配对 → 装甲板筛选
    → PnP 解算 → 卡尔曼预测 → 串口发送
```

### 卡尔曼滤波器

- **状态向量:** `[x, y, z, vx, vy, vz]`
- **测量向量:** `[x, y, z]`
- **模型:** 常速度模型

