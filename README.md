# task7

考核七为视觉组最终考核

## 点击这里直接阅读有关我的项目部分

- [click here](#从下面内容了解我的项目内容)

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

## 从下面内容了解我的项目内容

### 自瞄项目目录结构

项目结构的搭建基本参考[同济开源](https://github.com/TongjiSuperPower/sp_vision_25)的方式

```bash
.
├── README.md    
│  
├── auto_aim/                   # 自瞄核心功能部分
│   ├── include
│   │   ├── detector.hpp
│   │   ├── solver.hpp
│   │   ├── tracker.hpp
│   │   └── type.hpp    
│   └── src
│       ├── detector.cpp
│       ├── solver.cpp
│       └── tracker.cpp 
│  
├── calibration/                # 标定
│   ├── camera_calib.cpp
│   └── capture.cpp
│            
├── configs/                   # 全部参数配置
│   └── ...
│
├── io/                         # 硬件层
│   ├── camera.hpp
│   ├── serials
│   │   ├── serial.cpp
│   │   └── serial.hpp
│   └── usbcamera
│       ├── usbcamera.cpp
│       └── usbcamera.hpp
│
├── tests/                     # 测试代码区               
│   └── ...
└── src/                       # 主程序源文件
    └── main.cpp              
```

### 项目简要介绍

- `io/`：目前仅实现了**USB相机**和**电脑间通讯**的功能，如果日后还有需求可以增加相应的内容
- `calibration/`：用于**相机标定**的独立功能区，想了解详情可以到文件夹里`README.md`查看
- `auto_aim`：实现初步**自瞄**核心的功能区，下面将会简单介绍
- `test`：用来放测试文件
- `src`：用来放主程序源文件

#### 自瞄核心

##### 特征识别 *

参考[中南开源](https://github.com/CSU-FYT-Vision/FYT2024_vision)，将识别所需的装甲板和灯条属性整合到`type.hpp`文件，即将特征识别与解算甚至将来所需的其他功能共需的类型结合，在把各个功能单需的独立到各自的类，个人认为这样可以使得代码的灵活性与复用性提升。

其次我仅仅是简单完成了`detector.cpp`的一些功能，代码部分还有非常大的可优化空间，下面是一些问题
1. 形态学操作的灵活性不足，即受环境和装甲板距离影响大
2. 匹配的灯条上下定点易飘，即容易抖动不稳定
  作者留言：这一点经过参考Chenjunnn的开源，得以解决（可重点查看`auto_aim/include/type.hpp`中Light的修改）如果想更进一步了解可以去看[Chenjunnn的开源](https://github.com/chenjunnn/rm_auto_aim?tab=readme-ov-file)。
3. 预测不够精准，结果绘制也不够明显
  修改了`solver.cpp`与`tracker.cpp`使得绘制结果明显,从只绘制中心点改为同时绘制**矩形+中心点**
4. 参数不精准
5. 算法逻辑可能不够严谨，目前仅是可以跑通

##### 位姿解算 *

此部分的**逻辑和架构**都是参考[同济开源](https://github.com/TongjiSuperPower/sp_vision_25)，**数据来源和方法**参考[Chenjunnn的开源](https://github.com/chenjunnn/rm_auto_aim?tab=readme-ov-file)（但此项目并不需要ROS，因此相机坐标的建立并不同）其他对应的知识可以看[OpenCV教程](https://docs.opencv.org/4.x/dc/d2c/tutorial_real_time_pose.html)

相机坐标轴：
- 原点：相机的光心（镜头中心）
- 坐标轴方向：
  - Z轴：沿光轴方向，指向前方（也就是物体在相机前方时，Z > 0）
  - X轴：水平向右
  - Y轴：竖直向下
装甲板坐标轴：
- 原点：装甲板中心
- 坐标轴方向：
  - Z轴：Down(竖直向下)
  - X轴：Forward(垂直与装甲板平面)
  - Y轴：Right(右)

##### 卡尔曼滤波 *

- 需要先学习或了解有关**线性代数**或**矩阵运算**的基本知识
- 了解卡尔曼滤波及其推导可以看B站博主[DR_CAN](https://www.bilibili.com/video/BV1yV411B7DM?spm_id_from=333.788.videopod.sections)，或者你有更好的建议可以issues

###### 重要内容

- 一些矩阵操作：
  - 转置：H.t() 
  - 求逆：S.inv()
  - 0矩阵： zeros()
  - 单位矩阵： eye()
  - 索引： at<T>(row, col)

- 参数

1. 状态矩阵 x_ (6*1)
2. 测量矩阵 z (3*1)
3. 状态转移矩阵 F(6*6)

```cpp
// CV_64F 64位浮点数

[ 0 ] /*x*/     [ 0 ]     [1  0  0  dt 0  0 ] // x方向：x[t+dt] = x[t] + v_x * dt  
[ 0 ] /*y*/     [ 0 ]     [0  1  0  0  dt 0 ]
[ 0 ] /*z*/     [ 0 ]     [0  0  1  0  0  dt]
[ 0 ] /*v_x*/             [0  0  0  1  0  0 ] 
[ 0 ] /*v_y*/             [0  0  0  0  1  0 ] 
[ 0 ] /*v_z*/             [0  0  0  0  0  1 ] 
```

4. 过程噪声协方差矩阵 Q_ (6*6)
5. 观测噪声协方差矩阵 R_ (3*3)
6. 转换矩阵 H (3*6)

用于将 6*1的状态矩阵 转换为 3*1的测量矩阵的形式

```cpp
 [1  0  0  0  0  0 ]     [1  0  0 ]     [1  0  0  0  0  0]
 [0  1  0  0  0  0 ]     [0  1  0 ]     [0  1  0  0  0  0]
 [0  0  1  0  0  0 ]     [0  0  1 ]     [0  0  1  0  0  0]
 [0  0  0  1  0  0 ] 
 [0  0  0  0  1  0 ] 
 [0  0  0  0  0  1 ] 
```

- 五个主要公式
  - 先验估计
    - x_ = F * x_  (6x1 = 6x6 * 6x1)
  - 先验误差协方差矩阵
    -  P_ = F * P_ * F.t() + Q_  (6x6 = 6x6 * 6x6 * 6x6 + 6x6)
  - 卡尔曼增益:
    - S = H * P_ * H.t() + R_   (3x3 = 3x6 * 6x6 * 6x3 + 3x3)
    - K = P_ * H.t() * S.inv()  (6x3 = 6x6 * 6x3 * 3x3) 
  - 后验估计: 
    - y = z - H * x_   (3x1 = 3x1 - 3x6 * 6x1)
    - x_ = x_ + K * y  (6x1 = 6x1 + 6x3 * 3x1)
  - 后验误差协方差矩阵
    - P_ = (I - K * H) * P_  (6x6 = (6x6 - 6x3 * 3x6) * 6x6)

### 感谢

#### GitHUb
- [Chenjunnn开源](https://github.com/chenjunnn/rm_auto_aim?tab=readme-ov-file)
- [同济开源](https://github.com/TongjiSuperPower/sp_vision_25)
- [中南开源](https://github.com/CSU-FYT-Vision/FYT2024_vision)

#### Bilibili
- [DR_CAN](https://www.bilibili.com/video/BV1yV411B7DM?spm_id_from=333.788.videopod.sections)
- [OpenCV实战](https://www.bilibili.com/video/BV1erWmzPE54/?spm_id_from=333.337.search-card.all.click)

#### OpenCV 
- [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html)
