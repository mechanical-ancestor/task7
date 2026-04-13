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
**代码参考同济**

## *1* . 代码结构
 ```
task-7(wrt)
├── assets           //相关模型
│   └── tiny_resnet.onnx
|   └── yolov8.bin
|   └── assets/yolov8.xml
├── calibration      // 标定相关程序及数据
│   └──include
│         └──....
│   └──camera_calibration.py
│   └──camera_open_screenshot.cpp
|—— configs          //yolo等的相关配置文件
|   └──yolov8.yaml
|   └── camera_calibration_sources.yaml
|—— src              //自瞄主程序
|   └──aim.cpp
|—— tasks            //yolo检测的实现和卡尔曼波预测实现
|   └──yolov8.cpp    //yolo检测推理
|   └──yolov8.hpp    //将yolo推理结果封装
|   └── include
|     └──targert.cpp //卡尔曼波预测
|     └──targert.hpp //卡尔曼波预测结果封装
|     └──armor.hpp   //封装装甲板的相关数据
|     └──camera.cpp
|     └──camera.hpp  //封装相机功能
|—— io               //串口通信
|   └──serial.cpp
|   └──serial.hpp
。。。。。。。

```


## *2* . 代码相关功能的实现

### 2.1 [基于 OpenVINO 部署的 YOLOv8 装甲板检测核心实现](tasks/yolov8.cpp)
  
  定义一个yolov8的类 包含以下：
  - 构造函数 yolov8 初始化模型
  - 成员函数 detect 检测装甲板 包含后处理函数parse :<br/>
    *裁剪出ROI区域->其余部分用其他颜色填充-> 转换成模型适配的格式->进行模型推理->调用parse*
  - 成员函数 draw_detections  : <br/> 
    *框出装甲板*

### 2.2 [自瞄主程序](src/aim.cpp)
  
  - 打开相机 图像格式转换为BGR
  - 初始化yolov8模型
  - 加载相机参数
  - 获取每一帧图像 矫正图像
  - 构建detector对象 调用yolov8类.....
  * 卡尔曼预测
  - 显示检测+预测结果

### 2.3 [卡尔曼波预测功能的实现](tasks/targert.cpp)
  
1.  状态预测公式（先验状态估计）<br/>
    x^k−​=Ax^k−1+​+Buk​
2.  协方差预测公式（先验误差协方差）<br/>
    Pk−​=APk−1+​AT+Q
*   核心调用：predict() 
*   装甲板中心坐标：
    x=左上角_x+width/2;
    y=左上角_y+height/2;
*   缺点：仅当 YOLOv8 检测到装甲板时，才执行卡尔曼滤波；没检测到时时，不会输出预测位置。

### 2.4 [相机功能的实现](tasks/include/camera.cpp)
*   打开相机
*   显示画面
*   关闭相机
*   加载内参
*   获取图像帧

### 2.5 [串口通信](io/serial.cpp)
*   将卡尔曼预测的装甲板中心坐标发送

### 2.6  [获取标定图片](calibration/camera_open_screenshot.cpp)
 
  * 打开相机  <br/>   
  * 截取图像  <br/> 
  * 用于标定
  
### 2.7  [标定程序](calibration/camera_calibration.py)
  
  * 获取相机内参

  




  
  
