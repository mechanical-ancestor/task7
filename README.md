# task7

考核七为视觉组最终考核

## 任务描述

完成一个包含以下功能的视觉项目：

1.打开相机 (可用海康，迈德或电脑自带的相机，若需要工业相机，来战队借取)

2.对相机进行标定

3.要求对工业相机读取帧里的装甲板进行识别

4.对识别到的装甲板进行卡尔曼滤波，预测装甲板未来的位置

5.将预测结果通过串口从你的电脑发送到另一端 (可以自行购买stm32学习套件，或是来战队借取)

提交要求：

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