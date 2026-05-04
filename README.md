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

### <em>1.代码结构</em>
```
.
├── assets            //yolo配置文件
│   └── yolo8
├── build
├── calibration       //相机标定代码
│   ├── build
│   ├── calibration.cpp
│   ├── calibration.hpp
│   ├── CMakeLists.txt
│   ├── dataSet
│   ├── main.cpp
│   └── result
├── camera             //相机驱动代码
│   ├── Hik_Camera.cpp  //海康相机
│   └── MV_Camera.cpp   //迈德威视
├── CMakeLists.txt
├── config
│   ├── camera.yaml    //相机内参配置文件
│   └── config.yaml    //主要代码参数配置文件
├── include            //头文件目录
│   ├── auto_aim       //自瞄模块集成代码，也是主要实现代码
│   ├── camera
│   ├── serial
│   └── tools
├── package.xml
├── serial             //串口外设
│   └── serial.cpp 
├── src               //主要源文件
│   ├── auto_aim      //c++可直接进行make编译的代码
│   └── ros           //依赖ros环境
├── tools             //自瞄代码各功能模块的实现目录
│   ├── filter        //卡尔曼滤波
│   ├── solverPnP     //pnp算法
│   └── yolo          //yolo识别
```
### <em>2.代码说明</em>
1.src/auto_aim 包含海康相机，迈德威视以及电脑摄像头的自瞄实现代码
2.src/ros 包含电脑摄像头的自瞄实现代码以及订阅相机话题**image_raw**实现自瞄的代码模块
```
cd src/auto_aim/build
./auto_aim_cp      #电脑摄像头打开代码
./auto_aim_camera  #海康相机
./auto_aim_ MV     #迈德威视

ros2 run auto_aim auto_aim_node    #通用订阅话题进行自瞄模块
ros2 run auto_aim auto_aim_cp_node #摄像头自瞄模块
```


### <em>3.自瞄实现思路</em>
1.相机标定获取内参矩阵和畸变系数
```
内参矩阵 camera_matrix:
[fx,0,cx]   fx,fy代表相机焦距
[0,fy,cy]   cx,cy代表相机中心坐标
[0, 0, 1]  
```
##### 2.训练模型获取.pt在再转换为.xml和.bin用于c++和openVINO推理
  创建推理模型<br>
  将输入图像转换为向量形式<br>
  通过模型得到输出向量<br>
  在通过解析输出向量得到置信度，rect等信息<br>
#### 3.将推理获得检测框的中心坐标用于卡尔曼滤波预测
  用状态转移方程得到先验估计也就是预测值并返回和更新先验协方差<br>
  得到观测值计算卡尔曼增益以及后验估计作为最优结果更新后验证协方差<br>
#### 4.将预测结果通过pnp算法发送给串口
   获取相机参数<br>
   获取装甲版平面各个角点的三维坐标<br>
   利用预测得到的中心坐标获取四个角点的二维平面坐标<br>
   用cv::solvePnP得到平移向量和旋转向量<br>

### <em>4.代码存在问题</em>
1.推理结果存在误检，且误检率较高<br>
2.模型的检测框与装甲板的重合效果并不是太好，存在一定的偏差<br>
3.模型识别的最大距离大概为0.6m，效果较差，难以满足需求<br>
4.相机驱动代码未经历实践，尚不清楚存在的问题<br>
5.标定代码存在可能无法识别棋盘角点的错误导致标失败<br>
