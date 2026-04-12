#   task7

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

  




  
  

