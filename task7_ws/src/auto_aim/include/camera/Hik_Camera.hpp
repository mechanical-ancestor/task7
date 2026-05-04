#ifndef HIK_CAMERA_HPP
#define HIK_CAMERA_HPP


#include<iostream>
#include<vector>
#include"MvCameraControl.h"
#include"opencv2/opencv.hpp"

namespace Camera{
   class Hik_Camera{
      private:
         MV_CC_DEVICE_INFO_LIST device_list;  //设备列表
         void *handle=nullptr;                //相机句柄
         int nRet=MV_CC_EnumDevices(MV_GIGE_DEVICE|MV_USB_DEVICE,&device_list);   //状态码
         unsigned int nDatasize=1920*1200*3;       //图像数据大小
         unsigned char* ptrData=new unsigned char[nDatasize];   //图像数据指针
      public:
         Hik_Camera();                       //相机构造函数
         void info_camera();                 //获取相机信息
         void open();                        //打开相机
         void start_grab_image();            //开始采集
         cv::Mat grab_image();               //获取图像
         void stop_grab();                    //停止采集
         ~Hik_Camera();                       //释放资源
   };
}



#endif // HIK_CAMERA_HPP