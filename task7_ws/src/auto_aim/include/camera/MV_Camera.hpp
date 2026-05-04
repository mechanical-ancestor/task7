#ifndef MV_CAMERA_HPP
#define MV_CAMERA_HPP

#include"opencv2/opencv.hpp"
#include"CameraApi.h"
#include <opencv2/core/types_c.h>

//初始化 → 枚举 → 打开 → 配置 → 采集 → 释放

namespace Camera{
 class MV_Camera{
    unsigned char   *g_pRgbBuffer;  //数据缓冲区

    int                     iCameraCounts = 1;
    int                     iStatus=-1;   //状态码
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    //IplImage *iplImage = NULL;
    int                     channel=3;

  public:
     MV_Camera();

     int initCamera();  //初始化，获取图片

     cv::Mat grab_image();
     
    void release();

  };
}

#endif 