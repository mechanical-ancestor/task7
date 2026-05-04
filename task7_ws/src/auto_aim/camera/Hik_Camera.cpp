#include"camera/Hik_Camera.hpp"

namespace Camera{
    Hik_Camera::Hik_Camera(){
        this->info_camera();
    }

    void Hik_Camera::info_camera(){
        //获取SDK版本
        unsigned int version=MV_CC_GetSDKVersion();
        std::cout<<"SDK 版本: "<<version<<std::endl;

        //枚举设备
        
        memset(&device_list,0,sizeof(MV_CC_DEVICE_INFO_LIST));

        if(MV_CC_EnumDevices(MV_GIGE_DEVICE|MV_USB_DEVICE,&device_list)!=MV_OK){
            std::cerr<<"无法枚举设备!"<<std::endl;
            return;
        }

        //打印设备信息
        if(device_list.nDeviceNum>0){
            std::cout<<"设备数量: "<<device_list.nDeviceNum<<std::endl;
            for(unsigned int i=0;i<device_list.nDeviceNum;i++){
                MV_CC_DEVICE_INFO* device_info=device_list.pDeviceInfo[i];
                if(device_info->nTLayerType==MV_GIGE_DEVICE){
                    std::cout<<"设备 "<<i<<": GIGE 摄像头"<<std::endl;
                }else if(device_info->nTLayerType==MV_USB_DEVICE){
                    std::cout<<"设备 "<<i<<": USB 摄像头"<<std::endl;
                }
            }
        }else{
            std::cout<<"未找到设备"<<std::endl;
        }
    }

    void Hik_Camera::open(){
        //创建相机句柄
        nRet=MV_CC_CreateHandle(&handle,device_list.pDeviceInfo[0]);
        if(nRet!=MV_OK){
            std::cerr<<"无法创建句柄!"<<std::endl;
            return;
        }else{
            std::cout<<"句柄创建成功!"<<std::endl;
        }
        //打开相机
        nRet=MV_CC_OpenDevice(handle);
        if(nRet!=MV_OK){
            std::cerr<<"无法打开设备!"<<std::endl;
            return;
        }else{
            std::cout<<"设备打开成功!"<<std::endl;
        }
    }

     void  Hik_Camera::start_grab_image(){
         //开始采集
        nRet=MV_CC_StartGrabbing(handle);
        if(nRet!=MV_OK){
            std::cerr<<"无法开始采集!"<<std::endl;
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            return ;
        }else{
            std::cout<<"成功开始采集!"<<std::endl;
            return ;
        }
     }
        
    cv::Mat Hik_Camera::grab_image(){
         //获取每一帧图像
               MV_FRAME_OUT_INFO_EX stFrame={0};
               nRet=MV_CC_GetOneFrameTimeout(handle,ptrData,nDatasize,&stFrame,1000);
               if(nRet!=MV_OK){
                   std::cerr<<"无法采集图像!"<<std::endl;
                   return cv::Mat();
               }else{
                 std::cout<<"成功采集图像!"<<std::endl;
                  //处理图像数据，例如显示或保存
                  return cv::Mat (stFrame.nHeight,stFrame.nWidth,CV_8UC3,ptrData); 
                   }
            }

    

    void Hik_Camera::stop_grab(){
        //停止采集
        nRet=MV_CC_StopGrabbing(handle);
        if(nRet!=MV_OK){
            std::cerr<<"无法停止采集！"<<std::endl;
        }else{
            std::cout<<"停止采集成功！"<<std::endl; 
        }
    }
    
    Hik_Camera::~Hik_Camera(){
        //释放资源
        if(handle!=nullptr){
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            std::cout<<"设备关闭，句柄销毁成功!"<<std::endl;
        }
        if(ptrData!=nullptr){
            delete[] ptrData;
            std::cout<<"图像数据内存释放成功!"<<std::endl;
        }
   }
}