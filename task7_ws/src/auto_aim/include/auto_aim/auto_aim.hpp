#ifndef AUTO_AIM_CP_HPP
#define AUTO_AIM_CP_HPP

#include"tools/yolo8.hpp"
#include"tools/Armor.hpp"
#include"serial/serial.hpp"
#include"tools/KF.hpp"
#include"tools/solverPnP.hpp"
#include"opencv2/core/eigen.hpp"
#include"tools/param.hpp"

namespace auto_aim{
  class auto_aim{
   std::unique_ptr<tools::param> param;
   std::unique_ptr<tools::YOLO8> model;
   std::unique_ptr<tools::KMFilter> kf;
   std::unique_ptr<Serial::serial> serial;
   std::vector<tools::Armor> armors;
   tools::Armor armor;
   std::vector<cv::Point3f> object_points;
   cv::Mat Camera_Matrix;
   cv::Mat distortion_Matrix;
    public:
       auto_aim(){
       param=std::make_unique<tools::param>();
     
       //创建yolo8对象
       model=std::make_unique<tools::YOLO8>(param->xml_path,param->conf_threshold,param->iou_threshold);
       //创建KF对象
       kf=std::make_unique<tools::KMFilter>(param->dt);

      //创建串口
      serial=std::make_unique<Serial::serial>(param->_file,param->baudrate);
      Camera_Matrix=param->Camera_Matrix; 
      distortion_Matrix=param->distortion_coefficients;

      float width=0.2f;  //装甲板宽度，单位为米
      float height=0.1f; //装甲板高度，单位为米

      object_points=tools::solverPnP::object_Points(width,height);  //生成装甲板的三维坐

       }

    void  procces(cv::Mat &frame){
         

      cv::flip(frame,frame,1);
    
      armors=model->solver(frame);

      if(armors.empty()){          //如果有装甲板,只更新P

        kf->predict_P();

        serial->write();

      }else{
       
        tools::YOLO8::drawContours(frame,armors);    //检测框画框（红色）

        armor=armors[0];

        cv::Point2f points=armor.centry; //保存观测值以便后续更新

        cv::Point2f centry=kf->predict(); //预测返回当前帧的中心坐标传给装甲板的预测更新函数更新四个角点

        armor.get_predict(centry); 
       
        kf->update(points);

        tools::YOLO8::drawContours(frame,armor);     //预测框画框（蓝色）
        //PnP算法解析三维坐标
        cv::Mat rvec,tvec;  //旋转向量，平移向量
        cv::solvePnP(object_points,armor.Predict_Points,Camera_Matrix,distortion_Matrix,rvec,tvec,false,cv::SOLVEPNP_IPPE);
      
        serial->write(rvec.t(),tvec.t());
          }
    }
  };
}


#endif




