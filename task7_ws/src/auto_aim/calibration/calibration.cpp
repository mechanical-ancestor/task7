#include"calibration.hpp"

namespace calibration{
    //构造函数
   calibration::calibration(int image_height,int image_width,int board_width,int board_height,float squareSize):
          image_height(image_height),image_width(image_width),boardsize(board_width,board_height),squareSize(squareSize)
    {
        //准备世界坐标系
     for(int i=0;i<boardsize.height;i++){
        for(int j=0;j<boardsize.width;j++){
            objp.push_back(cv::Point3f(j*squareSize,i*squareSize,0.0f));
        }
     }
    };

    //进行标定
    bool calibration::calibration_(std::string &dataSet_path){
    //读取每一张图形
    cv::glob(dataSet_path+"/*.png",filenames,false);
    //cv::glob(dataSet_path+"/*.jpg",filenames,false);
    //处理每张图形
    if(filenames.empty()){
        std::cout<<"数据集为空，无法进行标定!"<<std::endl;
        return false;
    }
    for(auto file:filenames){
        std::cout<<"27"<<std::endl;
        cv::Mat image=cv::imread(file);
        //二值化
        cv::Mat gray;
        cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        //找到角点
        bool find=cv::findChessboardCorners(gray,boardsize,corners);
        
        if(find){
            std::cout<<"找到角点"<<std::endl;
           //亚像素精细化
           cv::cornerSubPix(gray,corners,cv::Size(11,11),cv::Size(-1,-1),
           cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1)
          );
          img_points.push_back(corners);
          obj_points.push_back(objp);
        }
    }
    //尝试标定
    try{ double rms=cv::calibrateCamera(obj_points,img_points,
                                cv::Size(640,480),
                                camera_matrix,dist_matrix,
                                rvec,tvec
                                );
      std::cout<<"相机内参矩阵："<<std::endl<<camera_matrix<<std::endl;
      std::cout<<"畸变矩阵："<<std::endl<<dist_matrix<<std::endl;
      return true;
    }catch(cv::Exception e){
        std::cout<<e.what()<<std::endl;
        std::cout<<obj_points.size()<<std::endl;
        std::cout<<img_points.size()<<std::endl;
    }
       return false;
    }
    
    //保存标定结果
    void calibration::save_result(std::string save_path){
          cv::FileStorage  fs(save_path+"/camera.yaml",cv::FileStorage::WRITE);
          fs<<"%YAML:1.0";
          fs<<"---";
          fs<<"height"<<image_height;
          fs<<"width"<<image_width;
          fs<<"camera_matrix: !!opencv-matrix";
          fs<<"data: "<<camera_matrix;
          fs<<"distortion_coefficients: !!opencv-matrix";
          fs<<"data: "<<dist_matrix;
          std::cout<<"结果保存成功！"<<std::endl;
    }
}