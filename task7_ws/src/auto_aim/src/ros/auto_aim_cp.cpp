#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/image.hpp"
#include"cv_bridge/cv_bridge.h"
#include"auto_aim/auto_aim.hpp"


class auto_aim_cp:public rclcpp::Node{
private:
      using Image=sensor_msgs::msg::Image;
      rclcpp::Publisher<Image>::SharedPtr pub_image_;
      rclcpp::TimerBase::SharedPtr timer_;
      cv::Mat last_image_;
      cv::VideoCapture cap_;
      auto_aim::auto_aim auto_aim_;
      int pub_period=33;
public:
      auto_aim_cp():Node("auto_aim_cp"){ 
        open_camera();
        pub_image_=this->create_publisher<Image>("/auto_aim/image",10);
        timer_=this->create_wall_timer(
            std::chrono::milliseconds(this->pub_period),
            std::bind(&auto_aim_cp::call_image,this)
        );
      }

      void call_image(){  
        cap_>>last_image_;
        if(last_image_.empty()){
          RCLCPP_ERROR(this->get_logger(),"捕捉到空帧！");
          return ;
        }
        auto_aim_.procces(last_image_);
        auto image_msg=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",last_image_).toImageMsg();
        pub_image_->publish(*image_msg);
      }

      void open_camera(){
        cap_.open(0,cv::CAP_V4L2);
        if(!cap_.isOpened()){
            RCLCPP_ERROR(this->get_logger(),"无法打开摄像头！");
        }else{
            RCLCPP_INFO(this->get_logger(),"成功打开摄像头！");
        }
      }

      void close_camera(){
         if(cap_.isOpened()){
             cap_.release();
             RCLCPP_INFO(this->get_logger(),"摄像头资源已释放！");
       }
      }

      ~auto_aim_cp(){
        close_camera();
      }
};

int main(int args, char **argv){
    rclcpp::init(args,argv);
    auto auto_aim_cp_node=std::make_shared<auto_aim_cp>();
    rclcpp::spin(auto_aim_cp_node);
    rclcpp::shutdown();
    return 0;
}