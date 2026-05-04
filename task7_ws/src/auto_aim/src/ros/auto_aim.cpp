#include"rclcpp/rclcpp.hpp"
#include"auto_aim/auto_aim.hpp"
#include"sensor_msgs/msg/image.hpp"   //订阅image_raw,发布检测效果
#include"cv_bridge/cv_bridge.h"
#include"opencv2/opencv.hpp"

class auto_aim_node:public rclcpp::Node{
    private:
     cv::Mat last_image_=cv::Mat();
     using Image=sensor_msgs::msg::Image;
     rclcpp::Subscription<Image>::SharedPtr sub_image_;
     rclcpp::Publisher<Image>::SharedPtr pub_image_;
     rclcpp::TimerBase::SharedPtr time_;
     auto_aim::auto_aim auto_aim_;
     int pub_period_=33;
     public:
     auto_aim_node():Node("auto_aim"){
        //订阅者
        sub_image_=this->create_subscription<Image>("image_raw",10,std::bind(&auto_aim_node::sub_image,this,std::placeholders::_1));
        //发布者
        pub_image_=this->create_publisher<Image>("/auto_aim/camera",10);
        time_=this->create_wall_timer(
            std::chrono::milliseconds(this->pub_period_),
            std::bind(&auto_aim_node::call_image,this)
        );
     }
     //订阅回调函数
     void sub_image(const Image::SharedPtr msg){
         auto image=cv_bridge::toCvCopy(msg,"bgr8")->image;
         last_image_=image;
     }
     //发布回调函数
     void call_image(){
        if(last_image_.empty()){  //没有图片就返回
            RCLCPP_ERROR(this->get_logger(),"没有消息发布！");
            return ;
        }
          auto_aim_.procces(last_image_);
          auto image_msg_=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",last_image_).toImageMsg();
          pub_image_->publish(*image_msg_);
     }
};

int main(int argv,char **argc){
 rclcpp::init(argv,argc);
 auto auto_aim_node_=std::make_shared<auto_aim_node>();
 rclcpp::spin(auto_aim_node_);
 rclcpp::shutdown();
    return 0;
}


