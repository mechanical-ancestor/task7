#include"auto_aim/auto_aim.hpp"
#include"camera/Hik_Camera.hpp"

int main(){
//创建相机
Camera::Hik_Camera camera;

auto_aim::auto_aim auto_aim_camera;

camera.open();
camera.start_grab_image();

cv::Mat frame;

while(true){
  frame=camera.grab_image();

  auto_aim_camera.procces(frame);

  cv::imshow("auto_aim_camera",frame);

  if(cv::waitKey(30)==27){
    std::cout<<"手动推出成功！"<<std::endl;
   }
  }
cv::destroyAllWindows();
return 0;
}