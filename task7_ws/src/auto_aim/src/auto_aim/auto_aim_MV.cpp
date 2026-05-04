#include"camera/MV_Camera.hpp"
#include"auto_aim/auto_aim.hpp"

int main(){
Camera::MV_Camera MV_camera;

auto_aim::auto_aim auto_aim_MV;

cv::Mat frame;
while(true){
   frame=MV_camera.grab_image();
   auto_aim_MV.procces(frame);
   if(cv::waitKey(30)==27){
    printf("手动推出成功！");
   }
  }
  cv::destroyAllWindows();
  MV_camera.release();
}