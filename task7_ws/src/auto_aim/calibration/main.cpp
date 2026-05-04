#include"calibration.hpp"

int main(){

  calibration::calibration cali(640,480,8,6,25.0f);
  std::string dataSet_path="src/auto_aim/calibration/dataSet";
  if(cali.calibration_(dataSet_path)){
        cali.save_result();
  }
  return 0;
}