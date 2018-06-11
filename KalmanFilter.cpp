//defines the kalman filter class which takes readings from a sensor and provides the best estimate of the actual value
//deals with raw sensor values therefore returns int16_t data types
#include <stdint.h>
class kalmanFilter{
  //variable
  int c;

  //definitions
  kalmanFilter();
  int16_t estimate();
  void updateError(int16_t);
};

//constructor for the calman filter class
kalmanFilter::kalmanFilter(){
  int c = 0;
}

int16_t kalmanFilter::estimate(){
  return 0;
}

void kalmanFilter::updateError(int16_t t){
  
}

