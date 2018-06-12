#ifndef stdint
  #define stdint
  #include <stdint.h>
#endif
class kalmanFilter{
  private:
  //variable
  double errorEst, prevErrorEst, errorMeas;
  double estimate, prevEstimate, measVal;
  double KG; //the kalmanGain
  
  //definitions  
  public:
  kalmanFilter(double, double);
  double filter(double);
    
  private:
  void calcEstimate();
  void calcError();
  void calcGain();
};
