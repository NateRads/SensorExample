//defines the kalman filter class which takes readings from a sensor and provides the best estimate of the actual value
//deals with raw sensor values therefore returns int16_t data types (would be more accurate working with doubles)
//is defined only for a single sensor
#include "KalmanFilter.h"

//constructor for the calman filter class
kalmanFilter::kalmanFilter(double origErr, double origEst){
  errorEst = origErr;
  prevEstimate = origEst;
}

//performs the 3 steps of the kalman filter
double kalmanFilter::filter(double meas){
  measVal = meas;
  //calculate the kalmanGain
  calcGain();
  //update the estimate
  calcEstimate();
  //update the error
  calcError();
  return estimate;
}

void kalmanFilter::calcGain(){
  KG = errorEst/(errorEst+errorMeas);
}

//calculate the estimated value
//this is the one that is called
void kalmanFilter::calcEstimate(){
  estimate = prevEstimate + KG*(measVal - prevEstimate);
  prevEstimate = estimate;
}

void kalmanFilter::calcError(){
  errorEst = (1-KG)*errorEst;  
}





