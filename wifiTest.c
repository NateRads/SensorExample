/*Author NateRads
Provides functions to connect to a wifi network and send the sensor data to a host*/
/*#include <ESP8266WiFi.h>

//use wifi to store data on feather
//and store them on the feather until they're no longer valid

//connects to a wifi network
boolean connectWifi(char* ssid, char* ssid_password){
  Serial.println("Attempting to connect to wifi network...");
  WiFi.begin(ssid,ssid_password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print("...");
  }
  Serial.println();
  Serial.println("Wifi Connected");
  return true;
}
*/
