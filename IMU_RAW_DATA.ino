/* 
 * This sketch shows how nicla can be used in standalone mode.
 * Without the need for an host, nicla can run sketches that 
 * are able to configure the bhi sensors and are able to read all 
 * the bhi sensors data.
*/

#include "Arduino.h"
#include "Arduino_BHY2.h"
#include "Nicla_System.h"

SensorXYZ correctedacc(SENSOR_ID_ACC);
SensorXYZ linearacc(SENSOR_ID_LACC);
SensorOrientation ori(SENSOR_ID_ORI);
SensorXYZ rawacc(SENSOR_ID_ACC_RAW);
// SensorXYZ gyro(SENSOR_ID_GYRO);
// Sensor temp(SENSOR_ID_TEMP);
// Sensor gas(SENSOR_ID_GAS);
// SensorQuaternion rotation(SENSOR_ID_RV);
//using namespace nicla;

SensorBSEC bsec(SENSOR_ID_BSEC);

void setup()
{
  //begin();
  //leds.begin();
  Serial.begin(115200);
  while(!Serial);

  BHY2.begin();
  bsec.begin();

  correctedacc.begin();
  linearacc.begin();
  rawacc.begin();
  ori.begin();
  
}

void loop()
{

  // Update function should be continuously polled
  BHY2.update();
  //leds.setColor(green);

  // Serial.println(String("acceleration: ") + accel.toString());
  // Serial.println(String("gyroscope: ") + gyro.toString());
  // Serial.println(String("temperature: ") + String(temp.value(),3));
  // Serial.println(String("gas: ") + String(gas.value(),3));
  // Serial.println(String("rotation: ") + rotation.toString());
  // Serial.print(String(rawacc.x())+String(" ")+String(rawacc.y())+String(" ")+String(rawacc.z())+String(" "));
  //raw
  // float xacc_l = linearacc.x();//4096;
  // float yacc_l = linearacc.y();//4096;
  // float zacc_l = linearacc.z();//4096;
  
  float xacc = linearacc.x();//4096;
  float yacc = linearacc.y();//4096;
  float zacc = linearacc.z();//4096;
  //in gs
  float accx = xacc/4096;
  float accy = yacc/4096;
  float accz = zacc/4096;  
  //in m/s^2
  float si_accx = (xacc/4096)*9.81;
  float si_accy = (yacc/4096)*9.81;
  float si_accz = ((zacc/4096)*9.81);  
  
  //Serial.print(String(correctedacc.x()/4096)+String(" ")+String(correctedacc.y()/4096)+String(" ")+String(correctedacc.z()/4096)+String(" "));
  //Serial.print(String(xacc)+String(" ")+String(yacc)+String(" ")+String(zacc)+String(" "));
  //Serial.print(String(accx)+String(" ")+String(accy)+String(" ")+String(accz)+String(" "));
  //Serial.println(String(si_accx)+String(" ")+String(si_accy)+String(" ")+String(si_accz)+String(" "));  
  Serial.println(String(si_accx)+String(" ")+String(si_accy)+String(" ")+String(si_accz));
  //Serial.print(String(rawacc.x())+String(" ")+String(rawacc.y())+String(" ")+String(rawacc.z())+String(" "));  
  
  //Serial.println(String(correctedacc.x())+String(" ")+String(correctedacc.y())+String(" ")+String(correctedacc.z())+String(" "));
  //Serial.print(String(xacc_l)+String(" ")+String(yacc_l)+String(" ")+String(zacc_l)+String(" "));

  //Serial.println(String(ori.pitch())+String(" ")+String(ori.heading())+String(" ")+String(ori.roll()));
}
