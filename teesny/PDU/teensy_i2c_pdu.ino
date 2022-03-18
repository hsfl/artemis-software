#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire2.begin();
  Serial.begin(115200);
  Serial1.begin(115200);

  delay(100);
  Wire2.beginTransmission(0x54);
  Wire2.write("I2C BEGIN\n");
  Wire2.endTransmission();
}

void loop() {
  // put your main code here, to run repeatedly:

  while(Serial.available() > 0) {
    
    Wire2.beginTransmission(0x54);
    Wire2.write(Serial.readString().c_str());
    Wire2.endTransmission();
  }
//  Wire.requestFrom(0x54, 1);
//  while(Wire.available()) {
//    char c = Wire.read();
//    if(c >= ' ' && c < 127)
//      Serial.print(c);
//  }
  
  while(Serial1.available() > 0) {
    char i2c_in = Serial1.read();
    if(i2c_in >= ' ' && i2c_in < 127)
      Serial.print(i2c_in);
  }
  delay(500);
}
