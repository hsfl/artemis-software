void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  String buf = "";
  
  if (Serial.available() > 0) {
    buf = Serial.readString();
    if(buf.length() > 0)
      Serial1.print(buf);
  }
  if (Serial1.available() > 0) {
    buf = Serial1.readString();
    Serial.print(buf);
  }
}
