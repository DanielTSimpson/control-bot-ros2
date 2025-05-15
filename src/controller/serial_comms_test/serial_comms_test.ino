const int numChars = 8;

void setup(){
  Serial.begin(115200);
  delay(1000);

}

void loop(){
  int mtrData[numChars];
  if(getInstructions(mtrData)){
    Serial.println("Got data");
  }
  delay(250);
}

bool getInstructions(int* motorData) {
  const byte delimByte = 255;
  byte msg;
  static byte i = 0;
  static bool receivingData = false;
  
  while (Serial.available() > 0) {
   msg = Serial.read();
   //Serial.println(msg);
   if (msg == delimByte && !receivingData) {
      receivingData = true;
      i = 0;
    } 
    
    else if (msg == delimByte  && receivingData) {
      if (i == numChars){
        receivingData = false;
        i = 0;
        return true;
      } 
    } 
    
    else if (msg != delimByte && receivingData) {
      motorData[i++] = msg;
    }
  }

  return false;
}





 
