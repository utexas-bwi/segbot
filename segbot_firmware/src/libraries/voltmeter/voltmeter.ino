/*
  *
  * Udemy.com
  * Building an Arduino DC Voltmeter
  *
  */
 
 float vPow = 4.8;
 float r1 = 100000;
 float r2 = 18000;
 
 void setup() {
   Serial.begin(9600);
   
   // Send ANSI terminal codes
   Serial.print("\x1B");
   Serial.print("[2J");
   Serial.print("\x1B");
   Serial.println("[H");
   // End ANSI terminal codes
   
   Serial.println("--------------------");
   Serial.println("DC VOLTMETER");
   Serial.print("Maximum Voltage: ");
   Serial.print((int)(vPow / (r2 / (r1 + r2))));
   Serial.println("V");
   Serial.println("--------------------");
   Serial.println("");
   
   delay(2000);
 }
 
 void loop() {
   float v = (analogRead(0) * vPow) / 1024.0;
   float v2 = v / (r2 / (r1 + r2));
   
   // Send ANSI terminal codes
   Serial.print("\x1");
   // End ANSI terminal codes
   
   Serial.println(v2/2);
 }
