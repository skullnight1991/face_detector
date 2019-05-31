#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>

Servo servo1;
Servo servo2;
Servo servo3;


int pos1 = servo1.read();
int pos2 = servo2.read();
int pos3 = servo3.read();


void setup(){
  Serial.begin(9600);
  
  pinMode(13, OUTPUT);
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10);//attach it to pin 10
  servo3.attach(11);//attach it to pin 11
}

void loop(){
  Serial.println(pos1);

  char val = Serial.read();
  Serial.println(val);
  if (val == 'r') {
     if (pos3 == 0) {
      Serial.println("Can't move to right");
     }
     else{
      pos3 -= 1;
      servo3.write(pos3);
     }
  }
  else if (val == 'l') {
    if (pos3 == 180) {
      Serial.println("Can't move to left");
    }
    else {
      pos3 += 1;
      servo3.write(pos3);
    }
  }
  else if (val == 'u') {
    if (pos1 == 110) {
      Serial.println("Can't move up");
    }
    else {
      pos1 += 1; pos2 -= 1;
      servo1.write(pos1);
      servo2.write(pos2);
    }
  }
  else if (val == 'd') {
    if (pos1 == 0) {
      Serial.println("Can't move down");
    }
    else {
      pos1 -= 1; pos2 -= 1;
      servo1.write(pos1);
      servo2.write(pos2);
    }
  }
}
