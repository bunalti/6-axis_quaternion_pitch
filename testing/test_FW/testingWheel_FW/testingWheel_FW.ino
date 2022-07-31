
// Motor 
const int motor_Pin1  = 5;  
const int motor_Pin2  = 6;  
// Potentiometer
const int potPin= A0;
// Buttons
const int butPin1 = 3;
const int butPin2 = 4;

long potReading;

//This will run only one time.
void setup(){
  Serial.begin(115200);
  Serial.println("---------------Pedal Sensor Testing Wheel---------------");
  
  pinMode(motor_Pin1,OUTPUT);
  pinMode(motor_Pin2,OUTPUT);

  pinMode(potPin,INPUT);

  pinMode(butPin1,INPUT);
  pinMode(butPin2,INPUT);


  digitalWrite(motor_Pin1,LOW); 
  digitalWrite(motor_Pin2,LOW);
  
}


void loop(){

  potReading = analogRead(potPin);
  potReading /= 4;

  if(digitalRead(butPin1) && !digitalRead(butPin2)){
    digitalWrite(motor_Pin2,LOW);
    analogWrite(motor_Pin1, potReading);
    }
   else if(!digitalRead(butPin1) && digitalRead(butPin2)){
    digitalWrite(motor_Pin1,LOW);
    analogWrite(motor_Pin2, potReading);
    }
   else{
    digitalWrite(motor_Pin1,LOW); 
    digitalWrite(motor_Pin2,LOW);    
    }
  

}
