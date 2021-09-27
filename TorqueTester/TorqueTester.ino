#include <ACS712.h>

#define INA  4
#define INB  5
#define PWM  6
#define ENA  2
#define ENB  3

ACS712 mySensor = ACS712(A1, ACS712_30);

int VoltagePin = A0;
float vout = 0.0;
float vin = 0.0;
float R1 = 30000.0;  
float R2 = 7500.0;
int value = 0;

float power = 0.0;
float electrical_torque = 0.0;
float mechnical_torque = 0.0;
float calibration_factor = 0.28;
float angular_velocity = 0.0;

float twopi = (2*3.14);


long last_checked_millis;
signed long encoderCount = 0;

unsigned long start;
volatile unsigned long counter = 0;
volatile bool pinB, pinA, dir;
const byte ppr = 12.0;
const int finish_time = 100 ;
const float constant = 600.0 / (ppr);
float rpm;

String _stop = "yes";

void setup() {

  Serial.begin(9600);
  
  pinMode(ENA, INPUT_PULLUP);           
  pinMode(ENB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENA), encoderEvent, RISING);
  pinMode(VoltagePin, INPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);           
  digitalWrite(ENA, HIGH);      
  digitalWrite(ENB, HIGH);      

  mySensor.begin();


}

void loop() 
{

    if (Serial.available()) 
    {
      _stop = Serial.read();
    }

    if (_stop == "111")
    {

    clockwise();
    
    signed long count = encoderCount;
    encoderCount = 0;

    long time_elapsed = millis() - last_checked_millis;
    double temporalmultiplier = 60.0 / (time_elapsed / 1000.0);
    last_checked_millis = millis();
    double rpm = ((count / ppr) * temporalmultiplier) / 18.0;

    Serial.print(abs(rpm));
    Serial.print(",");

    float I = mySensor.readDC(A);
    Serial.print(I); 
    Serial.print(",");

    value = analogRead(VoltagePin);
    vout = (value * 5.0) / 1024.0; 
    vin = (vout / (R2/(R1+R2))) - 1.0; 
    Serial.print(vin ); //printing Voltage
    Serial.print(",");

    power = (vin * I);
    Serial.print(power); //printing Power
    Serial.print(",");

    angular_velocity = (twopi / 60) * rpm;
    electrical_torque = (power/angular_velocity);

    mechnical_torque = (electrical_torque * calibration_factor) * -1;

    Serial.println((mechnical_torque));  //priniting Torque
      
   
    delay(100);
    }

    if (_stop == "115") 
    {
      delay(10);
      brake();
    }
 
}


void encoderEvent()
{
  if (clockwise)
  {
    encoderCount = encoderCount - 1;
  }
  else if (anticlcokwise)
  {
    encoderCount = encoderCount + 1;
  }
}



void clockwise()
{

    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    digitalWrite(PWM, HIGH);
}

void anticlcokwise()
{

    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    digitalWrite(PWM, HIGH);

}

void brake()
{
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
    digitalWrite(PWM, LOW);
}
