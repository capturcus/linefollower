#include <AFMotor.h>    //Adafruit Motor Shield Library. First you must download and install AFMotor library
#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library
  
AF_DCMotor motor1(1, MOTOR12_1KHZ ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency
  
#define KP 2 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 5 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define M1_minumum_speed 150  //minimum speed of the Motor1
#define M2_minumum_speed 150  //minimum speed of the Motor2
#define M1_maksimum_speed 250 //max. speed of the Motor1
#define M2_maksimum_speed 250 //max. speed of the Motor2
#define MIDDLE_SENSOR 4       //number of middle sensor used
#define NUM_SENSORS 6         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define DEBUG 0
#define THRESHOLD 750
#define BASE 40
#define DIVIDER 1.3


//sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsRC qtrrc((unsigned char[]) { A5, A4, A3, A2, A1, A0} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

void setup()
{ 
Serial.begin(9600);
manual_calibration();

set_motors(0,0);
}
  
void loop()
{

unsigned int sensors[NUM_SENSORS];

qtrrc.read(sensors);

double line[NUM_SENSORS];

for (int i = 0; i < NUM_SENSORS; i++)
{
  line[i] = ((double)sensors[i]-400.)/2000.;
  Serial.print((String)line[i]+" ");
}
Serial.println();

int dir = 0;

dir += line[0] * 20;
dir += line[1] * 10;
dir += line[2] * 13;

dir -= line[3] * 13;
dir -= line[4] * 10;
dir -= line[5] * 20;

int leftSpeed=BASE+dir;
int rightSpeed=BASE-dir;
/*
String debugOut = "";
debugOut += (String)line[0]+" ";
debugOut += (String)line[1]+" ";
debugOut += (String)line[2]+" ";
debugOut += (String)line[3]+" ";
debugOut += (String)line[4]+" ";
debugOut += (String)line[5]+" ";

debugOut += (String)dir+" ";
debugOut += (String)leftSpeed+" ";
debugOut += (String)rightSpeed+" ";

Serial.println(debugOut);*/

set_motors(leftSpeed/DIVIDER, rightSpeed/DIVIDER);

//int error = position - 2000;
//int motorSpeed = KP * error + KD * (error - lastError);
//set_motors(leftMotorSpeed, rightMotorSpeed);
//set_motors(0, 0);
}
  
void set_motors(int motor1speed, int motor2speed)
{ 
//if (motor1speed > M1_maksimum_speed ) motor1speed = M1_maksimum_speed;
//if (motor2speed > M2_maksimum_speed ) motor2speed = M2_maksimum_speed;
if (motor1speed < 0) motor1speed = 0; 
if (motor2speed < 0) motor2speed = 0; 
motor1.setSpeed(motor1speed); 
motor2.setSpeed(motor2speed);
motor1.run(BACKWARD); 
motor2.run(FORWARD);
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {
  
int i;
for (i = 0; i < 250; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
  
if (DEBUG) { 
Serial.begin(9600);
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMinimumOn[i]);
Serial.print(' ');
}
Serial.println();
  
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMaximumOn[i]);
Serial.print(' ');
}
Serial.println();
Serial.println();

}
}
