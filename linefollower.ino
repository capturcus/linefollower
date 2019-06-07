#include <AFMotor.h>    //Adafruit Motor Shield Library. First you must download and install AFMotor library
#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library

AF_DCMotor motor1(1, MOTOR12_1KHZ ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency

#define KP 5 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 15 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
//#define M1_base_speed 40  //minimum speed of the Motor1
//#define M2_base_speed 40  //minimum speed of the Motor2
#define base_speed 46
#define M1_maksimum_speed 130 //max. speed of the Motor1
#define M2_maksimum_speed 130 //max. speed of the Motor2
#define NUM_SENSORS 6         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define DEBUG 0


//sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  A5, A4, A3, A2, A1, A0
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

void setup()
{
  Serial.begin(9600);
  manual_calibration();

  set_motors(0, 0);
}

int lastError = 0;

void loop()
{
  
  unsigned int sensors[NUM_SENSORS];
  int sensorsCalib[NUM_SENSORS] = {0};
  qtrrc.readCalibrated(sensors);
  if(!(sensors[NUM_SENSORS/2] < 500 || sensors[(NUM_SENSORS+1)/2] < 500)){
    int position = qtrrc.readLine(sensors); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  }
  int error = -(position - 2500);

  int motorSpeed = ((double)(KP * error + KD * (error - lastError))) / 100.;
  lastError = error;

  int leftMotorSpeed = base_speed + motorSpeed;
  int rightMotorSpeed = base_speed - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);

  String debugOut;
  debugOut += (String)error + " ";
  debugOut += (String)motorSpeed + " ";
  debugOut += (String)leftMotorSpeed + " ";
  debugOut += (String)rightMotorSpeed + " ";
  Serial.println(debugOut);
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_maksimum_speed ) motor1speed = M1_maksimum_speed;
  if (motor2speed > M2_maksimum_speed ) motor2speed = M2_maksimum_speed;
  //if (motor1speed < 0) motor1speed = 0;
  //if (motor2speed < 0) motor2speed = 0;
  if (motor1speed > 0) {
    motor1.setSpeed(motor1speed);
    motor1.run(BACKWARD);
  } else {
    motor1.setSpeed(-motor1speed);
    motor1.run(FORWARD);
  }
  if (motor2speed > 0) {
    motor2.setSpeed(motor2speed);
    motor2.run(FORWARD);
  } else {
    motor2.setSpeed(-motor2speed);
    motor2.run(BACKWARD);
  }
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {

  Serial.println("begin calibration");
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
    Serial.println("end calibration");
  }
}
