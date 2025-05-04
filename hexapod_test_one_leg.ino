#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Ramp.h>

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Channels PCA9685
#define J1_CHANNEL 0
#define J2_CHANNEL 1
#define J3_CHANNEL 2

// Servo consts
#define SERVO_MIN 150  // 0 deg
#define SERVO_MAX 600  // 180 deg
#define SERVO_RANGE 180.0

// IK consts
const double J2L = 85.0;  // femur
const double J3L = 124.0; // tibia

const double Y_Rest = 70.0; 
const double Z_Rest = -80.0;

const double J3_LegAngle = 15.4;

// Neutral pos
double J1Act = 90.0; 
double J2Act = 45.0; 
double J3Act = 45.0;
rampDouble J1Tar = 90.0; 
rampDouble J2Tar = 45.0; 
rampDouble J3Tar = 45.0;

// Control varables
bool started = false;
bool ended = false;
uint8_t commandStep = 0;

// Sample test
bool test = false;

const double lines[][4] = {
  {0.0, 0.0, 0.0, 200},   
  {0.0, -20.0, 0.0, 200}, 
  {0.0, 20.0, 0.0, 200},  
  {0.0, 0.0, 20.0, 200},  
  {0.0, 0.0, -20.0, 200}, 
  {0.0, 0.0, 0.0, 200}    
};


void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // 60 Hz servo freq

  //UpdatePosition(90, 90, 90);
  //delay(5000);
}

void loop() {
  // Position update - ramp
  J1Act = J1Tar.update();
  J2Act = J2Tar.update();
  J3Act = J3Tar.update();

  // Cartesian func
  CartesianMove(J1Act, J2Act, J3Act);

  // Test mode
  if (test) {
    static bool forward = true;
    static bool test_started = false;

    if (!test_started || J2Tar.isFinished()) {
      double yMove = forward ? 40.0 : -40.0;
      forward = !forward;         
      uint16_t duration = 1000;  

      J1Tar.go(0.0, duration);   
      J2Tar.go(yMove, duration); 
      J3Tar.go(0.0, duration);   

      test_started = true;
    }

  } else {
    // Normal mode - moves from lines array
    if (!ended) {
      if (!started || J1Tar.isFinished()) {
        commandStep++;
        if (commandStep >= (sizeof(lines) / sizeof(lines[0]))) {
          ended = true;
          return;
        }

        double xMove = lines[commandStep][0];
        double yMove = lines[commandStep][1];
        double zMove = lines[commandStep][2];
        uint16_t duration = lines[commandStep][3] * 2;

        J1Tar.go(xMove, duration);
        J2Tar.go(yMove, duration);
        J3Tar.go(zMove, duration);

        started = true;
      }
    }
  }
}


void CartesianMove(double X, double Y, double Z) {
  Y += Y_Rest;
  Z += Z_Rest;

  double J1 = atan(X / Y) * (180.0 / PI);
  double H = sqrt((Y * Y) + (X * X));
  double L = sqrt((H * H) + (Z * Z));
  double J3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180.0 / PI);
  double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180.0 / PI);
  double A = atan(Z / H) * (180.0 / PI);
  double J2 = B + A;

  UpdatePosition(J1, J2, J3);
}

void UpdatePosition(double J1, double J2, double J3) {
  
  // MG996R settings (individual)
  setServoAngle(J1_CHANNEL, 90 - J1);
  setServoAngle(J2_CHANNEL, 90 - J2);
  setServoAngle(J3_CHANNEL, J3);
}

void setServoAngle(uint8_t channel, double angle) {
  angle = constrain(angle, 0.0, 180.0);
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}
