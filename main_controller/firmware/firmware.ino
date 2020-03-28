#include <Encoder.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"

class PIDControllerIMU {
  private:
    const int loopTime;
    const float Kp;
    const float Ki;
    const float Kd;
    double pitch;
    double Setpoint;
    double Input;
    double Output;
    PID pid;

    double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

  public:
    PIDControllerIMU(float _Kp, float _Ki, float _Kd, int _loopTime)
      : Kp(_Kp), Ki(_Ki), Kd(_Kd), loopTime(_loopTime), pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT)
    {
      Setpoint = 0;
      Input = 0;
      Output = 0;
      pitch = 0;
      
      pid.SetOutputLimits(-100, 100);
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(loopTime);
    }

    void setPitch(double _pitch) {
      if (_pitch > 90.0) _pitch = 90.0;
      else if (_pitch < -90.0) _pitch = -90.0;

      Setpoint = mapDouble(_pitch, -90, 90, -100, 100); //convert pitch angles into motor speed percentages
    }

    void updatePitch(double _pitch) {
      pitch = _pitch;
    }

    void update() {
      Input = mapDouble(pitch, -90, 90, -100, 100);
      pid.Compute();
    }

    double getOutput() {
      return Output;
    }

    double getSetpoint() {
      return Setpoint;
    }

    double getInput() {
      return Input;
    }
};

class PIDController {
  private:
    const int loopTime;
    const double CPSMax;
    const float Kp;
    const float Ki;
    const float Kd;
    double Setpoint;
    double Input;
    double Output;
    PID pid;

    double CPS;

    double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
  public:
    PIDController(float _Kp, float _Ki, float _Kd, int _loopTime, double _CPSMax = 3.6) 
      : Kp(_Kp), Ki(_Ki), Kd(_Kd), loopTime(_loopTime), CPSMax(_CPSMax), pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT)
    {
      Setpoint = 0;
      Input = 0;
      Output = 0;
      CPS = 0;
      
      pid.SetOutputLimits(-255, 255);
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(loopTime);
    }

    void setSpeed(double _speed) {
      //take in -100% to 100%
      if (_speed > 100) _speed = 100;
      else if (_speed < -100) _speed = -100;

      Setpoint = mapDouble(_speed, -100, 100, -255, 255);
    }

    void updateCPS(double _CPS) {
      CPS = _CPS;
    }

    void update() {
      Input = mapDouble(CPS, CPSMax*-1, CPSMax, -255, 255);
      pid.Compute();
    }

    double getOutput() {
      return Output;
    }

    double getSetpoint() {
      return Setpoint;
    }

    double getInput() {
      return Input;
    }

    double getCPS() {
      return CPS;
    }
    
};

class L298N {
  private:
    const int dir1Pin;
    const int dir2Pin;
    const int pwmPin;
    const bool flipOutput;
    
  public:
    L298N(int _dir1Pin, int _dir2Pin, int _pwmPin, bool _flipOutput = false)
      : dir1Pin(_dir1Pin), dir2Pin(_dir2Pin), pwmPin(_pwmPin), flipOutput(_flipOutput) {
        pinMode(dir1Pin, OUTPUT);
        pinMode(dir2Pin, OUTPUT);
        pinMode(pwmPin, OUTPUT);

        digitalWrite(dir1Pin, LOW);
        digitalWrite(dir2Pin, LOW);
        analogWrite(pwmPin, 0);
      }

      void setPWM(int _pwm) {
        analogWrite(pwmPin, _pwm);
      }

      void setDirection(bool _direction) {
        if (flipOutput) 
        {
          if (_direction) 
          {
            digitalWrite(dir1Pin, HIGH);
            digitalWrite(dir2Pin, LOW);
          } 
          else 
          {
            digitalWrite(dir1Pin, LOW);
            digitalWrite(dir2Pin, HIGH);
          }
        } 
        else 
        {
          if (_direction) 
          {
            digitalWrite(dir1Pin, LOW);
            digitalWrite(dir2Pin, HIGH);
          } 
          else 
          {
            digitalWrite(dir1Pin, HIGH);
            digitalWrite(dir2Pin, LOW);
          }
        }
      }

      void drive(int _pwm) {
        if (_pwm < -255) _pwm = -255;
        else if (_pwm > 255) _pwm = 255;

        if (_pwm < 0) {
          setPWM(abs(_pwm));
          setDirection(false);
        } else {
          setPWM(_pwm);
          setDirection(true);
        }
      }
};

// PID controller constants
const int loopTime = 10;      // milliseconds
unsigned long startTime = 0;

// Motor 1 configuration
const int encoder1_pin_A = 2;
const int encoder1_pin_B = 3;
const int motor1_enable = 4;
const int motor1_dir1 = 50;
const int motor1_dir2 = 51;

L298N Motor1(motor1_dir1, motor1_dir2, motor1_enable);
Encoder Motor1Encoder(encoder1_pin_A, encoder1_pin_B);
PIDController Motor1PID(2, 18, 0, loopTime);

// Motor 2 configuration
const int encoder2_pin_A = 18;
const int encoder2_pin_B = 19;
const int motor2_enable = 5;
const int motor2_dir1 = 52;
const int motor2_dir2 = 53;

L298N Motor2(motor2_dir1, motor2_dir2, motor2_enable);
Encoder Motor2Encoder(encoder2_pin_A, encoder2_pin_B);
PIDController Motor2PID(2, 18, 0, loopTime);

// MPU6050 IMU configuration
MPU6050 accelgyro;
int ax, ay, az;
int gx, gy, gz;
const float GYROSCOPE_SENSITIVITY = 65.536;
const float ACCELEROMETER_SENSITIVITY = 8192.0;
float pitch, roll; //need to create PID loop for balancing //make pitch go to zero
PIDControllerIMU IMUPID(2, 18, 0, loopTime);

void complementaryFilter(int ax, int ay, int az, int gx, int gy, int gz, float *pitch, float *roll) {
  float pitchAcc, rollAcc;

  *pitch += ((float)gx / GYROSCOPE_SENSITIVITY) * loopTime/1000;
  *roll -= ((float)gy / GYROSCOPE_SENSITIVITY) * loopTime/1000;

  int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
    pitchAcc = atan2f((float)ay, (float)az) * 180 / PI;
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;
    
    rollAcc = atan2f((float)ax, (float)az) * 180 / PI;
    *roll = *roll * 0.98 + rollAcc * 0.02;
  }
}

void setup() {
  
  Serial.begin(115200);
  //while (!Serial) {;}

  accelgyro.initialize();
  if (accelgyro.testConnection() == false) {
    while (1) {;}
  }

  // the OK light, means setup was successful
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Motor2PID.setSpeed(0);
  Motor1PID.setSpeed(0);
  IMUPID.setPitch(0);

  startTime = millis();
}

void loop() {
  
  unsigned long timeDelta = millis() - startTime;
  if (timeDelta >= loopTime) {
    Motor1PID.updateCPS((double)((double)Motor1Encoder.read()/(double)timeDelta));
    Motor2PID.updateCPS((double)((double)Motor2Encoder.read()/(double)timeDelta));

    Motor1Encoder.write(0);
    Motor2Encoder.write(0);

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    complementaryFilter(ax, ay, az, gx, gy, gz, &pitch, &roll);
    //Serial.print("pitch/roll: "); Serial.print(pitch); Serial.print("\t"); Serial.println(roll); 
    IMUPID.updatePitch(pitch);

    startTime = millis();
  }

  IMUPID.update();
  //Serial.println(IMUPID.getOutput());
  //update motorPID setpoints
  Motor2PID.setSpeed(IMUPID.getOutput());
  Motor1PID.setSpeed(-1*IMUPID.getOutput());
  
  Motor1PID.update();
  Motor2PID.update();
  Motor1.drive(Motor1PID.getOutput());
  Motor2.drive(Motor2PID.getOutput());
  
  //for Serial plotter and PID tuning
  //Serial.println(timeDelta);
  /*
  Serial.print(-300); Serial.print(" "); //for window spacing
  Serial.print(300); Serial.print(" ");  //for window spacing
  Serial.print(Motor1PID.getInput(), 5); Serial.print(" ");
  Serial.print(Motor1PID.getSetpoint(), 5); Serial.print(" ");
  Serial.print(Motor2PID.getInput(), 5); Serial.print(" ");
  Serial.println(Motor2PID.getSetpoint(), 5);
  */
}
