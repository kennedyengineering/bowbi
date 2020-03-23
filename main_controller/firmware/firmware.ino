#include <Servo.h>
#include <Encoder.h>
#include <PID_v1.h>

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

const int loopTime = 10;
unsigned long startTime = 0;

L298N Motor1(4, 7, 5, true);
Encoder Motor1Encoder(0, 1);
PIDController Motor1PID(2, 18, 0, loopTime);

L298N Motor2(8, 12, 6);
Encoder Motor2Encoder(2, 3);
PIDController Motor2PID(2, 18, 0, loopTime);

Servo Servo1;
Servo Servo2;

const int cmdLength = 5;
char input[cmdLength] = {0};
int index = 0;

void parseSerialCMD() {
  if (Serial.available() > 0) {
    input[index] = Serial.read();
    index++;

    if (index == cmdLength) {

      switch (input[0]) {
        case '1':
          //motor1 control
          if (input[1] == '+') {
            int speed = 0;
            speed += ((int)input[2]-48)*100;
            speed += ((int)input[3]-48)*10;
            speed += ((int)input[4]-48)*1;
            Motor1PID.setSpeed(speed);
          } else if (input[1] == '-') {
            int speed = 0;
            speed -= ((int)input[2]-48)*100;
            speed -= ((int)input[3]-48)*10;
            speed -= ((int)input[4]-48)*1;
            Motor1PID.setSpeed(speed);
          }
          break;
        case '2':
          //motor2 control
          if (input[1] == '+') {
            int speed = 0;
            speed += ((int)input[2]-48)*100;
            speed += ((int)input[3]-48)*10;
            speed += ((int)input[4]-48)*1;
            Motor2PID.setSpeed(speed);
          } else if (input[1] == '-') {
            int speed = 0;
            speed -= ((int)input[2]-48)*100;
            speed -= ((int)input[3]-48)*10;
            speed -= ((int)input[4]-48)*1;
            Motor2PID.setSpeed(speed);
          }
          break;
        case '3':
          //servo1 control
          {
            int pos = 0;
            pos += ((int)input[1]-48)*100;
            pos += ((int)input[2]-48)*10;
            pos += ((int)input[3]-48)*1;
            Servo1.write(pos);
          }
          break;
        case '4':
          //servo2 control
          {
            int pos = 0;
            pos += ((int)input[1]-48)*100;
            pos += ((int)input[2]-48)*10;
            pos += ((int)input[3]-48)*1;
            Servo2.write(pos);
          }
          break;
      }

      
      Serial.flush();
      index = 0;
    }
  }
}

void setup() {
  Servo1.attach(9);
  Servo2.attach(10);
  
  Serial.begin(115200);
  while (!Serial) {;}

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  startTime = millis();
}

void loop() {
  parseSerialCMD();
  
  unsigned long timeDelta = millis() - startTime;
  if (timeDelta >= loopTime) {
    Motor1PID.updateCPS((double)((double)Motor1Encoder.read()/(double)timeDelta));
    Motor2PID.updateCPS((double)((double)Motor2Encoder.read()/(double)timeDelta));

    Motor1Encoder.write(0);
    Motor2Encoder.write(0);
    startTime = millis();
  }

  Motor1PID.update();
  Motor2PID.update();
  Motor1.drive(Motor1PID.getOutput());
  Motor2.drive(Motor2PID.getOutput());

  //for Serial plotter and PID tuning
  //Serial.println(timeDelta);
  /*
  Serial.print(-300); Serial.print(" ");
  Serial.print(300); Serial.print(" ");
  Serial.print(Motor1PID.getInput(), 5); Serial.print(" ");
  Serial.print(Motor1PID.getSetpoint(), 5); Serial.print(" ");
  Serial.print(Motor2PID.getInput(), 5); Serial.print(" ");
  Serial.println(Motor2PID.getSetpoint(), 5);
  */
  
}
