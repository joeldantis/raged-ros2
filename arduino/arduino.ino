#include "CytronMotorDriver.h"
#include "Servo.h"

CytronMD motor1(PWM_DIR, 5, 7);
CytronMD motor2(PWM_DIR, 6, 8);
CytronMD motor3(PWM_DIR, 9, 12);
CytronMD motor4(PWM_DIR, 10, 13);

// ANGLE VARIABLES START
// PD control constants (for angle)
float Kp = 1.5;
float Kd = 3.0;

// PD control variables (for angle)
float error = 0;
float lastError = 0;
float derivative = 0;
float controlSignal = 0;
int delay_time = 0;
int k = 4;

// Base speed for motors (adjust as needed)
int baseSpeed = 0;
int maxPWM = 255;
// ANGLE VARIABLES END

void setup() {
  Serial.begin(9600);
}

void automatic(String var1, String var2) {
  if (var1.toInt() == 0) {
    collect(var2);
  }

  else if (var1.toInt() == 1) {
    ahead(var2);
  }

  else if (var1.toInt() == 2) {
    rotate(var2);
  }
}

void manual(String var1) {
  
}

void rotate(String err) {
    // PD control
    float error = err.toFloat();
    derivative = error - lastError;
    controlSignal = (Kp * error) + (Kd * derivative);
    lastError = error;
    delay_time = k * abs(error);

    // Calculate left and right motor speeds
    //int pwmLeft  = constrain(baseSpeed - controlSignal, -255, maxPWM);
    int pwm = constrain(baseSpeed + controlSignal, -150, 1);

    if ((pwm < 100) && (pwm>=0))
    {
      pwm = map(pwm,0,100,100,150);
    }

    else if ((pwm > -100) && (pwm<=0))
    {
      pwm = map(pwm,0,-100,-100,-150);
    }  

    motor1.setSpeed(pwm);
    motor2.setSpeed(pwm);
    motor3.setSpeed(pwm);
    motor4.setSpeed(pwm);

    delay(delay_time);

    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    Serial.write("r");
}

void ahead(String dist) {
  float del = dist.toFloat() * 0.04 * 1000;

  motor1.setSpeed(-31);
  motor2.setSpeed(25);
  motor3.setSpeed(40);
  motor4.setSpeed(51);

  delay(del);

  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}

void collect(String section) {
// includde bin movement and actuation of claw and lid
}

void loop() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');  // Read data until newline
    //String str = "hello,world";
    str.trim();  // Remove extra spaces or newlines
    char cstr[str.length()+1];
      strcpy(cstr, str.c_str());
    char *myPtr = strtok(cstr, ",");
    String arr[3];

    for (int i=0;i<3;i++) {
      if (myPtr != NULL) {
          arr[i] = myPtr; 
          //Serial.print("Parsed token: ");
          //Serial.println(arr[i]);
          myPtr = strtok(NULL, ","); 
      }
    }

    if ((arr[0].toInt()) == 1) {
      automatic(arr[1], arr[2]);
    }

  }
}
