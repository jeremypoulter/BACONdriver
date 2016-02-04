/* v202_rx.ino -- An arduino sketch to test the protocol v202
 *
 * Copyright (C) 2014 Alexandre Clienti
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <SoftwareServo.h>
#include <SPI.h>
#include <v202_protocol.h>

nrf24l01p wireless; 
v202Protocol protocol;

unsigned long time = 0;

SoftwareServo myservo;
#define STEARING_RANGE 28
#define STEARING_MIN (90-STEARING_RANGE)
#define STEARING_MAX (90+STEARING_RANGE)

#define THROTTLE_MIN    32
#define THROTTLE_MAX    255

#define MOTOR_EN    8
#define MOTOR_CS    7
#define MOTOR_INA   4
#define MOTOR_INB   5
#define MOTOR_PWM   6

#define SEP ","
//#define SEP "\t"

// the setup routine runs once when you press reset:
void setup() 
{
  // SS pin must be set as output to set SPI to master !
  pinMode(SS, OUTPUT);
  Serial.begin(115200);

  // Set CS pin to D7 and CE pin to D8
  wireless.setPins(9,SS);
  protocol.init(&wireless);
  
  time = micros();

  myservo.attach(3);  // attaches the servo on pin 3 to the servo object 

  pinMode(MOTOR_CS, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  digitalWrite(MOTOR_CS, LOW);
  digitalWrite(MOTOR_EN, LOW);

  Serial.println("Start");
}

rx_values_t rxValues;

bool bind_in_progress = false;
unsigned long newTime;

void loop() 
{
  time = micros();
  uint8_t value = protocol.run(&rxValues); 
  newTime = micros();
   
  switch( value )
  {
    case  BIND_IN_PROGRESS:
      if(!bind_in_progress)
      {
        bind_in_progress = true;
        Serial.println("Bind in progress");
      }
    break;
    
    case BOUND_NEW_VALUES:
    {
        digitalWrite(MOTOR_EN, HIGH);

        //newTime = micros();
        Serial.print(newTime - time); //120 ms for 16 Mhz

        Serial.print(SEP); Serial.print(rxValues.yaw);
        Serial.print(SEP); Serial.print(rxValues.pitch);
        Serial.print(SEP); Serial.print(rxValues.trim_yaw);
        Serial.print(SEP); Serial.print(rxValues.trim_pitch);
        Serial.print(SEP); Serial.print(rxValues.flags);

        //time = newTime;

        int servo = map(rxValues.yaw, -126, 126, STEARING_MIN, STEARING_MAX);
        int throttle = 0;
        bool reverse = false;
        
        if (rxValues.pitch < 0)
        {
            throttle = map(rxValues.pitch, 0, -126, THROTTLE_MIN, THROTTLE_MAX);
            reverse = false;
        }
        else if (rxValues.pitch > 0)
        {
            throttle = map(rxValues.pitch, 0, 124, THROTTLE_MIN, THROTTLE_MAX);
            reverse = true;
        }

        digitalWrite(MOTOR_INA, reverse ? HIGH : LOW);
        digitalWrite(MOTOR_INB, reverse ? LOW : HIGH);
        analogWrite(MOTOR_PWM, throttle);

        Serial.print(SEP); Serial.print(servo);
        Serial.print(SEP); Serial.print(throttle);
        Serial.print(SEP); Serial.print(reverse);
        Serial.print(SEP); Serial.print(analogRead(A0));
        Serial.print(SEP); Serial.println(analogRead(A1));

        myservo.write(servo);
    } break;
    
    case BOUND_NO_VALUES:
      //Serial.print(newTime - time); Serial.println(" : ----"); // 32ms for 16Mhz
      break;
    
    default:
        Serial.println(value);
        digitalWrite(MOTOR_EN, LOW);
        break;
  }

  SoftwareServo::refresh();
}
