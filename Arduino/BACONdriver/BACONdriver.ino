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
#define STEARING_MID (90)
#define STEARING_MIN (STEARING_MID-STEARING_RANGE)
#define STEARING_MAX (STEARING_MID+STEARING_RANGE)

#define THROTTLE_MIN    32
#define THROTTLE_MAX    255

#define STEARING_SERVO  3

#define MOTOR_EN    7
#define MOTOR_INA   4
#define MOTOR_INB   5
#define MOTOR_PWM   6

#define CELL1_VOLTAGE   A0
#define TOTAL_VOLTAGE   A1

#define BATTERY_CURRENT A2
#define MOTOR_CURRENT   A3

#define SEP ","
//#define SEP "\t"

// the setup routine runs once when you press reset:
void setup() 
{
  // SS pin must be set as output to set SPI to master !
  pinMode(SS, OUTPUT);
  Serial.begin(9600);

  // Set CS pin to D7 and CE pin to D8
  wireless.setPins(9,SS);
  protocol.init(&wireless);
  
  time = micros();

  myservo.attach(STEARING_SERVO);  // attaches the servo on pin 3 to the servo object 

  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  digitalWrite(MOTOR_EN, HIGH);

  Serial.println("Start");
}

rx_values_t rxValues;

bool bind_in_progress = false;
unsigned long newTime;

int servo = STEARING_MID;
int throttle = 0;
bool reverse = false;

float vPow = 5.0;
float r1 = 10000;
float r2 = 10000;

void loop() 
{
  // --------------------------------------------------------
  // Read the remote controll radio (nRF21)

  time = micros();
  uint8_t value = protocol.run(&rxValues); 
  newTime = micros();
  Serial.print(newTime - time); //120 ms for 16 Mhz
   
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
      //newTime = micros();

      Serial.print(SEP); Serial.print(rxValues.yaw);
      Serial.print(SEP); Serial.print(rxValues.pitch);
      Serial.print(SEP); Serial.print(rxValues.trim_yaw);
      Serial.print(SEP); Serial.print(rxValues.trim_pitch);
      Serial.print(SEP); Serial.print(rxValues.flags);

      //time = newTime;

      servo = map(rxValues.yaw, -126, 126, STEARING_MIN, STEARING_MAX);
      throttle = 0;
      reverse = false;

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
    } break;
    
    case BOUND_NO_VALUES:
      //Serial.print(newTime - time); Serial.println(" : ----"); // 32ms for 16Mhz
      break;
    
    default:
//        Serial.println(value);
//        digitalWrite(MOTOR_EN, LOW);
        break;
  }

  // --------------------------------------------------------
  // Read the serial port
  if (Serial.available())
  {
    switch (Serial.read())
    {
      case 'L':
      case 'l':
        servo = STEARING_MIN;
        break;

      case 'R':
      case 'r':
        servo = STEARING_MAX;
        break;

      case 'M':
      case 'm':
        servo = STEARING_MID;
        break;

      case 'F':
      case 'f':
        reverse = false;
        throttle = 0;
        break;

      case 'B':
      case 'b':
        reverse = true;
        throttle = 0;
        break;

      case '0':
        throttle = THROTTLE_MIN;
        break;

      case '1':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.1);
        break;

      case '2':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.2);
        break;

      case '3':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.3);
        break;

      case '4':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.4);
        break;

      case '5':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.5);
        break;

      case '6':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.6);
        break;

      case '7':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.7);
        break;

      case '8':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.8);
        break;

      case '9':
        throttle = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * 0.9);
        break;

      case '-':
      case '_':
        throttle = 0;
        break;

      case '+':
      case '=':
        throttle = THROTTLE_MAX;
        break;
    }
  }


  // --------------------------------------------------------
  // Write the new positions
  digitalWrite(MOTOR_INA, reverse ? HIGH : LOW);
  digitalWrite(MOTOR_INB, reverse ? LOW : HIGH);
  analogWrite(MOTOR_PWM, throttle);

  myservo.write(servo);

  vPow = (float)readVcc() / 1000.0;

  float cell1 = (analogRead(CELL1_VOLTAGE) * vPow) / 1024.0;
  float v = (analogRead(TOTAL_VOLTAGE) * vPow) / 1024.0;
  float total = v / (r2 / (r1 + r2));
  float cell2 = total - cell1;

  Serial.print(SEP); Serial.print(servo);
  Serial.print(SEP); Serial.print(throttle);
  Serial.print(SEP); Serial.print(reverse);
  Serial.print(SEP); Serial.print(vPow);
  Serial.print(SEP); Serial.print(cell1);
  Serial.print(SEP); Serial.print(cell2);
  Serial.print(SEP); Serial.print(total);
  Serial.print(SEP); Serial.print(analogRead(BATTERY_CURRENT));
  Serial.print(SEP); Serial.println(analogRead(MOTOR_CURRENT));

  SoftwareServo::refresh();
}


// From https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
long readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
