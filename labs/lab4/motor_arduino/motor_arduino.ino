#define calibration_factor 0.85
#define pwm 255

void setup()
{
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
}

/////////// loop for oscilloscope testing //////////
// void loop()
// {
//   analogWrite(3, 0);
//   for (int i = 0; i < 255; i++)
//   {
//     analogWrite(3, i);
//     analogWrite(4, 0);
//     analogWrite(1, 255-i);
//     analogWrite(2, 0);
//     delay(10);
//   }
// }

///////////// both wheels going back and forward //////////////
// void loop()
// {
//   delay(1000);
//   analogWrite(1, 0);
//   analogWrite(2, pwm);
//   analogWrite(3, 0);
//   analogWrite(4, pwm * calibration_factor);

//   delay(2000);

//   analogWrite(1, 0);
//   analogWrite(2, 0);
//   analogWrite(3, 0);
//   analogWrite(4, 0);
//   delay(1000);

//   analogWrite(1, pwm);
//   analogWrite(2, 0);
//   analogWrite(3, pwm * calibration_factor);
//   analogWrite(4, 0);

//   delay(2000);

//   analogWrite(1, 0);
//   analogWrite(2, 0);
//   analogWrite(3, 0);
//   analogWrite(4, 0);
//   delay(1000);
// }

void loop()
{
  analogWrite(1, 0);
  analogWrite(2, pwm);
  analogWrite(3, 0);
  analogWrite(4, pwm * calibration_factor);

  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(4, 0);
  delay(5000);

  analogWrite(1, pwm);
  analogWrite(2, 0);
  analogWrite(3, pwm * calibration_factor);
  analogWrite(4, 0);
  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(4, 0);
  delay(5000);

  analogWrite(1, 0);
  analogWrite(2, pwm);
  analogWrite(3, 0);
  analogWrite(4, pwm * calibration_factor);
  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(4, 0);
  delay(5000);

  analogWrite(1, pwm);
  analogWrite(2, 0);
  analogWrite(3, pwm * calibration_factor);
  analogWrite(4, 0);
  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(4, 0);
  delay(10000);

}