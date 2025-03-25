#define calibration_factor 0.86
#define pwm 255

void setup()
{
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
}

/////////// loop for oscilloscope testing //////////
// void loop()
// {
//   analogWrite(16, 0);
//   for (int i = 0; i < 255; i++)
//   {
//     analogWrite(16, i);
//     analogWrite(15, 0);
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
//   analogWrite(16, 0);
//   analogWrite(15, pwm * calibration_factor);

//   delay(2000);

//   analogWrite(1, 0);
//   analogWrite(2, 0);
//   analogWrite(16, 0);
//   analogWrite(15, 0);
//   delay(1000);

//   analogWrite(1, pwm);
//   analogWrite(2, 0);
//   analogWrite(16, pwm * calibration_factor);
//   analogWrite(15, 0);

//   delay(2000);

//   analogWrite(1, 0);
//   analogWrite(2, 0);
//   analogWrite(16, 0);
//   analogWrite(15, 0);
//   delay(1000);
// }

void loop()
{
  analogWrite(1, 0);
  analogWrite(2, pwm);
  analogWrite(16, 0);
  analogWrite(15, pwm * calibration_factor);

  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(16, 0);
  analogWrite(15, 0);
  delay(5000);

  analogWrite(1, pwm);
  analogWrite(2, 0);
  analogWrite(16, pwm * calibration_factor);
  analogWrite(15, 0);
  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(16, 0);
  analogWrite(15, 0);
  delay(5000);

  analogWrite(1, 0);
  analogWrite(2, pwm);
  analogWrite(16, 0);
  analogWrite(15, pwm * calibration_factor);
  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(16, 0);
  analogWrite(15, 0);
  delay(5000);

  analogWrite(1, pwm);
  analogWrite(2, 0);
  analogWrite(16, pwm * calibration_factor);
  analogWrite(15, 0);
  delay(500);

  analogWrite(1, 0);
  analogWrite(2, 0);
  analogWrite(16, 0);
  analogWrite(15, 0);
  delay(10000);

}