int ENA = 2; //PWM output signal right ****CHANGE TO PWM PIN!!! '2' is NOT PWM****
int ENB = 3; //PWM output signal left
byte RCleft = 9; //remote control left joystick 
byte RCright = 10; //remote control right joystick 
int PWM_L;
int PWM_R;

void setup() {
  pinMode(RCleft, INPUT);
  pinMode(RCright, INPUT); 
  DDRC = B11111111; //Port C enable 
  Serial.begin(9600);
}

void loop() {
  PWM_L = pulseIn(RCleft, HIGH); //take RC signal 
  Serial.println(PWM_L);

  if(PWM_L >= 1430 && PWM_L <= 1540)  // LEFT joystick is centered (neither fwd/reverse)
  {
    analogWrite(ENB, 0);
  }
  else if(PWM_L > 1540) //LEFT joystick is in reverse, set pins 31 & 33 HIGH, map PWM values, send signal
  {
    PORTC = B01010000;
    PWM_L = map(PWM_L, 1539, 1915, 55, 255);
    analogWrite(ENB, PWM_L);
  }
  else if(PWM_L < 1430) //LEFT joystick is forward, set pins 30 & 32 HIGH, map PWM values, send signal 
  {
    PORTC = B10100000;
    PWM_L = map(PWM_L, 1431, 1080, 55, 255);
    analogWrite(ENB, PWM_L);
  }
}
