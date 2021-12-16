int AN2 = 5;       //PIN -- PWM output (right) 
int AN1 = 6;       //PIN -- PWM output (left)
int RC_R = 10;     //PIN -- RC input   (right) 
int RC_L = 3;      //PIN -- RC input   (left) 
int PWM_R;         //SIGNAL RIGHT 
int PWM_L;         //SIGNAL LEFT

void setup() {
  pinMode(RC_R, INPUT);
  pinMode(RC_L, INPUT); 
  DDRC = B11111111; //Port C enable 
  Serial.begin(9600);
}

void loop() {
  PWM_L = pulseIn(RC_L, HIGH); //take RC input signals & print to serial monitor 
  PWM_R = pulseIn(RC_R, HIGH); 
  Serial.println(PWM_L);

  //****** LEFT joystick control **********//
  if(PWM_L >= 1430 && PWM_L <= 1540)  // LEFT joystick is centered (neither fwd/reverse)
  {
    analogWrite(AN1, 0);
  }
  else if(PWM_L > 1540) //LEFT joystick is in reverse, set pins 31 & 33 HIGH, map PWM values, send signal
  {
    PORTC = B01010000;
    PWM_L = map(PWM_L, 1539, 1915, 55, 255);
    analogWrite(AN1, PWM_L);
  }
  else if(PWM_L < 1430) //LEFT joystick is forward, set pins 30 & 32 HIGH, map PWM values, send signal 
  {
    PORTC = B10100000;
    PWM_L = map(PWM_L, 1431, 1080, 55, 255);
    analogWrite(AN1, PWM_L);
  }

  //*********** RIGHT joystick control *************//
  if(PWM_R >= 1430 && PWM_R <= 1540)  // LEFT joystick is centered (neither fwd/reverse)
  {
    analogWrite(AN2, 0);
  }
  else if(PWM_R > 1540) //RIGHT joystick is in reverse, set pins 31 & 33 HIGH, map PWM values, send signal
  {
    PORTC = B01010000;
    PWM_R = map(PWM_R, 1539, 1915, 55, 255);
    analogWrite(AN2, PWM_R);
  }
  else if(PWM_R < 1430) //RIGHT joystick is forward, set pins 30 & 32 HIGH, map PWM values, send signal
  {
    PORTC = B10100000;
    PWM_R = map(PWM_R, 1431, 1080, 55, 255);
    analogWrite(AN2, PWM_R);
  }	
}
