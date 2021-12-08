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
  //Take input signal, convert to PWM signal//
  PWM_L = pulseIn(RCleft, HIGH);
  Serial.println(PWM_L);
  delay(2000);
  PWM_L = map(PWM_L, 1915, 1073, 0, 255);
  Serial.println(PWM_L);
  delay(2000);

/*
  while(PWM_L >= 115 && PWM_L <= 140)  //while LEFT joystick is centered (neither fwd/reverse)
  {
    analogWrite(ENB, 0);
    break;
  }
  
  while(PWM_L < 115) //while LEFT joystick is in reverse, set pin 31 & 33 HIGH
  {
    PORTC = B01010000; 
    if(PWM_L <= 70)
    {
      analogWrite(ENB, 0);	
      break;
    }
    else
    {
      PWM_L = map(PWM_L,
      analogWrite(ENB, PWM_L);
      break;
    }
  }
  
  while(PWM_L > 130)
  {
    PORTC = B10100000;
    analogWrite(ENB, PWM_L);
    break;
  }
*/
}
