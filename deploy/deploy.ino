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
  //PORTC = B10100000; //direction (***confirm is forward or reverse***)
  PWM_L = pulseIn(RCleft, HIGH);  //take input signals, convert to PWM signals
  PWM_L = map(PWM_L, 1915, 1073, 0, 255);
  constrain(ENB, 0, 255);
  
  while(PWM_L >= 120 && PWM_L <= 135)  //while LEFT joystick is centered (neither fwd/reverse)
  {
    analogWrite(ENB, 0);
    break;
  }
  
  while(PWM_L < 120) //while LEFT joystick is in reverse
  {
    PORTC = B01010000;
    if(PWM_L <= 70)
    {
      analogWrite(ENB, 0);	
      break;
    }
    else
    {
      //PORTC = B01010000; //pin 31 & 33 HIGH (REVERSE)
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
  Serial.println(PWM_L);
}
