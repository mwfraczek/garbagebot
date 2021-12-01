int LED = 51;
int IRleft = 52;
int IRright = 53;
int ENAr = 2; //PWM output signal rear right 
int ENBr = 3; //PWM output signal rear left
int ENAf = 4; //PWM OS front left 
int ENBf = 5; //PWM OS front right
byte RCleft = 9;
byte RCright = 10;
int PWM_L;
int PWM_R;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(IRleft, INPUT);
  pinMode(IRright, INPUT);
  pinMode(RCleft, INPUT);
  pinMode(RCright, INPUT);
  DDRC = B11111111; //Rear
  DDRL = B11111111; //Front
  Serial.begin(9600);

}

void loop() {
  PORTC = B10100000;
  PORTL = B01010000;
  PWM_L = pulseIn(RCleft, HIGH);
  PWM_R = pulseIn(RCright, HIGH);
  PWM_L = map(PWM_L, 1915, 1073, 0, 255);
  PWM_R = map(PWM_R, 1915, 1073, 0, 255);
  constrain(ENBr, 0, 255);
  constrain(ENAf, 0, 255);
  
  while(PWM_L >= 110 && PWM_L <= 145)
  {
    analogWrite(ENBr, 0);
    analogWrite(ENAf, 0);
    break;
  }
  while(PWM_R >= 110 && PWM_R <= 145)
  {
    analogWrite(ENBf, 0);
    analogWrite(ENAr, 0);
    break;
  }
    while(PWM_L < 110)
  {
    PORTC = B01010000;
    PORTL = B10100000;
    PWM_L = map(PWM_L, 109, 0, 0, 255);
    analogWrite(ENBr, PWM_L);
    analogWrite(ENAf, PWM_L);
    break;
  }
    while(PWM_L > 145)
  {
    PORTC = B10100000;
    PORTL = B01010000;
    PWM_L = map(PWM_L, 146, 255, 0, 255);
    analogWrite(ENBr, PWM_L);
    analogWrite(ENAf, PWM_L);
    break;
  }
      while(PWM_R < 110)
  {
    PORTC = B01010000;
    PORTL = B10100000;
    PWM_R = map(PWM_R, 109, 0, 0, 255);
    analogWrite(ENBf, PWM_R);
    analogWrite(ENAr, PWM_R);
    break;
  }
    while(PWM_R > 145)
  {
    PORTC = B10100000;
    PORTL = B01010000;
    PWM_L = map(PWM_L, 146, 255, 0, 255);
    analogWrite(ENBf, PWM_R);
    analogWrite(ENAr, PWM_R);
    break;
  }
}
