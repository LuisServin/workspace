byte pwm1Pin = 25;
byte en1Pin = 26;
byte en2Pin = 27;

byte encM1pin = 7;

volatile long encPos1 = 0;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(en1Pin, OUTPUT);
  pinMode(en2Pin, OUTPUT);
  
  pinMode(encM1pin, INPUT);
  
  attachInterrupt(encM1pin, funcInt, CHANGE);
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(en1Pin, HIGH);
  digitalWrite(en2Pin, LOW);
  analogWrite(pwm1Pin, 200);
  if(millis() % 300 == 0) {
    Serial.println(encPos1);
    encPos1 = 0;
    delay(1);
  }
}

void funcInt() {
  encPos1 += 1;
}

