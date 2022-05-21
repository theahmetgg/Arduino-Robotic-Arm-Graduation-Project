#include <Servo.h>

bool repeatePlaying = false; 
int delayBetweenCycles = 2000; 

int basePin = 11;       
int shoulderPin = 10;   
int elbowPin = 9;       
int gripperPin = 6;    

int xdirPin = 0;        
int ydirPin = 1;        
int zdirPin = 3;        
int gdirPin = 2;        

//int pinRecord = A4;    
//int pinPlay = A5;      
int pinRecord = PD2;     
int pinPlay = PD3;       
int pinLedRecord = PD4;  

bool useInternalPullUpResistors = false;

const int buffSize = 512; 
int startBase = 0;
int startShoulder = 0;
int startElbow = 0;
int startGripper = 0;

int posBase = 90;
int posShoulder = 90;
int posElbow = 90;
int posGripper = 0;

int lastBase = 90;
int lastShoulder = 90;
int lastElbow = 90;
int lastGripper = 90;

int minBase = 0;
int maxBase = 150;
int minShoulder = 0;
int maxShoulder = 150;
int minElbow = 0;
int maxElbow = 150;
int minGripper = 0;
int maxGripper = 150;

const int countServo = 4;
int buff[buffSize];
int buffAdd[countServo];
int recPos = 0;
int playPos = 0;

int buttonRecord = HIGH;
int buttonPlay = HIGH;

int buttonRecordLast = LOW;
int buttonPlayLast = LOW;

bool record = false;
bool play = false;
bool debug = false;

String command = "Manual";
int printPos = 0;

int buttonPlayDelay = 20;
int buttonPlayCount = 0;

bool ledLight = false;

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

void setup() {
  Serial.begin(9600);

  if (useInternalPullUpResistors) {
    pinMode(pinRecord, INPUT_PULLUP);
    pinMode(pinPlay, INPUT_PULLUP);
  }
  else
  {
    pinMode(pinRecord, INPUT);
    pinMode(pinPlay, INPUT);
  }

  pinMode(xdirPin, INPUT);
  pinMode(ydirPin, INPUT);
  pinMode(zdirPin, INPUT);
  pinMode(gdirPin, INPUT);

  pinMode(pinLedRecord, OUTPUT);

  servoBase.attach(basePin);
  servoShoulder.attach(shoulderPin);
  servoElbow.attach(elbowPin);
  servoGripper.attach(gripperPin);

  StartPosition();

  digitalWrite(pinLedRecord, HIGH);
  delay(1000);
  digitalWrite(pinLedRecord, LOW);
}

void loop() {

  buttonRecord = digitalRead(pinRecord);
  buttonPlay = digitalRead(pinPlay);

  //  Serial.print(buttonRecord);
  //  Serial.print("\t");
  //  Serial.println(buttonPlay);
  //  for testing purposes

  if (buttonPlay == LOW)
  {
    buttonPlayCount++;

    if (buttonPlayCount >= buttonPlayDelay)
    {
      repeatePlaying = true;
    }
  }
  else buttonPlayCount = 0;

  if (buttonPlay != buttonPlayLast)
  {
    if (record)
    {
      record = false;
    }

    if (buttonPlay == LOW)
    {
      play = !play;
      repeatePlaying = false;

      if (play)
      {
        StartPosition();
      }
    }
  }

  if (buttonRecord != buttonRecordLast)
  {
    if (buttonRecord == LOW)
    {
      record = !record;

      if (record)
      {
        play = false;
        repeatePlaying = false;
        recPos = 0;
      }
      else
      {
        if (debug) PrintBuffer();
      }
    }
  }

  buttonPlayLast = buttonPlay;
  buttonRecordLast = buttonRecord;

  float dx = map(analogRead(xdirPin), 0, 1023, -5.0, 5.0);
  float dy = map(analogRead(ydirPin), 0, 1023, 5.0, -5.0);
  float dz = map(analogRead(zdirPin), 0, 1023, 5.0, -5.0);
  float dg = map(analogRead(gdirPin), 0, 1023, 5.0, -5.0);

  if (abs(dx) < 1.5) dx = 0;
  if (abs(dy) < 1.5) dy = 0;
  if (abs(dz) < 1.5) dz = 0;
  if (abs(dg) < 1.5) dg = 0;

  posBase += dx;
  posShoulder += dy;
  posElbow += dz;
  posGripper += dg;

  if (play)
  {
    if (playPos >= recPos) {
      playPos = 0;

      if (repeatePlaying)
      {
        delay(delayBetweenCycles);
        StartPosition();
      }
      else
      {
        play = false;
      }
    }

    bool endOfData = false;

    while (!endOfData)
    {
      if (playPos >= buffSize - 1) break;
      if (playPos >= recPos) break;

      int data = buff[playPos];
      int angle = data & 0xFFF;
      int servoNumber = data & 0x3000;
      endOfData = data & 0x4000;

      switch (servoNumber)
      {
        case 0x0000:
          posBase = angle;
          break;

        case 0x1000:
          posShoulder = angle;
          break;

        case 0x2000:
          posElbow = angle;
          break;

        case 0x3000:
          posGripper = angle;
          dg = posGripper - lastGripper;
          break;
      }

      playPos++;
    }
  }

  if (posBase > maxBase) posBase = maxBase;
  if (posShoulder > maxShoulder) posShoulder = maxShoulder;
  if (posElbow > maxElbow) posElbow = maxElbow;
  if (posGripper > maxGripper) posGripper = maxGripper;

  if (posBase < minBase) posBase = minBase;
  if (posShoulder < minShoulder) posShoulder = minShoulder;
  if (posElbow < minElbow) posElbow = minElbow;
  if (posGripper < minGripper) posGripper = minGripper;

  servoBase.write(posBase);
  servoShoulder.write(posShoulder);
  servoElbow.write(posElbow);

  bool waitGripper = false;
  if (dg < 0) {
    posGripper = minGripper;
    waitGripper = true;
  }
  else if (dg > 0) {
    posGripper = maxGripper;
    waitGripper = true;
  }

  servoGripper.write(posGripper);
  if (play && waitGripper)
  {
    delay(1000);
  }

  if ((lastBase != posBase) | (lastShoulder != posShoulder) | (lastElbow != posElbow) | (lastGripper != posGripper))
  {
    if (record)
    {
      if (recPos < buffSize - countServo)
      {
        int buffPos = 0;

        if (lastBase != posBase)
        {
          buffAdd[buffPos] = posBase;
          buffPos++;
        }

        if (lastShoulder != posShoulder)
        {
          buffAdd[buffPos] = posShoulder | 0x1000;
          buffPos++;
        }

        if (lastElbow != posElbow)
        {
          buffAdd[buffPos] = posElbow | 0x2000;
          buffPos++;
        }

        if (lastGripper != posGripper)
        {
          buffAdd[buffPos] = posGripper | 0x3000;
          buffPos++;
        }

        buffAdd[buffPos - 1] = buffAdd[buffPos - 1] | 0x4000;

        for (int i = 0; i < buffPos; i++)
        {
          buff[recPos + i] = buffAdd[i];
        }

        recPos += buffPos;
      }
    }

    command = "Manual";
    printPos = 0;

    if (play)
    {
      command = "Play";
      printPos = playPos;
    }
    else if (record)
    {
      command = "Record";
      printPos = recPos;
    }

    Serial.print(command);
    Serial.print("\t");
    Serial.print(printPos);
    Serial.print("\t");
    Serial.print(posBase);
    Serial.print("\t");
    Serial.print(posShoulder);
    Serial.print("\t");
    Serial.print(posElbow);
    Serial.print("\t");
    Serial.print(posGripper);
    Serial.print("\t");
    Serial.print(record);
    Serial.print("\t");
    Serial.print(play);
    Serial.println();
  }

  lastBase = posBase;
  lastShoulder = posShoulder;
  lastElbow = posElbow;
  lastGripper = posGripper;

  if ( repeatePlaying)
  {
    ledLight = !ledLight;
  }
  else
  {
    if (ledLight)
    {
      ledLight = false;
    }

    if (record)
    {
      ledLight = true;
    }
  };

  digitalWrite(pinLedRecord, ledLight);
  delay(50);
}

void PrintBuffer()
{
  for (int i = 0; i < recPos; i++)
  {
    int data = buff[i];
    int angle = data & 0xFFF;
    int servoNumber = data & 0x3000;
    bool endOfData = data & 0x4000;

    Serial.print("Servo=");
    Serial.print(servoNumber);
    Serial.print("\tAngle=");
    Serial.print(angle);
    Serial.print("\tEnd=");
    Serial.print(endOfData);
    Serial.print("\tData=");
    Serial.print(data, BIN);
    Serial.println();
  }
}

void StartPosition()
{
  int angleBase = servoBase.read();
  int angleShoulder = servoShoulder.read();
  int angleElbow = servoElbow.read();
  int angleGripper = servoGripper.read();

  Serial.print(angleBase);
  Serial.print("\t");
  Serial.print(angleShoulder);
  Serial.print("\t");
  Serial.print(angleElbow);
  Serial.print("\t");
  Serial.print(angleGripper);
  Serial.println("\t");

  posBase = startBase;
  posShoulder = startShoulder;
  posElbow = startElbow;
  posGripper = startGripper;

  servoBase.write(posBase);
  servoShoulder.write(posShoulder);
  servoElbow.write(posElbow);
  servoGripper.write(posGripper);
}
