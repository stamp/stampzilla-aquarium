#include <SoftTimer.h>
#include <DelayRun.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <dht.h>

OneWire  cover_ds(22);// kåpa
OneWire  water_ds(24);// vatten
dht DHT;
unsigned long lowLevelFilter;
unsigned long pumpTime;
unsigned long pumpTimeStamp;
unsigned long serialTimeStamp;

unsigned long doseTimeStamp1;
unsigned long doseTimer1;

double temperature;
double coverTemperature;

double red;
double green;
double blue;
double white;


struct pinsetup {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t white;
    uint8_t coverCooling;

    uint8_t waterTemperature;
    uint8_t waterLevel;
    uint8_t waterLevel2;
    uint8_t waterCooling;
    uint8_t waterCooling2;
    uint8_t waterCoolingRelay;

    uint8_t circulationPumps;
    uint8_t skimmer;
    uint8_t heating;
    uint8_t topupPump;

    uint8_t eTape;

    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
    uint8_t button1_led;
    uint8_t button2_led;
    uint8_t button3_led;

    uint8_t dose[6];

};
pinsetup pin;

int error;
int cnt;
double Setpoint, coolingLevel, coverCoolingLevel;
double Kp=10, Ki=0.1, Kd=0;
PID coolingPID(&temperature, &coolingLevel, &Setpoint,Kp,Ki,Kd, REVERSE);


boolean turnOnPumps(Task* task) {
    digitalWrite(pin.circulationPumps, true); // Turn pumps back on again
    return true;
}

DelayRun restartPumps(30UL*60UL*1000UL, turnOnPumps); // 30 minutes

void feeding() {
    digitalWrite(pin.circulationPumps, false); // Turn off pumps
    SoftTimer.remove(&restartPumps); // Reset previous timer
    restartPumps.startDelayed(); // Start the delayed task to turn the pumps back on
}


float getTemp(){// {{{
 //returns the temperature from one DS18S20 in DEG Celsius

 byte data[12];
 byte addr[8];

 if ( !water_ds.search(addr)) {
   //no more sensors on chain, reset search
   water_ds.reset_search();
   return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   return -1001;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   return -1002;
 }

 water_ds.reset();
 water_ds.select(addr);
 water_ds.write(0x44,0); // start conversion, with parasite power on at the end

// byte present = 
 water_ds.reset();
 water_ds.select(addr);  
 water_ds.write(0xBE); // Read Scratchpad

 
 for (int i = 0; i < 9; i++) { // we need 9 bytes
  data[i] = water_ds.read();
 }
 
 water_ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 return TemperatureSum;
}// }}}
float getTemp_cover(){// {{{
 //returns the temperature from one DS18S20 in DEG Celsius

 byte data[12];
 byte addr[8];

 if ( !cover_ds.search(addr)) {
   //no more sensors on chain, reset search
   cover_ds.reset_search();
   return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   return -1001;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   return -1002;
 }

 cover_ds.reset();
 cover_ds.select(addr);
 cover_ds.write(0x44,0); // start conversion, with parasite power on at the end

// byte present = 
 cover_ds.reset();
 cover_ds.select(addr);  
 cover_ds.write(0xBE); // Read Scratchpad

 
 for (int i = 0; i < 9; i++) { // we need 9 bytes
  data[i] = cover_ds.read();
 }
 
 cover_ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 return TemperatureSum;
}// }}}
bool readSerialCommand() {/*{{{*/

  if (Serial.available() < 4) {
    if ( Serial.available() > 0) {
      serialTimeStamp++;

      if ( serialTimeStamp > 50 ) {
         Serial.println("Serial, timeout. Clearing buffer");
         serialTimeStamp = 0;
         while(Serial.available() > 0) {
           Serial.read();
         }
      }
    } else {
      serialTimeStamp = 0;
    }
    return false;
  }

  if ( Serial.read() != 2 ) { // byte 0
    Serial.println("Serial, wrong start byte");
    return false;
  }

  byte command = Serial.read(); // byte 1
  byte value2 = Serial.read();   // byte 2
  byte value = Serial.read();   // byte 3

  int integer = value2;
  integer = integer * 256 + value;

  if ( Serial.read() != 3 ) { // byte 3
    Serial.println("Serial, wrong stop byte");
    return false;
  }

 /* Serial.print("Serial, accepted command:");
  Serial.print(command);
  Serial.print(":");
  Serial.println(value);*/

  switch(command) {
    case 1: // circulation pumps
        digitalWrite(pin.circulationPumps, value>0);
        break;
    case 2: // skimmer
        digitalWrite(pin.skimmer, value==0);
        break;
    case 3: // heating
        digitalWrite(pin.heating, value>0);
        break;
    case 4: // cooling P
        Kp = value;
        coolingPID.SetTunings(Kp, Ki, Kd);
        break;
    case 5: // cooling I
        Ki = value;
        Ki /= 10;
        coolingPID.SetTunings(Kp, Ki, Kd);
        break;
    case 6: // cooling D
        Kd = value;
        coolingPID.SetTunings(Kp, Ki, Kd);
        break;
    case 7: //red
        red = constrain(value,0,100);
        analogWrite(pin.red, red*2.55);break;
    case 8: //green
        green = constrain(value,0,100);
        analogWrite(pin.green, green*2.55);break;
    case 9: //blue
        blue = constrain(value,0,100);
        analogWrite(pin.blue, blue*2.55);break;
    case 10: //white
        white = constrain(value,0,100);
        analogWrite(pin.white, white*2.55);break;
    case 11: //cooling lvl
        coolingPID.SetMode(MANUAL);
        coolingPID.Compute();
        coolingLevel = constrain(value,0,100);
        coolingPID.SetMode(AUTOMATIC);
        coolingPID.Compute();
        break;
    case 12: //dose 1
        digitalWrite(pin.dose[0], LOW);
        delay(constrain(integer,0,20000));
        digitalWrite(pin.dose[0], HIGH);
        break;
    case 13: //dose 2
        digitalWrite(pin.dose[1], LOW);
        delay(constrain(integer,0,20000));
        digitalWrite(pin.dose[1], HIGH);
        break;
    case 14: //dose 3
        digitalWrite(pin.dose[2], LOW);
        delay(constrain(integer,0,20000));
        digitalWrite(pin.dose[2], HIGH);
        break;
    case 15: //dose 4
        digitalWrite(pin.dose[3], LOW);
        delay(constrain(integer,0,20000));
        digitalWrite(pin.dose[3], HIGH);
        break;
    case 16: //dose 5
        digitalWrite(pin.dose[4], LOW);
        delay(constrain(integer,0,20000));
        digitalWrite(pin.dose[4], HIGH);
        break;
    case 17: //dose 6
        digitalWrite(pin.dose[5], LOW);
        delay(constrain(integer,0,20000));
        digitalWrite(pin.dose[5], HIGH);
        /*Serial.print("Dose timer set: ");
        Serial.print(doseTimer1);
        Serial.print(" - ");
        Serial.print(value);
        Serial.print(" - ");
        Serial.print(value2);
        Serial.print(" - ");
        Serial.println(integer);*/
        break;
  }

  return true;
}/*}}}*/


void mainLoop(Task* me);
Task mainTask(0, mainLoop);

void setup() {
  pin.red = 5;
  pin.green = 6;
  pin.blue = 2;
  pin.white = 3;
  pin.coverCooling = 4;

  pin.waterTemperature = 24;
  pin.waterLevel = 25;
  pin.waterLevel2 = 26;
  pin.waterCooling = 9;
  pin.waterCooling2 = 10;
  pin.waterCoolingRelay = 7;

  pin.circulationPumps = 41; // True = off
  pin.skimmer = 40;
  pin.heating = 39;
  pin.topupPump = 38;

  pin.button1 = A0;
  pin.button2 = A1;
  pin.button3 = A2;
  pin.button1_led = 44;
  pin.button2_led = 45;
  pin.button3_led = 46;

  pin.eTape = A3;
 
  pin.dose[0] = 27;
  pin.dose[1] = 28;
  pin.dose[2] = 29;
  pin.dose[3] = 30;
  pin.dose[4] = 31;
  pin.dose[5] = 32;


  //Serial.begin(9600);
  Serial.begin(115200);
  coolingPID.SetMode(AUTOMATIC);
  coolingPID.SetOutputLimits(0, 100);

  temperature = 25.5;
  Setpoint = 25.5;

  pinMode(pin.white, OUTPUT);
  pinMode(pin.blue, OUTPUT);
  pinMode(pin.red, OUTPUT);
  pinMode(pin.green, OUTPUT);
  pinMode(pin.coverCooling, OUTPUT);

  pinMode(pin.waterTemperature ,INPUT);
  pinMode(pin.waterLevel, INPUT);
  pinMode(pin.waterLevel2, INPUT);
  pinMode(pin.waterCooling, OUTPUT);
  pinMode(pin.waterCooling2, OUTPUT);
  pinMode(pin.waterCoolingRelay, OUTPUT);

  pinMode(pin.circulationPumps, OUTPUT);
  pinMode(pin.skimmer, OUTPUT);
  pinMode(pin.heating, OUTPUT);
  pinMode(pin.topupPump, OUTPUT);

  pinMode(pin.button1, INPUT);
  pinMode(pin.button2, INPUT);
  pinMode(pin.button3, INPUT);
  pinMode(pin.eTape, INPUT);
  pinMode(pin.button1_led, OUTPUT);
  pinMode(pin.button2_led, OUTPUT);
  pinMode(pin.button3_led, OUTPUT);

  digitalWrite(pin.dose[0], HIGH);
  digitalWrite(pin.dose[1], HIGH);
  digitalWrite(pin.dose[2], HIGH);
  digitalWrite(pin.dose[3], HIGH);
  digitalWrite(pin.dose[4], HIGH);
  digitalWrite(pin.dose[5], HIGH);

  pinMode(pin.dose[0], OUTPUT);
  pinMode(pin.dose[1], OUTPUT);
  pinMode(pin.dose[2], OUTPUT);
  pinMode(pin.dose[3], OUTPUT);
  pinMode(pin.dose[4], OUTPUT);
  pinMode(pin.dose[5], OUTPUT);

  pinMode(23, INPUT);
  digitalWrite(23, HIGH);

  digitalWrite(pin.circulationPumps, true);
  digitalWrite(pin.skimmer, false);
  digitalWrite(pin.heating, true);
  digitalWrite(pin.waterCoolingRelay, false);
  digitalWrite(pin.topupPump, true);

  analogWrite(pin.red,0);
  analogWrite(pin.green,0);
  analogWrite(pin.blue,0);
  analogWrite(pin.white,0);

  SoftTimer.add(&mainTask);
}

void mainLoop(Task* me) {
  cnt++;
  if ( readSerialCommand() ) {
    return;
  }

  // Make an average on the temperature
  double t = getTemp();
  if ( t > 15 && t < 40 ) {
    temperature = temperature * 0.995 + t * 0.005;
  }
  coolingPID.Compute();

  // Check the water level, if to low start a count to filter waves
  // Also make sure that we have enough time left on the millis counter to do the cycle
  if ( !digitalRead(pin.waterLevel) && digitalRead(pin.waterLevel2) && millis() < (4294967295-30000) ) {
    if ( lowLevelFilter == 0 ) {
        lowLevelFilter = millis();
    }
  } else {
    lowLevelFilter = 0;
    pumpTimeStamp = 0;
    pumpTime = 0;
  }

  // When the sensor have been off for 10000ms
  if ( lowLevelFilter > 0 && (millis() - lowLevelFilter) > 10000 ) {
    if ( pumpTime == 0 ) {
      pumpTimeStamp = millis();
    }
  }

  // Run the pump, and stop after a limited time
  if ( pumpTimeStamp > 0 && pumpTime < 20000 ) {
    pumpTime = millis() - pumpTimeStamp;
    digitalWrite(pin.topupPump,false);
    digitalWrite(pin.button2_led, false);
  } else {
    if (pumpTime >= 20000 ) {
        error = 3; // Fill error
    }
      if (!digitalRead(pin.button2) && pumpTimeStamp > 0 ) {
        digitalWrite(pin.button2_led, false);
        digitalWrite(pin.topupPump,false); // turn on
      } else {
        digitalWrite(pin.button2_led, true);
        digitalWrite(pin.topupPump,true);
      }
  }

/*
    if ( doseTimeStamp1 == 0 && doseTimer1 > 0 && millis() < (4294967295-30000) ) {
        doseTimeStamp1 = millis() + doseTimer1;
        doseTimer1 = 0;
        digitalWrite(pin.skimmer, false);
    } else if (doseTimeStamp1 > 0 && doseTimeStamp1 < millis()) {
        digitalWrite(pin.skimmer, true);
        doseTimeStamp1 = 0;
    }
*/
  analogWrite(pin.waterCooling, coolingLevel*255/100);
  analogWrite(pin.waterCooling2, coolingLevel*255/100);
  analogWrite(pin.coverCooling,coverCoolingLevel);
  
  /*if (white>0 || red>0 || blue>0 || green>0) {*/
      /*analogWrite(pin.coverCooling,255);*/
  /*} else {*/
      /*analogWrite(pin.coverCooling,0);*/
  /*}*/
  if (coolingLevel > 1) {
    digitalWrite(pin.waterCoolingRelay, false); //Enable cooling relay
  } else if(coolingLevel <= 0){
    digitalWrite(pin.waterCoolingRelay, true); //Disable cooling relay
  }

  if (temperature < Setpoint - 0.5) {
      digitalWrite(pin.heating, true); // Turn on
  } else if(temperature > Setpoint - 0.1) {
      digitalWrite(pin.heating, false); // Turn off
  }

  if (temperature < Setpoint - 0.5) {
    error = 1; // underheat error
  }

  if (temperature > Setpoint + 0.5) {
    error = 2; // overheat errorn
  }

  coverTemperature = getTemp_cover();

  if (coverTemperature > 34 && coverCoolingLevel < 255) {
    coverCoolingLevel += 0.05;
  }
  if (coverTemperature < 30 && coverCoolingLevel > 0) {
    coverCoolingLevel -= 0.05;
  }


  Serial.print(temperature);
  Serial.print("|");
  Serial.print(pumpTime);
  Serial.print("|");
  Serial.print(coolingLevel);
  Serial.print("|");

  byte bits = 0;
  boolean pumps = digitalRead(pin.circulationPumps);
  bits |= 0x01 * pumps;
  bits |= 0x02 * !digitalRead(pin.skimmer);
  bits |= 0x04 * digitalRead(pin.heating);
  bits |= 0x08 * digitalRead(pin.topupPump);
  bits |= 0x10 * !digitalRead(pin.waterLevel);
  bits |= 0x20 * !digitalRead(pin.waterLevel2);

  Serial.write(bits);
  Serial.print("|");
  Serial.print(t); // Last sensor value
  Serial.print("|");
  Serial.print(Kp);
  Serial.print(":");
  Serial.print(Ki);
  Serial.print(":");
  Serial.print(Kd);
  Serial.print("|");
  Serial.print(red);
  Serial.print("*");
  Serial.print(green);
  Serial.print("*");
  Serial.print(blue);
  Serial.print("*");
  Serial.print(white);
  Serial.print("|");
  Serial.print(coverTemperature);
  Serial.print("|");
  Serial.print(analogRead(A4));
  Serial.print(":");
  Serial.print(analogRead(A5));
  Serial.print(":");
  Serial.print(analogRead(A7));
  Serial.print("|");

  int chk = DHT.read22(23);
  switch (chk)
  {
    case DHTLIB_OK:  
        Serial.print(DHT.humidity, 1);
        Serial.print(",");
        Serial.print(DHT.temperature, 1);
        break;
    case DHTLIB_ERROR_CHECKSUM: 
        Serial.print("Checksum error"); 
        break;
    case DHTLIB_ERROR_TIMEOUT: 
        Serial.print("Time out error"); 
        break;
    default: 
        Serial.print("Unknown error"); 
        break;
  }
  Serial.print("|");
  Serial.print(coverCoolingLevel);
  Serial.print("|");
  Serial.print(analogRead(pin.eTape));
  Serial.print("|");
  Serial.print(error);

  Serial.print("\r\n");


  digitalWrite(pin.button1_led, error>0);
  if ( digitalRead(pin.button1) ) {
    error = 0;
  }



  if (pumps) {
      if ( digitalRead(pin.button3) ) { // Activate feeding if you push the button and pumps are on
        feeding();
      }
      digitalWrite(pin.button3_led, !(cnt & 0x1B)); // 16+8+4
  } else {
      digitalWrite(pin.button3_led, (cnt & 0x1B)); // 16+8+4 - long on, short off
  }
}


