#define RS485 Serial1         //Select RS-485 Serial Port.                //This needs to be changed based on the design
//#define POTHOS_DEBUG          //Comment this out for extra efficiency. Leave as is for verbose debug statements over USB.
#include <pothos.h>           //Include pothos library

enum PIN{             //Enum of pinouts
  EN485 = 0,                     //RS485 Enable Pin
  RGB_R = 1,                      //RGB LED pins (logic low)
  RGB_G = 11,
  RGB_B = 12,
  PWM = A2,
  CS = A3,
  INA = 4,
  INB = 5,
  SEL = 8,
  TEMP = A9
};

enum REGISTER{     //Enum of register addresses
  SPEED = 0,
  DIR = 1,
  TMP = 2,                   //example register 0 for reading temperature data
  CURRENT = 3,                //example register 0 for reading current data
};

uint32_t updateTimer = 0;
int updateTime = 50;

int pothosTimeout = 200;        //The recomended pothos timeout is 50 ms
uint8_t slaveID = 1;            //The slave ID for the node

pothos comms(slaveID, PIN::EN485, pothosTimeout, PIN::RGB_R, PIN::RGB_G, PIN::RGB_B);         //init the pothos library

void setup(){
  comms.setup(500000);          //Pothos will communicate at a baudrate of 500Kbps      //This is high enough to ensure that data speeds will never be the bottleneck.

  #ifndef POTHOS_DEBUG
  Serial.begin(115200);
  #endif

  setPinModes();                                //Sets all pinmodes
  setDataTypes();                               //Sets all pothos data types
}

void loop(){
  comms.update();                               //maintains communication over pothos
  if(millis()-updateTimer >= updateTime){       //update timer to trigger at 20Hz, This saves a vast majority of processing power for watching communicatrions
    updateTimer = millis();
    driveMotors();
    comms.data.set_data(REGISTER::TMP, readTemp());
    comms.data.set_data(REGISTER::CURRENT, readCurrent());
  }
}

void setPinModes(){                             //This function will set the pinmode of all non-pothos pins (exclude 485-enable, Rx, Tx, and RGBLED pins)
                                                //Sets the blue LED on pin 13 to an output
  pinMode(PIN::PWM,OUTPUT);
  pinMode(PIN::INA,OUTPUT);
  pinMode(PIN::INB,OUTPUT);
  pinMode(PIN::SEL,OUTPUT);
  
  pinMode(PIN::CS,INPUT);
  pinMode(PIN::TEMP,INPUT);
}

void setDataTypes(){                          //This function is for setting the data type of each register

  
  //comms.data.set_type(REGISTER::LED, "char");           //chars are actually unsigned 8bit integers in disquise and are the closest thing to a bool that's supported by pothos
  //comms.data.set_type(REGISTER::TMP, "int");            //the temperature data is an int
	//comms.data.set_type(REGISTER::TIME_DATA, "long");     //longs are also supported. time is often a long.
	//comms.data.set_type(REGISTER::TMP_DATA, "float");     //floats are also supported

  comms.data.set_type(REGISTER::SPEED, "char");
  comms.data.set_type(REGISTER::DIR, "char");
  comms.data.set_type(REGISTER::TMP, "float");
  comms.data.set_type(REGISTER::CURRENT, "float");
}

float readTemp(){
  int raw = analogRead(PIN::TEMP);
  float tempC = (((raw/1023.0)*3.3)-0.4)/.0195;
  return(tempC);
}

void driveMotors(){
    bool direct = (comms.data.get_char_data(REGISTER::DIR) != '\0');
    uint8_t motorSpeed = int(comms.data.get_char_data(REGISTER::SPEED));
    digitalWrite(PIN::INA, direct);
    digitalWrite(PIN::SEL, direct);
    digitalWrite(PIN::INB, !direct);
    analogWrite(PIN::PWM, motorSpeed);
}

float readCurrent(){                                    //current reading is only accurate at > 200mA
  int raw = analogRead(PIN::CS);
  float current = ((raw/1023.0)*3.3)*(112.0/43.0);
  return(current);
}
