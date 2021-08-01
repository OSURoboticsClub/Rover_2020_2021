#define RS485 Serial2         //Select RS-485 Serial Port.                //This needs to be changed based on the design
#define POTHOS_DEBUG          //Comment this out for extra efficiency. Leave as is for verbose debug statements over USB.
#include <pothos.h>           //Include pothos library
#include <Servo.h>

enum PIN{             //Enum of pinouts
  EN485 = 2,                     //RS485 Enable Pin
  RGB_R = 1,                      //RGB LED pins (logic low)
  RGB_G = 32,
  RGB_B = 6,
  TP_PWM1 = 22,
  TP_PWM2 = 20,
  TP_RX = 7,
  TP_TX = 8,
  TP_D1 = 15,
  TP_D2 = 14,
  TP_D3 = 12,
  TP_D4 = 11,
  SERVO_PAN = 5,
  SERVO_TILT = 4,
  SERVO_1 = 23,
  SERVO_2 = 21,
  BLUE_LED = 13,
};

enum REGISTER{     //Enum of register addresses
  SERVO_PAN_POS = 0,
  SERVO_TILT_POS = 1,
  SERVO_1_POS = 2,
  SERVO_2_POS = 3
};

int updateTime = 50;
unsigned long timer = 0;
int pothosTimeout = 200;        //The recomended pothos timeout is 50 ms
uint8_t slaveID = 8;            //The slave ID for the node (1-255)
uint8_t defTilt = 110;
uint8_t defPan = 100; 

pothos comms(slaveID, PIN::EN485, pothosTimeout, PIN::RGB_R, PIN::RGB_G, PIN::RGB_B);         //init the pothos library
Servo Pan;
Servo Tilt;
Servo SOne;
Servo STwo;


void setup(){
  comms.setup(500000);          //Pothos will communicate at a baudrate of 500Kbps      //This is high enough to ensure that data speeds will never be the bottleneck.

  #ifdef POTHOS_DEBUG
  Serial.begin(115200);
  #endif

  setPinModes();                                //Sets all pinmodes
  setDataTypes();                               //Sets all pothos data types
  comms.data.set_data(REGISTER::SERVO_PAN_POS, defPan);
  comms.data.set_data(REGISTER::SERVO_TILT_POS, defTilt);
}

void loop(){
  comms.update();                               //maintains communication over pothos
  
  if (millis() - timer >= updateTime) {
    timer = millis();
    setServos();
  }
  
}

void setPinModes(){                             //This function will set the pinmode of all non-pothos pins (exclude 485-enable, Rx, Tx, and RGBLED pins)
  //pinMode(PIN::SERVO_PAN, OUTPUT);             //Sets the blue LED on pin 13 to an output  pinMode(PIN::SERVO_PAN, OUTPUT):            //seTS
  //pinMode(PIN::SERVO_TILT, OUTPUT);             //Sets the blue LED on pin 13 to an output
  //pinMode(PIN::SERVO_1, OUTPUT);             //Sets the blue LED on pin 13 to an output
  //pinMode(PIN::SERVO_2, OUTPUT);             //Sets the blue LED on pin 13 to an output
  Pan.attach(PIN::SERVO_PAN);
  Tilt.attach(PIN::SERVO_TILT);
  SOne.attach(PIN::SERVO_1);
  STwo.attach(PIN::SERVO_2);
}

void setDataTypes(){                          //This function is for setting the data type of each register
  //comms.data.set_type(REGISTER::LED, "char");           //chars are actually unsigned 8bit integers in disquise and are the closest thing to a bool that's supported by pothos
  //comms.data.set_type(REGISTER::TMP, "int");            //the temperature data is an int
	//comms.data.set_type(REGISTER::TIME_DATA, "long");     //longs are also supported. time is often a long.
	//comms.data.set_type(REGISTER::TMP_DATA, "float");     //floats are also supported
 
  comms.data.set_type(REGISTER::SERVO_PAN_POS, "int"); 
  comms.data.set_type(REGISTER::SERVO_TILT_POS, "int"); 
  comms.data.set_type(REGISTER::SERVO_1_POS, "int"); 
  comms.data.set_type(REGISTER::SERVO_2_POS, "int"); 
}


void setServos() {
  
  Pan.write(comms.data.get_int_data(REGISTER::SERVO_PAN_POS));
  Tilt.write(comms.data.get_int_data(REGISTER::SERVO_TILT_POS));
  SOne.write(comms.data.get_int_data(REGISTER::SERVO_1_POS));
  STwo.write(comms.data.get_int_data(REGISTER::SERVO_2_POS));
  
}
