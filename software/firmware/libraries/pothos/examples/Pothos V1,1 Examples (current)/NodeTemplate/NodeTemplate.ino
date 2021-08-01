#define RS485 Serial1         //Select RS-485 Serial Port.                //This needs to be changed based on the design
//#define POTHOS_DEBUG          //Comment this out for extra efficiency. Leave as is for verbose debug statements over USB.
#include <pothos.h>           //Include pothos library

enum PIN{             //Enum of pinouts
  EN485 = 0,                     //RS485 Enable Pin
  RGB_R = 1,                      //RGB LED pins (logic low)
  RGB_G = 11,
  RGB_B = 12,
  //BLUE_LED = 13,
};

enum REGISTER{     //Enum of register addresses
  //TEMP = 0,                   //example register 0 for reading temperature data
  //CURRENT = 1,                //example register 0 for reading current data
};

int pothosTimeout = 50;        //The recomended pothos timeout is 50 ms
uint8_t slaveID = 1;            //The slave ID for the node (1-255)

pothos comms(slaveID, PIN::EN485, pothosTimeout, PIN::RGB_R, PIN::RGB_G, PIN::RGB_B);         //init the pothos library

void setup(){
  comms.setup(500000);          //Pothos will communicate at a baudrate of 500Kbps      //This is high enough to ensure that data speeds will never be the bottleneck.

  #ifdef POTHOS_DEBUG
  Serial.begin(115200);
  #endif

  setPinModes();                                //Sets all pinmodes
  setDataTypes();                               //Sets all pothos data types
}

void loop(){
  comms.update();                               //maintains communication over pothos
}

void setPinModes(){                             //This function will set the pinmode of all non-pothos pins (exclude 485-enable, Rx, Tx, and RGBLED pins)
  //pinMode(PIN::BLUE_LED, OUTPUT);             //Sets the blue LED on pin 13 to an output
}

void setDataTypes(){                          //This function is for setting the data type of each register
  //comms.data.set_type(REGISTER::LED, "char");           //chars are actually unsigned 8bit integers in disquise and are the closest thing to a bool that's supported by pothos
  //comms.data.set_type(REGISTER::TMP, "int");            //the temperature data is an int
	//comms.data.set_type(REGISTER::TIME_DATA, "long");     //longs are also supported. time is often a long.
	//comms.data.set_type(REGISTER::TMP_DATA, "float");     //floats are also supported
}
