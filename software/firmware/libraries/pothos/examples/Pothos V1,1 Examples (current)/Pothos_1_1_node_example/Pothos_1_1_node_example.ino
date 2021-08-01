/************************************************************
*Title:							Pothos_1_1_Node_Example.ino
*Author:						Anthony Grana
*Purpose:						To give an example of simple node communicating with pothos
*Description:				This Code can be run on a node to communicate over pothos.
*										Through the pothos protocol, This node will relay data from a temperature sensor, and will change the state of an LED.
*************************************************************/


#define RS485 Serial3         //Select RS-485 Serial Port. Reference https://www.pjrc.com/teensy/card7a_rev1.png to find serial port for your pinout.
#define POTHOS_DEBUG          //Comment this out for extra efficiency. Leave as is for verbose debug statements over USB.
#include <pothos.h>           //Include pothos library

enum HARDWARE{                //HARDWARE enum for recording where all of the pins are connected.
	ENABLE = 33,                    //RS-485 broadcast enable
	RGB_R = 1,                      //RGB LED pins (logic low)
	RGB_G = 32,
	RGB_B = 6,
	BLED = 13,                      //msc LED pin
	TMP_PIN = A13                   //temperature sensor pin
};

enum DATA{                    //Enum the registers that will store pothos data
  LED = 0,                        //A register that will control an led
  TMP = 1                         //A register for storing and broadcasting temperature data
};

int pothos_timeout = 50;      //Set timeout (ms)
int slave_id = 2;             //Set slave ID



//            ID#         enable pin       timeout(ms)    RGB_LED_r pin    RGB_LED_g pin    RGB_LED_b pin
pothos comms(slave_id, HARDWARE::ENABLE, pothos_timeout, HARDWARE::RGB_R, HARDWARE::RGB_G, HARDWARE::RGB_B);       // setup Pothos class as "comms"

void setup() {
  comms.setup(115200);              //start pothos comms at baudrate 115200 (pretty fast)

  #ifdef POTHOS_DEBUG              //If debugging, start USB serial port at 115200 baud
  Serial.begin(115200);
  #endif

  pinMode(HARDWARE::BLED, OUTPUT);        //msc pinmode setting
  pinMode(HARDWARE::TMP_PIN, INPUT);

  set_data_types();                 //Pothos needs to know what data type to expect for each register so this function is necessary
}

void loop() {
  comms.update();                   //This does all of the pothos comms

  digitalWrite(HARDWARE::BLED, comms.data.get_char_data(DATA::LED));            //This is how you read data set by the master. In this case it is controlling an LED state

  comms.data.set_data(DATA::TMP, analogRead(HARDWARE::TMP_PIN));                //This is how to write to a register, in this case we're saving whatever data comes from the temp sensor
}


void set_data_types(){                          //This fiction is for setting the data type of each register
  comms.data.set_type(DATA::LED, "char");           //chars are actually unsigned 8bit integers in disquise and are the closest thing to a bool that's supported by pothos
  comms.data.set_type(DATA::TMP, "int");            //the temperature data is an int
	//comms.data.set_type(DATA::TIME_DATA, "long");     //longs are also supported. time is often a long.
	//comms.data.set_type(DATA::TMP_DATA, "float");     //floats are also supported
}
