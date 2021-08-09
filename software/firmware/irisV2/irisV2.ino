////////// Includes //////////
#include "SBUS.h"
#include <pothos.h>
#include <Encoder.h>
#define RS485 Serial1
//#define POTHOS_DEBUG

////////// Hardware / Data Enumerations //////////

enum REGISTER{
  CH1 = 0,
  CH2 = 1,
  CH3 = 2,
  CH4 = 3,
  CH5 = 4,
  CH6 = 5,
  CH7 = 6,
  CH8 = 7,
  CH9 = 8,
  CH10 = 9,
  CH11 = 10,
  CH12 = 11,
  CH13 = 12,
  CH14 = 13,
  CH15 = 14,
  CH16 = 15,
  ENC1 = 16,
  ENC2 = 17
};

enum HARDWARE{
  BLUE_LED = 13,
  EN485 = 2,
  ENC1A = 11,
  ENC1B = 12,
  ENC2A = 14,
  ENC2B = 15,
  SEGEN = 24,
  SEG01 = 25,
  SEG11 = 26,
  SEG21 = 27,
  SEG31 = 28,
  SEG02 = 29,
  SEG12 = 30,
  SEG22 = 31,
  SEG32 = 32
};

////////// Global Variables //////////
uint8_t node_id = 0;
uint8_t pothosTimeout = 50;

//Sbus Channels
uint16_t channels[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//Sbus Variables
uint8_t failSafe;
uint16_t lostFrames = 0;
#define SBUS_HARDWARE_PORT Serial3

long Enc1_pos;
long ENC2_pos;

////////// Class Instantiations //////////
SBUS x8r(SBUS_HARDWARE_PORT);
pothos comms(node_id, HARDWARE::EN485, pothosTimeout);         //init the pothos library
Encoder Enc1(HARDWARE::ENC1A, HARDWARE::ENC1B);
Encoder Enc2(HARDWARE::ENC2A, HARDWARE::ENC2B);

void setup() {
    setup_hardware();
    x8r.begin();

    comms.setup(500000);

    #ifdef POTHOS_DEBUG
    Serial.begin(115200);
    #endif

    setDataTypes();
}

void loop() {
    comms.update();
    poll_sbus();
    poll_encoders();
    set_leds();
    register_data();
}

void setup_hardware(){
    pinMode(HARDWARE::ENC1A, INPUT);
    pinMode(HARDWARE::ENC1B, INPUT);
    pinMode(HARDWARE::ENC2A, INPUT);
    pinMode(HARDWARE::ENC2B, INPUT);

    pinMode(HARDWARE::SEGEN, OUTPUT);
    pinMode(HARDWARE::SEG01, OUTPUT);
    pinMode(HARDWARE::SEG11, OUTPUT);
    pinMode(HARDWARE::SEG21, OUTPUT);
    pinMode(HARDWARE::SEG31, OUTPUT);
    pinMode(HARDWARE::SEG02, OUTPUT);
    pinMode(HARDWARE::SEG12, OUTPUT);
    pinMode(HARDWARE::SEG22, OUTPUT);
    pinMode(HARDWARE::SEG32, OUTPUT);
    pinMode(HARDWARE::BLUE_LED, OUTPUT);
    // Set default pin states

    digitalWrite(HARDWARE::BLUE_LED, LOW);
}

void poll_sbus(){
    x8r.read(&channels[0], &failSafe, &lostFrames);
}

void set_leds(){
  int i = 0;
}

void poll_encoders(){
  Enc1_pos = Enc1.read();
  Enc2_pos = Enc2.read();
}

void register_data(){
  comms.data.set_data(REGISTER::ENC1, Enc1_Pos);
  comms.data.set_data(REGISTER::ENC2, Enc2_Pos);
  for(int i=REGISTER::CH1;i<=REGISTER::CH16;i++){
    comms.data.set_data(i, channels[i]);
  }
}

void setDataTypes(){                          //This function is for setting the data type of each register
  comms.data.set_type(REGISTER::CH1, "int");
  comms.data.set_type(REGISTER::CH2, "int");
  comms.data.set_type(REGISTER::CH3, "int");
  comms.data.set_type(REGISTER::CH4, "int");
  comms.data.set_type(REGISTER::CH5, "int");
  comms.data.set_type(REGISTER::CH6, "int");
  comms.data.set_type(REGISTER::CH7, "int");
  comms.data.set_type(REGISTER::CH8, "int");
  comms.data.set_type(REGISTER::CH9, "int");
  comms.data.set_type(REGISTER::CH10, "int");
  comms.data.set_type(REGISTER::CH11, "int");
  comms.data.set_type(REGISTER::CH12, "int");
  comms.data.set_type(REGISTER::CH13, "int");
  comms.data.set_type(REGISTER::CH14, "int");
  comms.data.set_type(REGISTER::CH15, "int");
  comms.data.set_type(REGISTER::CH16, "int");
  comms.data.set_type(REGISTER::ENC1, "long");
  comms.data.set_type(REGISTER::ENC2, "long");
}
