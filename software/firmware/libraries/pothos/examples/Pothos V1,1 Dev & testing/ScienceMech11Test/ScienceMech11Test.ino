#define RS485 Serial3
#define POTHOS_DEBUG
#include <pothos.h>

enum HARDWARE{
	ENABLE = 2,
	RGB_R = 1,
	RGB_G = 32,
	RGB_B = 6,
	BLED = 13,
};

enum DATA{
  LED = 0,
  TIME= 1  
};

pothos comms(3, HARDWARE::ENABLE, 50, HARDWARE::RGB_R, HARDWARE::RGB_G, HARDWARE::RGB_B);

void setup() {
  comms.setup(115200);
  
  #ifndef POTHOS_DEBUG
  Serial.begin(9600);
  #endif
  
  pinMode(HARDWARE::BLED, OUTPUT);
  
  set_data_types();
}

void loop() {
  comms.update();
	
  digitalWrite(HARDWARE::BLED, comms.data.get_char_data(DATA::LED));
  
  comms.data.set_data(DATA::TIME, (long)(millis()));
}


void set_data_types(){
  comms.data.set_type(DATA::LED, "char");      //chars are actually unsigned 8bit integers in disquise and are the closest thing to a bool that's supported by pothos.
  comms.data.set_type(DATA::TIME, "long");
}
