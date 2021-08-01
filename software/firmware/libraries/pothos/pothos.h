/******************************************
*Pothos V1.1
*Author: Anthony Grana
*1/10/2020
*OSURC Mars Rover 2019 - 2020
******************************************/

#include <data_store.h>

#ifndef _POTHOS
#define _POTHOS
class pothos{
	private:
		int ID;
		int sync_counter;
		int enPin;
		bool LED;
		int R,G,B;
		int maxWrite;
		unsigned long timeout;
		unsigned long timeout_timer;
		void read();
		void write();
		void write_multiple();
		void sync();
		void setColor(int,int,int);
	public:
		data_store data;
		pothos(int, int, int);
		pothos(int, int, int, int, int, int);
		void setup(int);
		void update();
		bool synced;
		bool good_comms;
};

pothos::pothos(int SlaveID, int en, int time){
	ID = SlaveID;
	enPin = en;
	synced = true;
	LED = false;
	timeout = time;
}

pothos::pothos(int SlaveID, int en, int time, int red, int green, int blue){
	ID = SlaveID;
	enPin = en;
	synced = true;
	LED = true;
	R = red;
	G = green;
	B = blue;
	timeout = time;
}


void pothos::setup(int baud){
	pinMode(enPin,OUTPUT);
	digitalWrite(enPin,LOW);
	if(LED){
		pinMode(R,OUTPUT);
		pinMode(G,OUTPUT);
		pinMode(B,OUTPUT);
	}
	RS485.begin(baud);
	maxWrite = RS485.availableForWrite();
}

void pothos::setColor(int red,int green,int blue){
	analogWrite(R,255-red);
	analogWrite(G,255-green);
	analogWrite(B,255-blue);
}

void pothos::update(){
	if(synced){
		if(RS485.available()>4){
			if(LED)
				setColor(0,100,0);				//green if synced
			good_comms = true;
    		#ifdef POTHOS_DEBUG
			Serial.print("Heard Something   ");
			Serial.println(RS485.peek());
			#endif
			if(RS485.read() == 0x00){
				#ifdef POTHOS_DEBUG
				Serial.print("heard slave ID:    ");
				Serial.println(RS485.peek());
				#endif
				if(RS485.read() == ID){
					#ifdef POTHOS_DEBUG
					Serial.println("slave ID Match");
					#endif
					if(LED)
    					setColor(0,0,255);	//blue if recieved a packet
					int fc = RS485.read();
					switch(fc){
						case(1):
							read();
							break;
						case(2):
							write();
							break;
						case(3):
							write_multiple();
							break;
					}
					RS485.read();
					digitalWrite(enPin,HIGH);
					RS485.write(0xff);
					RS485.write(0xff);
					RS485.write(0xff);
					RS485.write(0xff);
					while(RS485.availableForWrite() != maxWrite)
						delayMicroseconds(1);
					delayMicroseconds(800);
					digitalWrite(enPin,LOW);
				}else{
					sync_counter = 0;
					sync();
				}
			}else {
				sync_counter = 0;
				sync();
			}
			timeout_timer = millis();
		}
	}else{
		sync();
	}
	if(millis() - timeout_timer < timeout){
		if(LED)
    		setColor(170,130,0);    //yellow if not synced
	}else{
		if(LED)
    		setColor(255,0,0);   // if there has been a timeout, set LED to red.
			good_comms = false;
	}
}

void pothos::read(){
	#ifdef POTHOS_DEBUG
	Serial.println("Reading!");
	#endif
	byte adrs[255];
	int length=0;
	while(!RS485.available()) delayMicroseconds(1);
	adrs[length] = RS485.read();
	unsigned long timer = millis();
	while(RS485.peek() != 255){
		if(RS485.available()){
			length++;
			#ifdef POTHOS_DEBUG
			Serial.print("getting adr ");
			Serial.println(RS485.peek());
			#endif
			adrs[length] = RS485.read();
			if(adrs[length] == 255){
				length = length -1;
				break;
			}
			timer = millis();
		}
		if(millis() - timer > timeout)
			break;
	}
	digitalWrite(enPin,HIGH);
	for(int i=0;i<=length;i++){
		#ifdef POTHOS_DEBUG
		Serial.print("Transmitting at adr ");
		Serial.print(adrs[i]);
		Serial.print(" : ");
		#endif
		int type = data.get_type(adrs[i]);

		switch(type){
			case(1):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_char_data(adrs[i]));
				#endif
				RS485.write(data.get_char_data(adrs[i]));
				break;
			case(2):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_int_data(adrs[i]));
				#endif
				for(int j=1;j>=0;j = j-1)
					RS485.write((byte)(0 | (data.get_int_data(adrs[i]) >> 8*(1-j))));
				break;
			case(3):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_long_data(adrs[i]));
				#endif
				for(int j=3;j>=0;j = j-1)
					RS485.write((byte)(0 | ((data.get_long_data(adrs[i]) >> 8*(3-j)))));
				break;
			case(4):
				#ifdef POTHOS_DEBUG
				Serial.println(data.get_float_data(adrs[i]),9);
				#endif
				float datf = data.get_float_data(adrs[i]);
				byte* dat_raw = (byte*)(&datf);
				for(int j=0;j<4;j++)
					RS485.write(dat_raw[j]);
				break;
		}
		RS485.write(byte(0x00));
	}
	RS485.write(0xff);
	#ifdef POTHOS_DEBUG
	Serial.println("\n");
	#endif
}

void pothos::write(){
	int reg = RS485.read();
	int len;
	int type = data.get_type(reg);
	#ifdef POTHOS_DEBUG
	Serial.println("Writing single!");
	Serial.print("data type: ");
	Serial.println(type);
	#endif
	if(type <= 2) len = type;                        //determine length of data
	else len = 4;
	#ifdef POTHOS_DEBUG
	Serial.print("register is : ");
	Serial.println(reg);
	Serial.print("length in bytes: ");
	Serial.println(len);
	if(len == 0) Serial.println("You are trying to access a register that has an unset type!!!!!!!!!!!!    ERROR!!!");
	#endif
	while(RS485.available()<=len) delayMicroseconds(1);                  //get trapped in a loop until all of the data has arived
	byte data_in[len];
	for(int i=0;i<len;i++){
		data_in[i] = RS485.read();              // recieve data as a byte array
		#ifdef POTHOS_DEBUG
		Serial.print("Byte ");
		Serial.print(i);
		Serial.print(": ");
		Serial.println(data_in[i]);
		#endif
	}
	data.set_data(reg,data_in);
	#ifdef POTHOS_DEBUG
	Serial.println("\n");
	#endif
}

void pothos::write_multiple(){
	#ifdef POTHOS_DEBUG
	Serial.println("Writing multiple!");
	#endif
	int reg = RS485.read();
	int len, type;
	while(reg != 0xff){
		type = data.get_type(reg);
		#ifdef POTHOS_DEBUG
		Serial.print("Writing to reg: ");
		Serial.println(reg);
		#endif
		if(type < 3) len = type;
		else len = 4;
		#ifdef POTHOS_DEBUG
		Serial.print("length in bytes: ");
		Serial.println(len);
		if(len == 0) Serial.println("You are trying to access a register that has not been set up!!!!!!!!!!!!    ERROR!!!");
		#endif
		while(RS485.available() <= len) delayMicroseconds(1);
		byte data_in[len];
		for(int i=0;i<len;i++){
			data_in[i] = RS485.read();              // recieve data as a byte array
			#ifdef POTHOS_DEBUG
			Serial.print("Byte ");
			Serial.print(i);
			Serial.print(": ");
			Serial.println(data_in[i]);
			#endif
		}
		data.set_data(reg,data_in);
		reg = RS485.read();
	}
	#ifdef POTHOS_DEBUG
	Serial.println("\n");
	#endif
}

void pothos::sync(){
	synced = false;
	if(sync_counter < 5){
		while(RS485.available()){
			if(RS485.read() == 0xff)
				sync_counter++;
			else sync_counter = 0;
		}
	}else{
		while(RS485.available()){
			if(RS485.peek() == 0x00){
				synced = true;
				break;
			}else RS485.read();
		}
	}
}

//0xff 0xff 0xff 0xff 0xff
#endif
