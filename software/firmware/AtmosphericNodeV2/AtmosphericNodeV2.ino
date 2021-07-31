#define RS485 Serial1         //Select RS-485 Serial Port.                //This needs to be changed based on the design
//#define POTHOS_DEBUG          //Comment this out for extra efficiency. Leave as is for verbose debug statements over USB.
//#define DEBUG              //uncomment this line for a serial print of sensor data
#include <pothos.h>           //Include pothos library
#include <Wire.h>          // Include I2C Comms Library
#include <Adafruit_VEML6070.h> // include sensor library (UV light)
#include <Adafruit_Si7021.h>   // include sensor library (Temp / Humidity)


enum PIN{             //Enum of pinouts
  EN485 = 5,                     //RS485 Enable Pin
  RGB_R = 32,                      //RGB LED pins (logic low)
  RGB_G = 25,
  RGB_B = 6,
  BLUE_LED = 13,                  //onboard blue LED
  WIND_SENSOR = A0,              // Anemometer sends analog voltage
  DUST_SENSOR = 23,              // Dust sensor sends PWM
  I2C_SCL = 19,                  // CO2, UV, and humid/temp sensors all on I2C
  I2C_SDA = 18
};

enum REGISTER{     //Enum of register addresses
  DUST = 0,                     // Pts/L        //unknown range
  UV = 1,                       // No units     //unknown range
  
                                //CO2 sensor datasheet: https://cdn.shopify.com/s/files/1/0406/7681/files/Manual-CU-1107-N-Dual-Beam-NDIR-CO2-Sensor-Cubic.pdf?v=1613695364
  CO2 = 2,                      // pts/mil  //Integer ranging from ranges from 0 to 5000
  CO2_SENSOR_STATUS = 3,        // 0: preheating, 1: normal operation, 2: operating issue, 3: out of FS, 5: not calibrated
  
  TEMP = 4,                     // degrees C +/- .4C   Floating point number. Ranges from -10C to 85C
  HUMIDITY = 5,                 // RH% +/- 3%   //ranges from 0%-80%
  WIND = 6                      // analog voltage. 0.4V (0 m/s wind) up to 2.0V (for 32.4m/s wind speed)
                                // ADC value will be converted to float representing wind speed assuming
                                // perfectly linear correlation between voltage and corresponding wind speed
                                // (datasheet seems to confirm this)
};

/*************************** GLOBAL VARIABLES **************************************/
int pothosTimeout = 50;          //The recomended pothos timeout is 50 ms
uint8_t slaveID = 12;            //The slave ID for the node (1-255)

//Dust Sensor Variables//
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//recomended sample time: 30s
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;


//Wind Sensor Variables//
float wind_ADC;

//CO2 Sensor Variables//
unsigned char buf[5];
int CO2_data;



/*************************** CLASS INSTANTIATION **************************************/
Adafruit_VEML6070 uv = Adafruit_VEML6070();                 //define Adafruit sensor objects as global variables to use Adafruit libraries
Adafruit_Si7021 tempAndHumidity = Adafruit_Si7021();
pothos comms(slaveID, PIN::EN485, pothosTimeout, PIN::RGB_R, PIN::RGB_G, PIN::RGB_B);         //init the pothos library





void setup(){
  comms.setup(500000);          //Pothos will communicate at a baudrate of 500Kbps      //This is high enough to ensure that data speeds will never be the bottleneck.

  #ifdef POTHOS_DEBUG
  Serial.begin(115200);
  #endif

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  setup_sensors();                             // does hardware setup for sensors
  setPinModes();                                //Sets all pinmodes
  setDataTypes();                               //Sets all pothos data types
}

void loop(){
  comms.update();                    //maintains communication over pothos
  update_sensor_data();              //update data for dust, wind, UV, CO2, temp, and humidity

  #ifdef DEBUG
  Serial.print("Dust: ");
  Serial.print(comms.data.get_float_data(REGISTER::DUST));
  Serial.print("pcs/L    UV: ");
  Serial.print(comms.data.get_int_data(REGISTER::UV));
  Serial.print("    humidity: ");
  Serial.print(comms.data.get_float_data(REGISTER::HUMIDITY));
  Serial.print("    temperature: ");
  Serial.print(comms.data.get_float_data(REGISTER::TEMP));
  Serial.print("C    CO2: ");
  Serial.print(comms.data.get_int_data(REGISTER::CO2));
  Serial.print("ppm    CO2 sensor status: ");
  Serial.print(comms.data.get_char_data(REGISTER::CO2_SENSOR_STATUS));
  Serial.print("    wind: ");
  Serial.print(comms.data.get_float_data(REGISTER::WIND));
  Serial.println("m/s");
  #endif
}


void setup_sensors(){
  
  starttime = millis();         //sets start time variable
  uv.begin(VEML6070_1_T);       // setup UV sensor using library
  #ifdef DEBUG
  if (!tempAndHumidity.begin())
    Serial.println("Sensor (Temperature & Humidity) failed to be found.");
  #else
  tempAndHumidity.begin();    // setup temp / humiditity sensor (Si7021) using library
  #endif

  Wire.begin();               // begin I2C comms for CO2 sensor
  
}


void setPinModes(){                             //This function will set the pinmode of all non-pothos pins (exclude 485-enable, Rx, Tx, and RGBLED pins)
  pinMode(PIN::BLUE_LED, OUTPUT);             //Sets the blue LED on pin 13 to an output
  pinMode(PIN::DUST_SENSOR, INPUT);
  pinMode(PIN::WIND_SENSOR, INPUT);
}

void setDataTypes(){                          //This function is for setting the data type of each register
  comms.data.set_type(REGISTER::DUST, "float");         //float representing pts/L of dust
  comms.data.set_type(REGISTER::UV, "int");             //int representing UV intensity. Unitless.
  comms.data.set_type(REGISTER::CO2, "int");            //int representing ppm CO2 from 0-5000
  comms.data.set_type(REGISTER::CO2_SENSOR_STATUS, "char");  // char representing CO2 sensor status
  comms.data.set_type(REGISTER::TEMP, "float");         //float representing degrees C -10 to 85 +/- 0.4C
  comms.data.set_type(REGISTER::HUMIDITY, "float");     //float representing humidity 0%-80%
  comms.data.set_type(REGISTER::WIND, "float");         //float representing wind speed 0 m/s to 32.4 m/s
}



// This function goes through each sensor and collects data from each
// All functions called are defined below
void update_sensor_data(){
  if(get_dust() >1)
    comms.data.set_data(REGISTER::DUST, concentration);
  get_humidity();
  get_temperature();
  get_CO2();
  get_UV();
  get_wind();
}

float get_dust() {
  duration = pulseIn(PIN::DUST_SENSOR, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  if ((millis() - starttime) > sampletime_ms) //if the sampel time == 30s
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0); // Integer percentage 0=>100
    concentration = (1000.0 / 283.0) * 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
    lowpulseoccupancy = 0;
    starttime = millis();
  }
  return concentration;
}

void get_humidity() {
  comms.data.set_data(REGISTER::HUMIDITY, tempAndHumidity.readHumidity());   // send humidity reading data to Pothos register
}

void get_temperature() {
  comms.data.set_data(REGISTER::TEMP, tempAndHumidity.readTemperature());  // send temp reading data to Pothos register
}

void get_UV(){
  comms.data.set_data(REGISTER::UV, uv.readUV());   // send UV reading data to Pothos register
}

void get_CO2() {
  // first write a measure command to the CO2 sensor
  // measure command is 0x01
  Wire.beginTransmission(0x31);               // start transmission to 0x31 (CO2 sensor slave address)
  Wire.write(0x01);                           // send command to take measurement
  Wire.endTransmission();

  // then read the resulting data from the CO2 sensor
  // the sensor echos command byte (0x01), then sends
  // 2 measurement bytes, status byte, and checksum byte (5 bytes sent total)
  Wire.requestFrom(0x31, 5);                     // request data from 0x31 (CO2 sensor slave address)

  int i = 0;
  while(Wire.available()) {   // read command, data1, data2, status, and checksum bytes (in this order)
    buf[i] = Wire.read();     // buf[1] and buf[2] now contain our data
    i++;
  }

  CO2_data = buf[1] * 256;          // convert the two data bytes into an integer
  CO2_data = CO2_data + buf[2];      // note that data is sent in big endian format

  comms.data.set_data(REGISTER::CO2, CO2_data);                      // send CO2 reading data to Pothos register
  comms.data.set_data(REGISTER::CO2_SENSOR_STATUS, (char) buf[3]);   // send CO2 sensor status to Pothos register
}

void get_wind() {
  // assuming linear relationship betweeen wind speed and voltage,
  // which is implied by datasheet.
  // Our max ADC value is 1023, voltage reference is 2.5 volts
  
  wind_ADC = analogRead(PIN::WIND_SENSOR);  // get analog reading from anemometer
  wind_ADC = wind_ADC - 163.84;                  // subtract 163.84 (0.4v) to remove base voltage offset of anemometer.
  wind_ADC = wind_ADC / 819.2;               // divide by 819.2 (maximum value: 2v ADC output) to normalize value between 0 and 1
  wind_ADC = wind_ADC * 32.4;                   // normalize WIND between 0 and 32.4, now this value represents wind speed in m/s!

  comms.data.set_data(REGISTER::WIND, wind_ADC);      //send wind speed float data to Pothos register
}
