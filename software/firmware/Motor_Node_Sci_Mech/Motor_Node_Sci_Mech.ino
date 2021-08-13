////////// Includes //////////
#include <ModbusRtu.h>

////////// Hardware / Data Enumerations //////////
enum HARDWARE {
    RS485_EN = 2,
    RS485_RX = 7,
    RS485_TX = 8,

    LIM_SWITCH_1 = A4,    // PREVIOUSLY MOTOR_CURRENT_SENSE
    MOTOR_DIRECTION = 19,
    MOTOR_PWM = 20,
    MOTOR_SLEEP = 21,
    LIM_SWITCH_2 = 22,     // PREVIOUSLY MOTOR_FAULT

    TEMP = A9,

    LED_RED = 1,
    LED_GREEN = 32,
    LED_BLUE = 6,

    LED_BLUE_EXTRA = 13
};

enum MODBUS_REGISTERS {
    DIRECTION = 0,  // Input
    SPEED = 1,      // Input
    SLEEP = 2,      // Input

    LIM1 = 3,    // Output (PREVIOUSLY CS)
    LIM2 = 4,      // Output (previously fault)

    //TEMPERATURE = 5        // Output
};

////////// Global Variables //////////
const uint8_t node_id = 2;
const uint8_t mobus_serial_port_number = 3;

uint16_t modbus_data[] = {0, 0, 0, 0, 0};
uint8_t num_modbus_registers = 0;

int8_t poll_state = 0;
bool communication_good = false;
uint8_t message_count = 0;

uint16_t rampdown_step = 2000;

////////// Class Instantiations //////////
Modbus slave(node_id, mobus_serial_port_number, HARDWARE::RS485_EN);
//CSTS sense(50,2.2,158,81,(3.3/1024));

void setup() {
    setup_hardware();

    num_modbus_registers = sizeof(modbus_data) / sizeof(modbus_data[0]);
    slave.begin(115200); // baud-rate at 19200
    slave.setTimeOut(150);

    Serial.begin(9600);
}

void loop() {
    poll_modbus();
    set_leds();
    set_motor();
    check_lim_switches();
}

void setup_hardware(){
    // Setup pins as inputs / outputs
    pinMode(HARDWARE::RS485_EN, OUTPUT);

    pinMode(HARDWARE::LIM_SWITCH_1, INPUT);
    pinMode(HARDWARE::MOTOR_DIRECTION, OUTPUT);
    pinMode(HARDWARE::MOTOR_PWM, OUTPUT);
    pinMode(HARDWARE::MOTOR_SLEEP, OUTPUT);
    pinMode(HARDWARE::LIM_SWITCH_2, INPUT);

    pinMode(HARDWARE::TEMP, INPUT);

    pinMode(HARDWARE::LED_RED, OUTPUT);
    pinMode(HARDWARE::LED_GREEN, OUTPUT);
    pinMode(HARDWARE::LED_BLUE, OUTPUT);

    pinMode(HARDWARE::LED_BLUE_EXTRA, OUTPUT);

    // Set default pin states
    digitalWrite(HARDWARE::MOTOR_SLEEP, HIGH);

    digitalWrite(HARDWARE::LED_RED, LOW);
    digitalWrite(HARDWARE::LED_GREEN, HIGH);
    digitalWrite(HARDWARE::LED_BLUE, HIGH);

    digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);

    // Set the PWM resolution to 16-bits
    analogWriteResolution(16);

    // Change motor PWM frequency so it's not in the audible range
    analogWriteFrequency(HARDWARE::MOTOR_PWM, 25000);

    // Set teensy to increased analog resolution
    analogReadResolution(13);
}

void poll_modbus(){
    poll_state = slave.poll(modbus_data, num_modbus_registers);
    communication_good = !slave.getTimeOutState();
}

void set_leds(){
    if(poll_state > 4){
        message_count++;
        if(message_count > 2){
            digitalWrite(HARDWARE::LED_BLUE_EXTRA, !digitalRead(HARDWARE::LED_BLUE_EXTRA));
            message_count = 0;
        }

        digitalWrite(HARDWARE::LED_GREEN, LOW);
        digitalWrite(HARDWARE::LED_RED, HIGH);
    }else if(!communication_good){
        digitalWrite(HARDWARE::LED_BLUE_EXTRA, LOW);
        digitalWrite(HARDWARE::LED_GREEN, HIGH);
        digitalWrite(HARDWARE::LED_RED, LOW);
    }
}

void set_motor(){
    if(communication_good){
        digitalWrite(HARDWARE::MOTOR_DIRECTION, modbus_data[MODBUS_REGISTERS::DIRECTION]);
        analogWrite(HARDWARE::MOTOR_PWM, modbus_data[MODBUS_REGISTERS::SPEED]);
        digitalWrite(HARDWARE::MOTOR_SLEEP, modbus_data[MODBUS_REGISTERS::SLEEP]);
    }else{
        while(modbus_data[MODBUS_REGISTERS::SPEED] != 0 && modbus_data[MODBUS_REGISTERS::SPEED] > rampdown_step){
            modbus_data[MODBUS_REGISTERS::SPEED] -= rampdown_step;
            analogWrite(HARDWARE::MOTOR_PWM, modbus_data[MODBUS_REGISTERS::SPEED]);
            delay(2);
        }

        modbus_data[MODBUS_REGISTERS::SPEED] = 0;
        analogWrite(HARDWARE::MOTOR_PWM, modbus_data[MODBUS_REGISTERS::SPEED]);
    }

}

void check_lim_switches(){
  
  modbus_data[MODBUS_REGISTERS::LIM1] = digitalRead(HARDWARE::LIM_SWITCH_1);
  modbus_data[MODBUS_REGISTERS::LIM2] = digitalRead(HARDWARE::LIM_SWITCH_2);
  
}
