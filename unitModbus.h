/**
 * @file 	plantModbus.h
 * @version     0.1
 * @date        2017May3
 * @author 	pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
*/


#include <hydroModbusCommon.h>


// specific read holding registers to unit
#define FUTURE CUSTOM_HR_START_READ

// specific write holding registers to unit
#define FUTURES CUSTOM_HR_START_WRITE

#define B1R1_1A_ZY_001_MB 80

#define HEART_BEAT HR_TEMPERATURE1 + 1 
#define B1R1_1A_LSH_001_MB HEART_BEAT + 1  
#define B1R1_1A_ZSC_001_MB B1R1_1A_LSH_001_MB + 1
#define B1R1_1A_HS_001_MB  B1R1_1A_ZSC_001_MB + 1


// B1R1_1A_LSH_001 
// write analogs/sp specific to units
//  HOLDING_REGISTER_WRITE_OFFSET + LAST #DEFINE IN THE LIST ON TOP.
//  IF YOU ADD MORE ENSURE THE CHANGE IS MADE HERE 



#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + FUTURE + 1
uint16_t modbusRegisters[MODBUS_REG_COUNT];

#define MB_SPEED 			    19200
#define MB_SLAVE_ID				11
#define MB_SERIAL_PORT			0
#define MB_MAX485_PIN			6  // set to zero for RS-232



Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT,MB_MAX485_PIN); 


