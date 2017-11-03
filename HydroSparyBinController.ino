/**
*  @file    NutrientEOL.ino
*  @author  peter c
*  @date    10/25/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Spray Bin Control - four identical ones. This one is based on Spray Bin Front Left I/O
** @section HISTORY
** 2017Nov2 - created
*/
#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // for RTC
#include <Streaming.h>
#include <DA_AnalogInput.h>
#include <DA_DiscreteOutput.h>

#include <DA_NonBlockingDelay.h>


#include "unitModbus.h"
// comment out to  include terminal processing for debugging
 //#define PROCESS_TERMINAL
 //#define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
// #define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate

// One Wire - Hydroponic temperatures
// 
#define TEMPERATURE1 8 // pin

#define ONE_TEMPERATURE_PRECISION 5
OneWire oneWireBus1(TEMPERATURE1);

DallasTemperature B1R1_1A_TT_001(& oneWireBus1);


DA_DiscreteOutput B1R1_1A_ZY_001 = DA_DiscreteOutput(A3, LOW); // Latch

DA_AnalogInput B1R1_1A_LSH_001 = DA_AnalogInput(  A5, 0., 1.0 ) ;
DA_AnalogInput B1R1_1A_ZSC_001 = DA_AnalogInput(  A4, 0., 1.0) ;
DA_AnalogInput B1R1_1A_HS_001 = DA_AnalogInput(  A2, 0., 1.0) ;
 




// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay( 2000, &doOnPoll);


// HEARTBEAT
unsigned int heartBeat = 0;



#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = & Serial;
#endif



void printOneWireAddress(HardwareSerial *tracePort, DeviceAddress aDeviceAddress, bool aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16)
      *tracePort << '0';
    tracePort->print(aDeviceAddress[i], HEX);
  }
  if (aCR)
    *tracePort << endl;
}

void init1WireTemperatureSensor(DallasTemperature * sensor, int idx)
{
  DeviceAddress address;
  sensor->begin();
  if (sensor->getAddress(address, 0))
  {

#ifdef TRACE_1WIRE
    *tracePort << "Channel " << idx << " 1Wire Temperature initialized. Address =  ";
    printOneWireAddress(tracePort, address, true);
#endif

    sensor->setResolution(address, ONE_TEMPERATURE_PRECISION);
  }
  else
  {

#ifdef TRACE_1WIRE
    *tracePort << "Unable to find address for 1Wire Temperature Device @ " << idx << endl;
#endif

  }
}

void initHydroponicOneWireTemps()
{
  init1WireTemperatureSensor(& B1R1_1A_TT_001, 1);

}

int polling; // 1=polling analogs, 2=polling digitals, -1 nothing
int currentTogglePin; // -1 none, >0 pin
void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif



  randomSeed(analogRead(3));

  initHydroponicOneWireTemps();
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif
pollTimer.refresh();
refreshAnalogs();
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  
  doProcess1WireTemperatures();
  heartBeat++;
}

void doPoll1WireTemperature(DallasTemperature * sensor, int idx)
{
  sensor->requestTemperatures();

#ifdef TRACE_1WIRE
  *tracePort << "Temperature " << idx << " = " << sensor->getTempCByIndex(0) << " C" << endl;
#endif

}

void doProcess1WireTemperatures()
{
  doPoll1WireTemperature(& B1R1_1A_TT_001, 1);
}

void refreshAnalogs()
{

B1R1_1A_LSH_001.refresh();
B1R1_1A_ZSC_001.refresh();
B1R1_1A_HS_001.refresh();

}



// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{


  modbusRegisters[B1R1_1A_LSH_001_MB] =  (bool) B1R1_1A_LSH_001.getScaledSample(); 
  modbusRegisters[B1R1_1A_ZSC_001_MB] = (bool) B1R1_1A_ZSC_001.getScaledSample(); 
  modbusRegisters[B1R1_1A_HS_001_MB] = (bool) B1R1_1A_HS_001.getScaledSample(); 
  modbusRegisters[HR_TEMPERATURE1] =  B1R1_1A_TT_001.getTempCByIndex(0) * 100;

  modbusRegisters[HR_HEARTBEAT] = heartBeat;




}


bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  // *tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void checkAndActivateDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->activate();

  #ifdef TRACE_MODBUS
    *tracePort << "Activate DO:";
    aDO->serialize(tracePort, true);
    LED.activate();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void checkAndResetDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (!getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->reset();

  #ifdef TRACE_MODBUS
    *tracePort << "Reset DO:";
    aDO->serialize(tracePort, true);
    LED.reset();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void processValveCommands()
{
  checkAndActivateDO(B1R1_1A_ZY_001_MB, & B1R1_1A_ZY_001);
  checkAndResetDO(B1R1_1A_ZY_001_MB, & B1R1_1A_ZY_001);

}

void processModbusCommands()
{
  processValveCommands();
}

#endif
