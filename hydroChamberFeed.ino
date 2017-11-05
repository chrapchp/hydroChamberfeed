/**
*  @file    ChamberFeed.ino
*  @author  peter c
*  @date    10/30/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Chamber feed and return. Same programm as EOL with the exception of an a chamber return
*  4 valves to control via I2C chip
** @section HISTORY
** 2017Oct30 - created
*/
#include <HardwareSerial.h>
#include <TimeAlarms.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include <Streaming.h>
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include "unitModbus.h"
// comment out to  include terminal processing for debugging
// #define PROCESS_TERMINAL
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
// #define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate
#define ALARM_REFRESH_INTERVAL 100 // ms
// MCP2008 Pins for chamber return
#define B1R1_1A_XY_009_PIN 5 // modbus adddress for valve open/close
#define B1R1_1A_XY_010_PIN 1
#define B1R1_1A_XY_011_PIN 3
#define B1R1_1A_XY_012_PIN 4
#define B1R1_1A_XY_009_OFFSET 0 // array offset. 1 for each MCP I/O in this case 4 valves
#define B1R1_1A_XY_010_OFFSET 1
#define B1R1_1A_XY_011_OFFSET 2
#define B1R1_1A_XY_012_OFFSET 3
#define CHAMBER_RETURN_MCP_ADDRESS 0
// One Wire - Hydroponic temperatures
// 
#define TEMPERATURE1 8 // pin
#define TEMPERATURE2 7 // pin
#define TEMPERATURE3 14 // pin
#define TEMPERATURE4 4 // pin
#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWireBus1(TEMPERATURE1);
OneWire oneWireBus2(TEMPERATURE2);
OneWire oneWireBus3(TEMPERATURE3);
OneWire oneWireBus4(TEMPERATURE4);
DallasTemperature B1N1_T1_TT004(& oneWireBus1);
DallasTemperature B1N1_N1_TT004(& oneWireBus2);
DallasTemperature B1N1_N2_TT004(& oneWireBus3);
DallasTemperature B1N1_N3_TT004(& oneWireBus4);
DA_DiscreteOutput B1N1_T1_XY004 = DA_DiscreteOutput(3, LOW); // V1
DA_DiscreteOutput B1N1_N1_XY004 = DA_DiscreteOutput(11, LOW); // V2
DA_DiscreteOutput B1N1_N2_XY004 = DA_DiscreteOutput(10, LOW); // V3
DA_DiscreteOutput B1N1_N3_XY004 = DA_DiscreteOutput(9, LOW); // V4
DA_AnalogInput B1N1_T1_PT004 = DA_AnalogInput(A1, 0.0, 1023.); // min max
DA_AnalogInput B1N1_N1_PT004 = DA_AnalogInput(A2, 0.0, 1023.); // min max
DA_AnalogInput B1N1_N2_PT004 = DA_AnalogInput(A6, 0.0, 1023.); // min max
DA_AnalogInput B1N1_N3_PT004 = DA_AnalogInput(A7, 0.0, 1023.); // min max
// DA_DiscreteOutput MAX285TX_ENABLE = DA_DiscreteOutput( 6, HIGH );  // 1 is enable, 0 rx enabled

#ifdef PROCESS_TERMINAL
DA_DiscreteOutput LED = DA_DiscreteOutput(13, HIGH); // for debugging
#endif

Adafruit_MCP23008 chamberReturn;
// HEARTBEAT
unsigned int heartBeat = 0;
struct _MCPEntry
{
  unsigned short pin;
  bool isFirstRun;
  bool state; // high/low
};

typedef _MCPEntry MCPEntry;
MCPEntry returnValves[] =
{
  {
    B1R1_1A_XY_009_PIN, true, false
  },
  {
    B1R1_1A_XY_010_PIN, true, false
  },
  {
    B1R1_1A_XY_011_PIN, true, false
  },
  {
    B1R1_1A_XY_012_PIN, true, false
  }
};

struct _AlarmEntry
{
  time_t epoch;
  AlarmId id = dtINVALID_ALARM_ID;
  bool firstTime = true;
};

typedef _AlarmEntry AlarmEntry;
AlarmEntry onRefreshAnalogs; // sonar and 1-wire read refresh
AlarmEntry onFlowCalc; // flow calculations

#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = & Serial2;
#endif

void onEdgeDetect(bool state, int pin)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "Edge Detection:" << state << " on pin " << pin << endl;
#endif

}

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
  init1WireTemperatureSensor(& B1N1_T1_TT004, 1);
  init1WireTemperatureSensor(& B1N1_N1_TT004, 2);
  init1WireTemperatureSensor(& B1N1_N2_TT004, 3);
  init1WireTemperatureSensor(& B1N1_N3_TT004, 4);
}

void setupMCP()
{
  chamberReturn.begin(CHAMBER_RETURN_MCP_ADDRESS);
  chamberReturn.pinMode(B1R1_1A_XY_009_PIN, OUTPUT);
  chamberReturn.pinMode(B1R1_1A_XY_010_PIN, OUTPUT);
  chamberReturn.pinMode(B1R1_1A_XY_011_PIN, OUTPUT);
  chamberReturn.pinMode(B1R1_1A_XY_012_PIN, OUTPUT);
}

void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

  setupMCP(); // i2C connection to chamber return
  randomSeed(analogRead(3));
  onRefreshAnalogs.id = Alarm.timerRepeat(POLL_CYCLE_SECONDS, doOnPoll);
  initHydroponicOneWireTemps();
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif

  Alarm.delay(ALARM_REFRESH_INTERVAL);
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  doReadAnalogs();
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
  doPoll1WireTemperature(& B1N1_T1_TT004, 0);
  doPoll1WireTemperature(& B1N1_N1_TT004, 1);
  doPoll1WireTemperature(& B1N1_N2_TT004, 2);
  doPoll1WireTemperature(& B1N1_N3_TT004, 3);
}

void doReadAnalogs()
{
  B1N1_T1_PT004.refresh();
  B1N1_N1_PT004.refresh();
  B1N1_N2_PT004.refresh();
  B1N1_N3_PT004.refresh();

#ifdef TRACE_3NALOGS
  B1N1_T1_PT004.serialize(tracePort, true);
  B1N1_N1_PT004.serialize(tracePort, true);
  B1N1_N2_PT004.serialize(tracePort, true);
  B1N1_N3_PT004.serialize(tracePort, true);
#endif

}

// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{
  modbusRegisters[HR_TEMPERATURE1] = B1N1_T1_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE2] = B1N1_N1_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE3] = B1N1_N2_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_TEMPERATURE4] = B1N1_N3_TT004.getTempCByIndex(0) * 100;
  modbusRegisters[HR_PRESSURE1] = B1N1_T1_PT004.getRawSample();
  modbusRegisters[HR_PRESSURE2] = B1N1_N1_PT004.getRawSample();
  modbusRegisters[HR_PRESSURE3] = B1N1_N2_PT004.getRawSample();
  modbusRegisters[HR_PRESSURE4] = B1N1_N3_PT004.getRawSample();
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

void writeMCP_DO(bool aState, unsigned short anOffset)
{
  if (returnValves[anOffset].isFirstRun || returnValves[anOffset].state != aState)
  {
    chamberReturn.digitalWrite(returnValves[anOffset].pin, aState);
    returnValves[anOffset].isFirstRun = false;
    returnValves[anOffset].state = aState;

  #ifdef TRACE_MODBUS
    *tracePort << "write  MCP DO:" << returnValves[anOffset].pin << " state:" << aState << endl;
  #endif

  }
}

void updateMCP_DO(unsigned int bitOffset, unsigned short anOffset)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    writeMCP_DO(HIGH, anOffset);
  }
  else
  {
    writeMCP_DO(LOW, anOffset);
  }
}

void processValveCommands()
{
  // returnValves
  updateMCP_DO(B1R1_1A_XY_009, B1R1_1A_XY_009_OFFSET);
  updateMCP_DO(B1R1_1A_XY_010, B1R1_1A_XY_010_OFFSET);
  updateMCP_DO(B1R1_1A_XY_011, B1R1_1A_XY_011_OFFSET);
  updateMCP_DO(B1R1_1A_XY_012, B1R1_1A_XY_012_OFFSET);

  checkAndActivateDO(VALVE1_OPEN_CLOSE, & B1N1_T1_XY004);
  checkAndResetDO(VALVE1_OPEN_CLOSE, & B1N1_T1_XY004);

  checkAndActivateDO(VALVE2_OPEN_CLOSE, & B1N1_N1_XY004);
  checkAndResetDO(VALVE2_OPEN_CLOSE, & B1N1_N1_XY004);

  checkAndActivateDO(VALVE3_OPEN_CLOSE, & B1N1_N2_XY004);
  checkAndResetDO(VALVE3_OPEN_CLOSE, & B1N1_N2_XY004);

  checkAndActivateDO(VALVE4_OPEN_CLOSE, & B1N1_N3_XY004);
  checkAndResetDO(VALVE4_OPEN_CLOSE, & B1N1_N3_XY004);
  
}

void processModbusCommands()
{
  processValveCommands();
}

#endif
