#include "config.h"
#include "BMSModule.h"
//#include "BMSUtil.h"
#include "Logger.h"

BMSModule::BMSModule()
{
  for (int i = 0; i < 32; i++)
  {
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;

  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

void BMSModule::clearmodule()
{
  for (int i = 0; i < 32; i++)
  {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

float BMSModule::decodeCellVoltage(int cell, CAN_message_t &msg, int msb, int lsb)
{
  if ((((msg.buf[msb] & 0x0F) << 8) + msg.buf[lsb]) > 0)
  {
    cellVolt[cell] = float((((msg.buf[msb] & 0x0F) << 8) + msg.buf[lsb]) * 0.00125);
  }
}

void BMSModule::decodecan(int Module, CAN_message_t &msg)
{
    // module voltages are on message IDs x200, x202, x204, and x206
    if (msg.id == 0x200 || msg.id == 0x202 || msg.id == 0x204 || msg.id == 0x206)
    {
        // there CAN messages are structured so there are 4 banks (0-3), 8 Modules (0-7) per bank, and 3 cells (A-C) per bank
        // the bank ID is coded as the biggest 3 bits (7,6,5) of the 6th (index 0) byte of the message
        // This message structure doesn't really correspond to the physical layout of the cells
        int bank = (msg.id & 0x00F) >> 1;  //convert the message IDs to mod ID 0, 1, 2, 3
        // this should give us 12 (3 * 4) cell voltages per Module - again not exactly matching the physical layout
        // The 3 cell voltages for the bank are kind of hard to describe, but open the DBC file in Kvaser DB Editor to visualize
        cellVolt[bank * 3 + 0] = float(((msg.buf[0] & 0x1F) << 7) + (msg.buf[1] >> 1)) * 0.00125;    //A (first) voltage in the bank
        cellVolt[bank * 3 + 1] = float(((msg.buf[2] & 0x1F) << 7) + (msg.buf[3] >> 1)) * 0.00125;    //B (second) voltage in the bank
        cellVolt[bank * 3 + 2] = float(((msg.buf[4] & 0xFF) << 4) + (msg.buf[5] >> 4)) * 0.00125;    //C (third) voltage in the bank
    }
    else if (msg.id == 0x302) 
    {
        // Battery Temp
        // There are 6 sensors in the pack and they don't exactly map to modules, but that's what I'm doing
        // this is kind of hacky, and could fail if a short msg.buf got passed in here somehow
        if (Module <= 6) {
            temperatures[0] = (float(msg.buf[Module + 1]) * 0.5) - 40.0;
        }
    }
    
    else 
    {
        switch (msg.id)
        {
        case 0x60:
            decodeCellVoltage(1, msg, 0, 1);
            decodeCellVoltage(2, msg, 2, 3);
            decodeCellVoltage(3, msg, 4, 5);
            decodeCellVoltage(4, msg, 6, 7);
            break;

        case 0x70:
            decodeCellVoltage(5, msg, 0, 1);
            decodeCellVoltage(6, msg, 2, 3);
            decodeCellVoltage(7, msg, 4, 5);
            decodeCellVoltage(8, msg, 6, 7);
            break;

        case 0xE0:
            temperatures[0] = float(((msg.buf[6] << 8) + msg.buf[7]) * -0.0324 + 150);
            break;

        default:
            break;
        }
    }
  /*
    if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
    if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();
    for (int i = 0; i < 32; i++)
    {
    if (lowestCellVolt[i] > cellVolt[i] && cellVolt[i] >= IgnoreCell)
    {
      lowestCellVolt[i] = cellVolt[i];
    }
    if (highestCellVolt[i] < cellVolt[i])
    {
      highestCellVolt[i] = cellVolt[i];
    }
    }
  */
}

int BMSModule::getCellsUsed()
{
  return cellsused;
}

uint8_t BMSModule::getFaults()
{
  return faults;
}

uint8_t BMSModule::getAlerts()
{
  return alerts;
}

uint8_t BMSModule::getCOVCells()
{
  return COVFaults;
}

uint8_t BMSModule::getCUVCells()
{
  return CUVFaults;
}

float BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 32)
    return 0.0f;
  return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 32; i++)
    if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell)
      lowVal = cellVolt[i];
  return lowVal;
}

float BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 32; i++)
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      if (cellVolt[i] > hiVal)
        hiVal = cellVolt[i];
    }
  return hiVal;
}

float BMSModule::getAverageV()
{
  int x = 0;
  cellsused = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 32; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      x++;
      avgVal += cellVolt[i];
      cellsused = i;
    }
  }

  scells = x;
  avgVal /= x;
  return avgVal;
}

int BMSModule::getscells()
{
  return scells;
}

float BMSModule::getHighestModuleVolt()
{
  return highestModuleVolt;
}

float BMSModule::getLowestModuleVolt()
{
  return lowestModuleVolt;
}

float BMSModule::getHighestCellVolt(int cell)
{
  if (cell < 0 || cell > 32)
    return 0.0f;
  return highestCellVolt[cell];
}

float BMSModule::getLowestCellVolt(int cell)
{
  if (cell < 0 || cell > 32)
    return 0.0f;
  return lowestCellVolt[cell];
}

float BMSModule::getHighestTemp()
{
  return (temperatures[0]);
}

float BMSModule::getLowestTemp()
{
  return (temperatures[0]);
}

float BMSModule::getLowTemp()
{
  return (temperatures[0]);
}

float BMSModule::getHighTemp()
{
  return (temperatures[0]);
}

float BMSModule::getAvgTemp()
{
  return (temperatures[0]);
}

float BMSModule::getModuleVoltage()
{
  moduleVolt = 0;
  for (int I; I < 32; I++)
  {
    if (cellVolt[I] > IgnoreCell && cellVolt[I] < 60.0)
    {
      moduleVolt = moduleVolt + cellVolt[I];
    }
  }
  return moduleVolt;
}

float BMSModule::getTemperature(int temp)
{
    /*
    // I don't know what this was doing. getTemperature is only referenced in printing out battery stats
    // everything else uses temperatures[0]
    
  if (temp < 0 || temp > 2)
    return 0.0f;
  */
  return temperatures[temp];
}

void BMSModule::setTemperature(int tempIndex, float temp)
{
    temperatures[tempIndex] = temp;
}

void BMSModule::setAddress(int newAddr)
{
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR)
    return;
  moduleAddress = newAddr;
}

int BMSModule::getAddress()
{
  return moduleAddress;
}

bool BMSModule::isExisting()
{
  return exists;
}

bool BMSModule::isReset()
{
  return reset;
}

void BMSModule::settempsensor(int tempsensor)
{
  sensor = tempsensor;
}

void BMSModule::setExists(bool ex)
{
  exists = ex;
}

void BMSModule::setReset(bool ex)
{
  reset = ex;
}

void BMSModule::setIgnoreCell(float Ignore)
{
  IgnoreCell = Ignore;
  Serial.print(Ignore);
  Serial.println();
}
