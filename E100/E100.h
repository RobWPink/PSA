#include <Wire.h>
#include <SPI.h>
#include <ADS7828.h>
#include <PI4IOE5V6534Q.h>
#include "Adafruit_MCP9600.h"
#include <Ewma.h>
#include <ModbusMaster_oneH2.h>

ModbusMaster mbRTU;

unsigned long timer[5] = {0};
unsigned long debouncer[4] = {0};
