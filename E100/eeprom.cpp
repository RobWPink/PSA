#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include "config_e100.h"

// Addresses up from 0-999 saved for future use of more generic data 
#define   PAGE_SIZE                                       256
#define   EEPROM_ADDR_PRODUCTION_EVO_DELTA_STEP           1000
// Define the chip select pin
#define   CS_PIN                                          10

// Create an Adafruit_SPIFlash object with SPI1
Adafruit_FlashTransport_SPI flashTransport(CS_PIN, SPI1);
Adafruit_SPIFlash flash(&flashTransport);

bool eeprom_online = false; 


/*
    Name:         printUniqueID
    Arguments:    void  
    Returns:      nil
    Description:  A simple debug function to confirm the operation of the EEPROM class by printing the unique manufacturer ID 
                  held within the chip. 
*/
void printUniqueID() {
  uint8_t uniqueID[8];

  digitalWrite(CS_PIN, LOW);
  SPI1.transfer(0x4B); // Unique ID read command
  for (int i = 0; i < 4; i++) {
    SPI1.transfer(0x00); // Send dummy bytes
  }
  for (int i = 0; i < 8; i++) {
    uniqueID[i] = SPI1.transfer(0x00); // Read Unique ID
  }
  digitalWrite(CS_PIN, HIGH);

  for (int i = 0; i < 8; i++) {
    if (uniqueID[i] < 0x10) Serial.print("0"); // Leading zero for single hex digits
    Serial.print(uniqueID[i], HEX);
    if (i < 7) {
      Serial.print(":");
    }
  }
  Serial.println();
}

/*
    Name:         eeprom_setup
    Arguments:    void  
    Returns:      nil
    Description:  This function is called only once at startup to initialize the operating EEPROM in the E100
*/
void eeprom_setup(void)
{
  // Initialize SPI1
  SPI1.begin();

  // Initialize the SPIFlash library
  if (flash.begin()) {
    const uint32_t address = 0x000000;
    uint8_t dataRead[PAGE_SIZE];
    FLOATUNION_t read_config_var; 

    Serial.println("SPIFlash library initialized.");

    // Print JEDEC ID
    uint32_t jedecID = flash.getJEDECID();
    Serial.print("JEDEC ID: 0x");
    Serial.println(jedecID, HEX);

    // Print Unique ID using raw SPI commands
    Serial.print("Unique ID: ");
    printUniqueID();

    // Read data from EEPROM    
    if (flash.readBuffer(address, dataRead, sizeof(dataRead))) {
      Serial.print("Read Data: ");
      for (int i = 0; i < sizeof(dataRead); i++) {
        Serial.print(dataRead[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      memcpy(read_config_var.bytes, dataRead, 4); 
      set_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP, read_config_var.number);
      memcpy(read_config_var.bytes, &dataRead[4], 4); 
      set_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME, read_config_var.number);
    } else {
      Serial.println("Failed to read data.");
    }
    // Unblock future writes to the EEPROM
    eeprom_online = true;
  } else {
    Serial.println("Failed to initialize SPIFlash library.");
  }
}


void eeprom_save(void){
  // Example: Erase, write, and read data
  FLOATUNION_t save_config_var; 
  const uint32_t address = 0x000000;
  uint8_t dataToWrite[PAGE_SIZE];
  
  if (!eeprom_online){
    Serial.println("EEPROM Write blocked by eeprom_online.");
    return;
  }

  save_config_var.number = get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP);
  memcpy(dataToWrite, save_config_var.bytes, 4);
  save_config_var.number = get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME);
  memcpy(&dataToWrite[4], save_config_var.bytes, 4);

  // Erase the sector
  if (flash.eraseSector(address)) {
    Serial.println("Sector erased.");

    // Write data to EEPROM
    if (flash.writeBuffer(address, dataToWrite, sizeof(dataToWrite))) {
      Serial.println("Data written.");

      // Read data from EEPROM
      uint8_t dataRead[PAGE_SIZE];
      if (flash.readBuffer(address, dataRead, sizeof(dataRead))) {
        Serial.print("Read Data: ");
        for (int i = 0; i < sizeof(dataRead); i++) {
          Serial.print(dataRead[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      } else {
        Serial.println("Failed to read data.");
      }
    } else {
      Serial.println("Failed to write data.");
    }
  } else {
    Serial.println("Failed to erase sector.");
  }
}