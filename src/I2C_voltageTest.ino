/* I2C Voltage control test 
-Want to verify that the DAC is outputting the correct voltages, this is to establish a voltage baseline
-Function essentially takes in voltage value converts it to a digital value(DAC) and then converts it to analog
-This will test the MCP4725 DAC by outputting 0V, 1.25V, 2.5V, 3.75V, and 5V
Hardware setup:
Arduino A4 (SDA) → MCP4725 SDA
Arduino A5 (SCL) → MCP4725 SCL  
Arduino 5V       → MCP4725 VDD
Arduino GND      → MCP4725 GND
Author: Diego Klish
*/ 

#include <Wire.h> //library with I2C communication functions    

//Define all our constants 
const uint8_t MCP4725_ADDRESS = 0x60;  // /defining our constant address for our board
const uint16_t DAC_MAX_VALUE = 4095;   // 12-bit maximum 
const float REFERENCE_VOLTAGE = 5.0;   // refrence voltage 
const float VOLTAGES[] = {0.0, 1.25, 2.5, 3.75, 5.0}; //our test voltage values 
const int NUM_TEST_POINTS = 5;

void setup(){
    Serial.begin(9600); //starts serial comms at 9600 bits per sec
    Wire.begin(); //Initalizes the I2C System
    delay(2000); //avoid buffer overlap 

    Serial.println("Testing voltage output at different DAC values");
    Serial.println("Measure VOUT pin with multimeter to verify");

    if (I2C_test()) {//verifies the MCP4725 is connected 
    Serial.println("MCP4725 is connected!");
  } else {
    Serial.println("MCP4725 is not connected, error#");
    while(1); //halt program  
  }
}

void loop(){
    for (int i = 0; i < NUM_TEST_POINTS; i++) { //start at test case i(0) and increase by 1 each loop iteration 
    float targetVoltage = VOLTAGES[i]; //assigns a target voltage with i position in our array 
    uint16_t dacValue = vToDAC(targetVoltage); //use our voltage to DAC function to get a dac value for this target voltage 
    float calculatedVoltage = dacToV(dacValue); //get our voltage value back again after running our dac value back to voltage 
    //this will effectively test both voltage to digital and digital to voltage 
    
    setDACOutput(dacValue); //sends DAC value to MCP4725
  /*DEBUG: Print what we actually sent
  Serial.print("DEBUG - Sent DAC value: ");
  Serial.print(dacValue);
  Serial.print(" (Binary: ");
  Serial.print(dacValue, BIN);
  Serial.println(")");
  */
Serial.print("Target Voltage: ");
    Serial.print("Target Voltage: ");
    Serial.print(targetVoltage, 2); //target voltage 2 decimal places 
    Serial.print("V | DAC: ");  
    Serial.print(dacValue); //digital value sent 
    Serial.print(" | Calc: "); 
    Serial.print(calculatedVoltage, 3); //expected voltage  3 decimals 
    Serial.println("V");
    
    delay(5000); //holds voltage for 5 seconds then next iteration in the loop 
  }
  
  Serial.println("Test cycle complete.\n"); //letting user know cycle complete 
  delay(2000); //holds another 2 seconds 
}

uint16_t vToDAC(float voltage) { //converts our voltage value to DAC
  if (voltage < 0) voltage = 0; //clamp to zero since DAC cant output negative 
  if (voltage > REFERENCE_VOLTAGE) voltage = REFERENCE_VOLTAGE; //clamps to 5V since our system is limited to 5
  return (uint16_t)((voltage / REFERENCE_VOLTAGE) * DAC_MAX_VALUE); //converts to percentage scaled to DAC value and converted to integer
}

float dacToV(uint16_t dacValue) {
  return ((float)dacValue / DAC_MAX_VALUE) * REFERENCE_VOLTAGE; //converts integer to DAC value and gives voltage 
}

void setDACOutput(uint16_t dacValue) {
  dacValue &= 0x0FFF; //Masks unwanted bits 
  
  Wire.beginTransmission(MCP4725_ADDRESS);//begins communication with MCP4725 
  Wire.write((dacValue >> 8) & 0x0F); //sends upper byte(upper 4 bits)
  Wire.write(dacValue & 0xFF); //sends lower byte(lower 8 bits)
  
  uint8_t result = Wire.endTransmission();//end communication and store result
  
  if (result != 0) { //result is not zero it means we got an error 
    Serial.print("I2C Error: ");
    Serial.println(result);
  }
}

bool I2C_test() {
  Wire.beginTransmission(MCP4725_ADDRESS);
   byte error = Wire.endTransmission(); //stores number of error values from transmission
   return(error == 0); //if error is not equal to zero returns false otherwise true
}