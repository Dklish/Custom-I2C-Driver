/* I2C Hardware Test 
-Goal for this is to verify the MCP4725 digital to analog converter is working using
Arduino I2C library before attempting custom approach with manual pins 
Author: Diego Klish 
*/

#include <Wire.h> //library with I2C communication functions    

#define MCP4725_ADDRESS 0x60 //defining our constant address for our board 

void setup(){
    Serial.begin(9600); //starts serial comms at 9600 bits per sec
    Wire.begin(); //Initalizes the I2C System
    delay(2000); //avoide buffer overlap 
    
    Serial.println("Checking to see if MCP4725 is working");

    Wire.beginTransmission(MCP4725_ADDRESS);//Begins talking to the address 0x60

    byte error = Wire.endTransmission(); //stores number of error values from transmission

    if (error == 0){ //if # of errors is 0 we know the device is functioning correctly 
        Serial.println("MCP4725 is connected!");
    }
    else{ // # of errors is not zero and the device is not functioning correctly 
        Serial.println("MCP4725 is not connected, error#");
        Serial.println(error);
    }
    j
}

void loop(){
    //empty for this 
}

