/* Custom I2C Driver 
-I2C is a protocol which utilizes two lines SDA(Serial Data) and SCL(Serial Clock)
-Data line carries information with the data being changed when SCL is LOW and being read 
when SCL is HIGH
-Now that weve tested the hardware and we know that the DAC is functioning correctly we can 
start writing the custom I2C Driver 
-We will be writing the wire library functions using our knowledge of the I2C protocol and manually manipulating 
the SDA and SCL lines so no need to import the wire library 
-Since we dont need specific analog pins we will define our SDA pin as 1 and SCL pin as 0 
Hardware setup:
Arduino 1   → MCP4725 SDA
Arduino 0   → MCP4725 SCL  
Arduino 5V  → MCP4725 VDD
Arduino GND → MCP4725 GND
Author: Diego Klish 
*/

//first define our custom pins for SDA and SCL 
#define SDA_PIN 1;
#define SCL_PIN 0;
//for custom driver yes could make it faster with smaller delays but wiring would need to be perfect 
#define MY_DELAY 10; //standard timing delay 
#define I2C_MAX_WAIT 1000; //just a diffirent increment for fun 

const uint8_t MCP4725_ADDRESS = 0x60; //define constant value for MCP4725 address 
const uint16_t DAC_MAX_VALUE = 4095; //max DAC value 
const float REFERENCE_VOLTAGE = 5.0; //reference voltage 

const float VOLTAGES[] = {0.0, 1.25, 2.5, 3.75, 5.0}; //our test voltage values(same as before)
const int NUM_TEST_POINTS = 5; //number of cases 


void setup(){
    Serial.begin(9600);
    
    custom_init() //call our custom function to initialize our two pins 
    Serial.println("Pins initalized");



} 

void loop(){

}
/*starting by building the wire library functions from the ground up using my 
knwoledge of the I2C protocol 
*/
void custom_init(){ //intiliaze both pins 
    pinMode(SDA_PIN, OUTPUT); //set both pins as outputs 
    pinMode(SCL_PIN, OUTPUT);

    digitalWrite(SDA_PIN, HIGH);//set both pins to idle state both set to HIGH
    digitalWrite(SCL_PIN, HIGH);

    delayMicroseconds(MY_DELAY);//set delay to ensure clean signal transition 
}

void custom_start(){ //start condition for I2C protocol
    //first well ensure both of our pins are set to high for idle state
    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(MY_DELAY);

    //need to set SDA from H to L while SCL is H for our start condition 
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(MY_DELAY);

    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(MY_DELAY);
}

void custom_stop(){ //stop condition for I2C protocol 
    //first well ensure both of our pins are set to low 
    digitalWrite(SDA_PIN, LOW);
    digitalWrite(SCL_PIN, LOW);

    //need to set SDA from L to H while SCL is H for our start condition 
    //SCL high first 
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(MY_DELAY);
    //SDA high next 
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(MY_DELAY);
}

