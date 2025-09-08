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
Arduino 2   → MCP4725 SDA
Arduino 3   → MCP4725 SCL  
Arduino 5V  → MCP4725 VDD
Arduino GND → MCP4725 GND
Author: Diego Klish 
*/

//first define our custom pins for SDA and SCL 
#define SDA_PIN 2
#define SCL_PIN 3
//for custom driver yes could make it faster with smaller delays but wiring would need to be perfect 
#define MY_DELAY 10 //standard timing delay 
#define I2C_MAX_WAIT 1000 //just a diffirent increment for fun 

const uint8_t MCP4725_ADDRESS = 0x60; //define constant value for MCP4725 address 
const uint16_t DAC_MAX_VALUE = 4095; //max DAC value 
const float REFERENCE_VOLTAGE = 5.0; //reference voltage 

const float VOLTAGES[] = {0.0, 1.25, 2.5, 3.75, 5.0}; //our test voltage values(same as before)
const int NUM_TEST_POINTS = 5; //number of cases 


void setup(){
    Serial.begin(9600);
    
    custom_init(); //call our custom function to initialize our two pins 
    Serial.println("Pins initalized");

    if(custom_test(MCP4725_ADDRESS)){ //uses custom test to see if connction is working 
    Serial.print("MCP4725 working with custom driver");
    }
    else{
        Serial.print("MCP4725 is not working with custom driver");
        while(1); //stops program while custom test is not working to ensure good connection 
    }
} 

void loop(){
    for (int i = 0; i < NUM_TEST_POINTS; i++) {//essentially the same test as our voltage test but this time using our custom drivers 
    float targetVoltage = VOLTAGES[i]; //starts with voltage array value 0 and goes up each iteration 
    uint16_t dacValue = voltageToDACValue(targetVoltage);//stores the result of our target voltage converted into a dac value
    
    // Use OUR custom I2C driver instead of Wire library
    bool success = custom_set_voltage(dacValue);
    
    Serial.print("Target: ");
    Serial.print(targetVoltage, 2);//2 decimal places since its a float 
    Serial.print("V | DAC: ");
    Serial.print(dacValue); //print dac value 
    
    if (success) {
      Serial.print("Custom I2C Driver Success");
    } else {
      Serial.print("Custom I2C Driver Failed");
    }
    
    Serial.println();
    delay(5000);  // Hold voltage for measurement
  }
  
  delay(2000);//avoid buffer 
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

    //need to set SDA from L to H while SCL is H for our end condition 
    //SCL high first 
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(MY_DELAY);
    //SDA high next 
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(MY_DELAY);
}

void custom_write_bit(uint8_t bit){//this function will bring SDA to high while SCL is low 
    digitalWrite(SDA_PIN, bit ? HIGH : LOW);//if bit is 1 SDA set to high otherwise low 
    delayMicroseconds(MY_DELAY);

    digitalWrite(SCL_PIN, HIGH); //SCL then reads the input 
    delayMicroseconds(MY_DELAY);

    digitalWrite(SCL_PIN, LOW); //set to low then ready for next input 
    delayMicroseconds(MY_DELAY);
}

bool custom_write_byte(uint8_t data){
    //send eight bits(byte) using our new write bit 
    for(int i = 7; i >= 0; i--){
        uint8_t bit = (data >> i) & 1;//shift by whatever our i value is and masking everything but the lowest bit we just shifted to 
        custom_write_bit(bit); //utilize our new function 
    }
    //we then need to read this bit from the slave 
    return custom_read_ack();
}

bool custom_read_ack(){
    //release SDA line 
    pinMode(SDA_PIN, INPUT); 
    digitalWrite(SDA_PIN, HIGH); //internal pullup now 
    delayMicroseconds(MY_DELAY);

    digitalWrite(SCL_PIN, HIGH);//CLK is now high and slave will pull SDA LOW 
    delayMicroseconds(MY_DELAY);

    bool ack_received = (digitalRead(SDA_PIN) == LOW);//LOW = is ACK otherwise if HIGH = NACK 
    
    digitalWrite(SCL_PIN, LOW);

    //now we need to take back control over the SDA line 
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(MY_DELAY);

    return ack_received;//give us our value for if the byte was read or not 
}

bool custom_test(uint8_t device_address){
    custom_start(); //start condition for our I2C comms 

    //want to test our connection to the board so we send address with one write bit 
    uint8_t addr_byte = (device_address << 1) | 0; 
    bool ack = custom_write_byte(addr_byte);  //stores in ack the result of ack_recieved if our value was read or not 1 or 0

    custom_stop();
    return ack; //return the 1 or 0 (true or false)
}

bool custom_set_voltage(uint16_t dac_value) { //function goes throught the full protocol and only returns true if every byte is ACK

  dac_value &= 0x0FFF;// Ensure 12-bit value by masking 
  custom_start();//start condition for our IC2 comms 
  
  if (!custom_write_byte((MCP4725_ADDRESS << 1) | 0)) {   // Send device address + write bit
    custom_stop();
    return false;  // Device didn't ACK address
  }
  
  if (!custom_write_byte((dac_value >> 8) & 0x0F)) {   // Send upper 4 bits of DAC value
    custom_stop();
    return false;  // Device didn't ACK data byte 1
  }
  
  // Send lower 8 bits of DAC value
  if (!custom_write_byte(dac_value & 0xFF)) {
    custom_stop();
    return false;  // Device didn't ACK data byte 2
  }
  
  custom_stop(); 
  
  return true;  //function worked 
}

uint16_t voltageToDACValue(float voltage) { //converts voltage to DAC value (same as in our voltage script)
  if (voltage < 0) voltage = 0;  //voltage cant be zero so cutsoff at 0
  if (voltage > REFERENCE_VOLTAGE) voltage = REFERENCE_VOLTAGE; //cutsoff at 5V in our case since thats our refrence voltage 
  return(uint16_t)((voltage / REFERENCE_VOLTAGE) * DAC_MAX_VALUE); //converts voltage to a ratio and mutlpilies it time our DAC value to scale it 
}
