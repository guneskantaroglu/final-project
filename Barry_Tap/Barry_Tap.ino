//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
//Default spi clock speed is one-fourth of the crystal being used. In this case it is 8/4 = 2Mhz.
//Datasheet recommends that output data rate can be kept anything for such a high speed, however in this code it has been kept 100 hz. This output data rate is also the default value set in BW_RATE register.
//Assign the Chip Select signal to pin 10.
int CS=10;

//ADXL345 Register Addresses // All the available registers are defined here.
#define	DEVID		0x00	//Device ID Register
#define THRESH_TAP	0x1D	//Tap Threshold
#define	OFSX		0x1E	//X-axis offset
#define	OFSY		0x1F	//Y-axis offset
#define	OFSZ		0x20	//Z-axis offset
#define	DURATION	0x21	//Tap Duration
#define	LATENT		0x22	//Tap latency
#define	WINDOW		0x23	//Tap window
#define	THRESH_ACT	0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT	0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	//free-fall threshold
#define	TIME_FF		0x29	//Free-Fall Time
#define	TAP_AXES	0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE		0x2C	//Data rate and power mode control // we do not set this in this code, therefore it has the default value acc to which output data rate is 100hz and bandwidth is 50 hz
#define POWER_CTL	0x2D	//Power Control Register
#define	INT_ENABLE	0x2E	//Interrupt Enable Control
#define	INT_MAP		0x2F	//Interrupt Mapping Control
#define	INT_SOURCE	0x30	//Source of interrupts
#define	DATA_FORMAT	0x31	//Data format control
#define DATAX0		0x32	//X-Axis Data 0
#define DATAX1		0x33	//X-Axis Data 1
#define DATAY0		0x34	//Y-Axis Data 0
#define DATAY1		0x35	//Y-Axis Data 1
#define DATAZ0		0x36	//Z-Axis Data 0
#define DATAZ1		0x37	//Z-Axis Data 1
#define	FIFO_CTL	0x38	//FIFO control
#define	FIFO_STATUS	0x39	//FIFO status

//This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];

//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;
char tapType=0;  //type of tap weather it is single tap or double tap.

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH); // with slave select line high, the slave ignores the master
  
  //Create an interrupt that will trigger when a tap is detected.
  attachInterrupt(0, tap, RISING);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);

  //Send the Tap and Double Tap Interrupts to INT1 pin
  writeRegister(INT_MAP, 0x9F); //INT_MAP decides which functions are sent to which interrupt
  //Look for taps on the Z axis only.
  writeRegister(TAP_AXES, 0x01); // select axis for tap detection
  //Set the Tap Threshold to 3.5g
  writeRegister(THRESH_TAP, 64);
  //Set the Tap Duration within which tap should occur
  writeRegister(DURATION, 40); // this is 10ms
  
  //100ms Latency before the second tap can occur.
  writeRegister(LATENT, 20); // this is 100ms
  writeRegister(WINDOW, 255); // this is 300ms
  
  //Enable the Single and Double Taps.
  writeRegister(INT_ENABLE, 0xE0);  //only single and double tap functions are enabled, so if an interrupt occurs, it must be either single tap or double tap.
                                    //This register is set at the end becasue it is mentioned in the datasheet that the interrupts must be configured before enabling them.
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode, only the measurement bit is set here
  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register by reading the register
  Serial.println(values[0]);
  delay(3000);
  
  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register by reading the register
  Serial.println(values[0]);
  delay(3000);
}

void loop(){
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];
  
  //Convert the accelerometer value to G's. 
  //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  xg = x * 0.0078;
  yg = y * 0.0078;
  zg = z * 0.0078;
  
  if(tapType > 0)
  {
    if(tapType == 1){
      Serial.println("SINGLE");
      Serial.print(x);
      Serial.print(',');
      Serial.print(y);
      Serial.print(',');
      Serial.println(z);
    }
    else{
      Serial.println("DOUBLE");
      Serial.print((float)xg,2);
      Serial.print("g,");
      Serial.print((float)yg,2);
      Serial.print("g,");
      Serial.print((float)zg,2);
      Serial.println("g");
    }
    detachInterrupt(0);
    delay(500);
    attachInterrupt(0, tap, RISING);
    tapType=0;    
  }
  delay(10); 
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, unsigned char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

void tap(void){
  
  //Clear the interrupts on the ADXL345
  readRegister(INT_SOURCE, 1, values); //INT_SOURCE register is read and is stored in values[0]. Reading the INT_SOURCE register also clears the interrupt functions.
  if(values[0] & (1<<5)) // This if statement will be either true or false. If it is true then bit 5 of int_source register had 1, in that case it is a double tap. 
  tapType=2;
  else // else it is a single tap coz tap function being called means an interrupt had occured, and if it wasnt double tap then it must be single tap.
  tapType=1;;
}
