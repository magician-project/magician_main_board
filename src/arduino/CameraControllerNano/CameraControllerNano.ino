#include <Arduino.h> //<- this should be first to make sure all includes are ok
#include <Wire.h>

#include "VL53L0X.h"
//  git clone https://github.com/pololu/vl53l0x-arduino && cp vl53l0x-arduino/VL53L0X.cpp ./ && cp vl53l0x-arduino/VL53L0X.h ./ 


#define USE_ARDUINO_NANO 1

#define USE_LASER 1
#define DEBUG_LASERS 0
#define ENABLE_LASER_LIGHT_DECISION 1

//Expecting a W5100 ethernet 
#define USE_ETHERNET 1
#define DEBUG_ETHERNET 0

#define NUMBER_OF_LIGHTS 6
#define NUMBER_OF_DISTANCE_SENSORS 3

#define VERSION_MAJOR 0
#define VERSION_MINOR 26

// I2C communication
#define SDA_PIN A4
#define SCL_PIN A5
#define I2C_EXTRA_PIN 9
#define ETHERNET_RESET_PIN 9

// 74HC595 control pins
uint8_t use74HC595ForLightControl = 0;  // By default start in direct pin mode
#define LATCH_PIN 3          // STCP
#define CLOCK_PIN 4          // SHCP
#define DATA_PIN  5          // DS
#define OUTPUT_ENABLE_PIN 2  // OUTPUT ENABLE / OE
#define CLEAR_PIN A3         // CLEAR / SRCLR

#define NO_LIGHT_ON 16

//Arduino Nano/UNO Port
//The laser library is so big that it may cause a stack overflow (running out of memory and over the heap)
//Causing the arduino to restart cycle. To fix this a special version of "Ammar_VL53L0X" strips a lot of fat
//*AND* there needs to be a change in the TX/RX buffer sizes by
// Altering : ~/.arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino/HardwareSerial.h
// changing SERIAL_TX_BUFFER_SIZE to 16 and SERIAL_RX_BUFFER_SIZE to 8
//With this trick :
//Sketch uses 20460 bytes (66%) of program storage space. Maximum is 30720 bytes.
//Global variables use 1647 bytes (80%) of dynamic memory, leaving 401 bytes for local variables. Maximum is 2048 bytes.
//--------------------------------------------------
#if USE_ARDUINO_NANO
const unsigned char directLightControl[NUMBER_OF_LIGHTS]={3,4,5,6,7,8}; //The last port is 9 because 7 doesn't work for some reason :S
#define annotationButton A7 
#define secondButton     A6
const unsigned char laserSwitch[NUMBER_OF_DISTANCE_SENSORS]={A0,A1,A2}; //Ports of XSHUT triggers
#endif
//--------------------------------------------------

//#define USE_LASER_1 1
//#define USE_LASER_2 1
//#define USE_LASER_3 1

//Global Variables (defines occupy no bytes on compilation)
//-------------------------------------------------------
//-------------------------------------------------------
//-------------------------------------------------------
#define CPUSleepTimeMilliseconds 1
//-------------------------------------------------------
bool serialOutputEnabled             = false;
unsigned int loopRateMsec            = 10;  // 1000ms = 1Hz / 20ms = 50Hz / -> 10ms = 100Hz 
unsigned int lightDurationMsec       = 100; // milliseconds that each light remains activated by default
unsigned int lightTurnOffMicrosecond = 0;   // If not zero ensures that each light remains powered for no more than this time
//-------------------------------------------------------
unsigned char autoLights = 0; //By default use time out 
unsigned char lightOn    = NO_LIGHT_ON; //Start at first light
//-------------------------------------------------------
unsigned long lastUpdateTime = 0;
unsigned long lightStartTime = 0;
//-------------------------------------------------------


// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x21
#define LOX3_ADDRESS 0x32
const unsigned char LOX_ADDRESS[NUMBER_OF_DISTANCE_SENSORS]={LOX1_ADDRESS,LOX2_ADDRESS,LOX3_ADDRESS}; //Ports of XSHUT triggers
#define D_T 10

// objects for the vl53l0x
unsigned char  laser_working[3]={0};
unsigned char  laser_status[3] ={0};
unsigned short laser_distance_millimeters[3]={0};
//see vl53l0x_def.h for definition of VL53L0X_RangingMeasurementData_t
//VL53L0X_RangingMeasurementData_t measure[3]={0};

//VL53L0X_Dev_t is declared in vl53l0x_platform.h
//VL53L0X_Dev_t lox[3]={0};

VL53L0X lox[3];
//-------------------------------------------------------
//-------------------------------------------------------
//-------------------------------------------------------

#if USE_ETHERNET
#include <SPI.h>
#include <Ethernet.h>
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte mac[] = { 0x02, 0xAB, 0xCD, 0x12, 0x34, 0x56 };


byte gateway[] = {192,168,137,1};
byte *dns = gateway;
byte subnet[] = {255,255,255,0};
IPAddress ip(192,168,137,64);
EthernetServer server(23); // Telnet-style server on port 23
EthernetClient client;
#endif

#if USE_ETHERNET
void ethPrint(const __FlashStringHelper *msg)   { if (client) client.print(msg); }
void ethPrint(const char *msg)                  { if (client) client.print(msg); }
void ethPrint(int val)                          { if (client) client.print((int) val); }
void ethPrint(unsigned long val)                { if (client) client.print((unsigned long) val); }
void ethPrintln(const __FlashStringHelper *msg) { if (client) client.println(msg); }
void ethPrintln(const char *msg)                { if (client) client.println(msg); }
void ethPrintln(int val)                        { if (client) client.println((int) val); }
void ethFlush()                                 { if (client) client.flush(); } 
#endif


#define TIMER_ATMEGA_328P 1
//------------------------------------------------------------------
//                         System Functions
//------------------------------------------------------------------
void(* resetFunc) (void) = 0; //declare reset function @ address 0
//------------------------------------------------------------------



#if TIMER_ATMEGA_328P
//------------------------------------------------------------------
extern volatile unsigned long timer0_millis;
void reset_millis()
{
 noInterrupts();
 timer0_millis = 0;
 interrupts();
}
//------------------------------------------------------------------
#else
//------------------------------------------------------------------
static volatile uint32_t _dwTickCount=0 ;
void reset_millis()
{
 noInterrupts();
 _dwTickCount = 0;
 interrupts();
}
//------------------------------------------------------------------
#endif



//------------------------------------------------------------------
void die_peacefully()
{
    unsigned char i=0;
    while(i<60)
    {
      delay(1000);
      i+=1;
      Serial.print(F("."));
    }
   Serial.println(F("Resetting\n"));
   delay(1000);
  resetFunc();  //call reset
}
//------------------------------------------------------------------
int freeRAM() 
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
//------------------------------------------------------------------
void version()
{
   Serial.print(F("V:"));
   Serial.print((int)VERSION_MAJOR);
   Serial.print(F("."));
   Serial.println(VERSION_MINOR);

   #if USE_ETHERNET
     ethPrint(F("V:"));
     ethPrint((int)VERSION_MAJOR);
     ethPrint(F("."));
     ethPrintln(VERSION_MINOR);
   #endif
}
//------------------------------------------------------------------
//------------------------------------------------------------------


void sram()
{
    Serial.print(F("SRAM("));
    Serial.print(freeRAM());
    Serial.println(F("b free)"));
}


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setI2CDistanceAddresses() 
{
  #if DEBUG_LASERS
    Serial.println(F("Starting VL53L0X 3x array"));  
  #endif

  // all reset
  digitalWrite(laserSwitch[0], LOW);    
  digitalWrite(laserSwitch[1], LOW);    
  digitalWrite(laserSwitch[2], LOW);
  delay(D_T);
  // all unreset
  digitalWrite(laserSwitch[0], HIGH);
  digitalWrite(laserSwitch[1], HIGH);
  digitalWrite(laserSwitch[2], HIGH);
  delay(D_T);

  // activating LOX1 and resetting LOX2/LOX3
  digitalWrite(laserSwitch[0], HIGH);
  digitalWrite(laserSwitch[1], LOW);
  digitalWrite(laserSwitch[2], LOW);

  Wire.begin();

  //Initialize all laser sensors
  for (char i=0; i<3; i++)
  {
    laser_working[i] = 0;

    // Activating Laser Switch
    digitalWrite(laserSwitch[i], HIGH);
    delay(D_T);

    // initing LOX1
    #if DEBUG_LASERS
      Serial.print(F("Free RAM "));
      Serial.print(freeRAM());
      Serial.println(F(" bytes "));
    
      Serial.print(F("VL53L0X "));
      Serial.print((int) i);
      Serial.print(F(" : "));
      Serial.flush();
    #endif


  lox[i].setAddress(LOX_ADDRESS[i]);
  lox[i].setTimeout(500);
  if (lox[i].init())
    {
      lox[i].startContinuous();
      #if DEBUG_LASERS 
        Serial.println(F("Success")); 
      #endif 
      laser_working[i] = 1;  
    } 
    #if DEBUG_LASERS
    else
    {
       Serial.print(F("Failed "));
       Serial.println(laserSwitch[i]); //See below for error values
       //die_peacefully();
    }
    #endif
    delay(D_T);
    delay(100);
  }
 
/*
#define 	VL53L0X_ERROR_NONE   ((VL53L0X_Error) 0)
#define 	VL53L0X_ERROR_CALIBRATION_WARNING   ((VL53L0X_Error) -1)
#define 	VL53L0X_ERROR_MIN_CLIPPED   ((VL53L0X_Error) -2)
#define 	VL53L0X_ERROR_UNDEFINED   ((VL53L0X_Error) -3)
#define 	VL53L0X_ERROR_INVALID_PARAMS   ((VL53L0X_Error) -4)
#define 	VL53L0X_ERROR_NOT_SUPPORTED   ((VL53L0X_Error) -5)
#define 	VL53L0X_ERROR_RANGE_ERROR   ((VL53L0X_Error) -6)
#define 	VL53L0X_ERROR_TIME_OUT   ((VL53L0X_Error) -7)
#define 	VL53L0X_ERROR_MODE_NOT_SUPPORTED   ((VL53L0X_Error) -8)
#define 	VL53L0X_ERROR_BUFFER_TOO_SMALL   ((VL53L0X_Error) -9)
#define 	VL53L0X_ERROR_GPIO_NOT_EXISTING   ((VL53L0X_Error) -10)
*/
}


/*
void deactivateLights(const unsigned char * lights,unsigned char numberOfLights)
{
 for (int i = 0; i < numberOfLights; i++) 
  {
   digitalWrite(lights[i], LOW);
  }
}


void activateLight(const unsigned char * lights,unsigned char numberOfLights,unsigned char lightToActivate)
{
  deactivateLights(lights,numberOfLights);
  // Turn on the next light
  digitalWrite(lights[lightToActivate % numberOfLights], HIGH); 
}
*/
void activate74HC595()
{
  // Manually clear shift register before pins are even configured
  digitalWrite(LATCH_PIN, LOW);
  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(DATA_PIN,  LOW);
  //74HC595 should be clean and ready
  digitalWrite(CLEAR_PIN, HIGH);         // Clear pin disabled
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);  // Outputs enabled from now on
  //--------------------------------------
  use74HC595ForLightControl=1;
}


void deactivate74HC595()
{
  uint8_t lightState = 0;
  updateShiftRegister(lightState);
  digitalWrite(CLEAR_PIN, LOW);           // Clear pin enabled
  digitalWrite(OUTPUT_ENABLE_PIN, HIGH);  // Outputs disabled from now on
  //--------------------------------------
  use74HC595ForLightControl=0;
}


void updateShiftRegister(uint8_t data) 
{
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, data);
  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);  // Outputs enabled from now on
}



void deactivateLightsWithoutChangingLightOn()
{
 if (use74HC595ForLightControl)
 {  
  digitalWrite(CLEAR_PIN, LOW);    // 1 instruction clear using clear pin
  uint8_t lightState = 0;                  // -----------------------------------
  updateShiftRegister(lightState); // Busy-work that shouldn't change something since we also hold clearpin low
  digitalWrite(CLEAR_PIN, HIGH);   // Lights can be reactivated now
 } else
 { 
  for (int i = 0; i < NUMBER_OF_LIGHTS; i++) 
  {
   digitalWrite(directLightControl[i], LOW);
  } 
 }
}


void deactivateLights()
{
 lightOn=NO_LIGHT_ON;
 deactivateLightsWithoutChangingLightOn();
}
 
void activateLight(unsigned char lightToActivate)
{
 if (use74HC595ForLightControl)
 {  
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);  // Outputs enabled from now on
  digitalWrite(CLEAR_PIN, HIGH);         // Make sure clear pin is not triggered
  lightToActivate = lightToActivate % NUMBER_OF_LIGHTS; //Make sure we don't activate past our number of lights
  uint8_t lightState = (1 << lightToActivate);  // Only one light on at a time
  updateShiftRegister(lightState);
  lightOn=lightToActivate % NUMBER_OF_LIGHTS;
 } else
 {
  //Not using 74HC595
  digitalWrite(OUTPUT_ENABLE_PIN, HIGH);  // 74HC595 Outputs should not be enabled
  digitalWrite(CLEAR_PIN, LOW);           // 74HC595 clear pin should be on!
  deactivateLights();
  // Turn on the next light
  if (lightToActivate>NUMBER_OF_LIGHTS)
  {
     //Safe guard against "disabled states"
  } else
  { 
    lightOn=lightToActivate % NUMBER_OF_LIGHTS;
    digitalWrite(directLightControl[lightToActivate % NUMBER_OF_LIGHTS], HIGH);
  }
 }


 if (lightTurnOffMicrosecond>0)
             {
              //If this lightTurnOffMicrosecond is enabled immediately turn off the light! 
              delayMicroseconds(lightTurnOffMicrosecond);
              deactivateLightsWithoutChangingLightOn();
             }
}


#if ENABLE_LASER_LIGHT_DECISION
// Function to choose the appropriate light based on sensor readings
unsigned char chooseClosestLight(const unsigned short * depths) 
{
    //ordering of lights/depths
    //const unsigned char lightPosition[NUMBER_OF_LIGHTS]={0,1,2,3,4,5};
    //const unsigned char distanceSensorPosition[NUMBER_OF_DISTANCE_SENSORS]={0,2,4};
    unsigned char bestLight = 0;
    
    //DISABLED TO FIT IN ARDUINO NANO
    // Convert depth readings into approximate "normal" direction
    // We treat the distance as a "height" component
    float normalX = 0.0f, normalY = 0.0f;
    
    //unsigned int depths[NUMBER_OF_DISTANCE_SENSORS] = {depth1, depth2, depth3};

    for (unsigned char i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) 
    {
        //unsigned char sensorIdx = distanceSensorPosition[i];  // Get sensor index (0-2)
        //unsigned char lightIdx = lightPosition[sensorIdx];    // Find corresponding light position (0-5)
        unsigned char sensorIdx =i*2;  // Get sensor index (0-2)
        unsigned char lightIdx = sensorIdx;    // Find corresponding light position (0-5)
        
        // Convert hexagonal direction to Cartesian unit vectors
        float angle = lightIdx * M_PI / 3.0;  // 60-degree increments
        normalX += cos(angle) * (1.0 / (depths[i] + 1));  // Inverse weighting (closer depth = stronger influence)
        normalY += sin(angle) * (1.0 / (depths[i] + 1));
    }
    
    // Find the light closest to the computed normal direction
    float bestDot = -1.0f;  // Minimum dot product to find max alignment

    for (int i = 0; i < NUMBER_OF_LIGHTS; i++) 
    {
        float lightAngle = (float) ((float) i * M_PI) / 3.0;  // 60-degree increments
        float lightX = (float) cosf(lightAngle);
        float lightY = (float) sinf(lightAngle);

        float dotProduct = normalX * lightX + normalY * lightY;  // Alignment check

        if (dotProduct > bestDot) 
        {
            bestDot = dotProduct;
            bestLight = i;
        }
    }

    return bestLight;
}
#endif


unsigned char getNextLight(const unsigned char currentLight, const char numberOfLights, const char lightMode)
{
  switch (lightMode)
      {
        case 0: //If light mode is 0 or 1
        case 1:
          return (currentLight + 1) % numberOfLights; 
        break;
        //-------------------------------------------
        case 3: // If light mode is 3
         switch (currentLight)
          {
           case 0: return 3; break;
           case 1: return 4; break;
           case 2: return 5; break;
           case 3: return 1; break;
           case 4: return 2; break;
           case 5: return 0; break;
          };
        break;
        //-------------------------------------------
      };
   return currentLight;
}


void read_triple_sensors() 
{
  //Sample all laser sensors
  char i;
  for (i=0; i<3; i++)
  {  
    //VL53L0X_RangingMeasurementData_t measure_tmp={0};
    if (laser_working[i])
    { 
      //if (VL53L0X_ERROR_NONE == getLoxSingleRangingMeasurement(&lox[i],&measure_tmp, false) ) // pass in 'true' to get debug data printout!
      { 
       laser_status[i]               = !lox[i].timeoutOccurred();
       laser_distance_millimeters[i] = lox[i].readRangeContinuousMillimeters();
      }   
    }
  }
}

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//                                 SETUP
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
void setup() 
{  
  // First of all wake up the 74HC595
  //--------------------------------------
  pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
  digitalWrite(OUTPUT_ENABLE_PIN, HIGH);  // Keep outputs disabled at boot

  pinMode(CLEAR_PIN, OUTPUT);
  digitalWrite(CLEAR_PIN, LOW);           // Keep clear pin signaling clear 
  //----------------------------------------
  deactivate74HC595(); //We boot without 74HC595, assuming direct pin control
  //---------------------------------------

  #if USE_ETHERNET
    //This 
    pinMode(ETHERNET_RESET_PIN,OUTPUT);
    digitalWrite(ETHERNET_RESET_PIN, LOW);   //This will now reset ethernet!
  #endif


  //Wait for VIN to stabilize
  delay(10);

  //Direct light control pins
  int i=0;
  for (i=0; i<6; i++)
  {
    pinMode(directLightControl[i], OUTPUT);
    digitalWrite(directLightControl[i],  LOW);
  }



  // Immediately initialize shift register control pins
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  deactivateLights();

  Serial.begin(115200);


  #if DEBUG_LASERS
    Serial.println(F("Arduino Power Cycle...")); 
  #endif

   //Warn Self-check about not enough SRAM being available
   if (freeRAM()< 231)
   {
     //231 bytes works!
     Serial.print(F("Low "));
     sram();
     delay(1000);
     //The arduino will probably restart shortly after reaching this point..
     //If this triggers change SERIAL_RX_BUFFER_SIZE to 8 and SERIAL_TX_BUFFER_SIZE to 16 in 
     //~.arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino/HardwareSerial.h  
   }
  

  //----------------------------------------------------
  // Laser Setup
  //----------------------------------------------------
  #if USE_LASER
    pinMode(laserSwitch[0], OUTPUT);
    pinMode(laserSwitch[1], OUTPUT);
    pinMode(laserSwitch[2], OUTPUT);
  
    #if DEBUG_LASERS
      Serial.println(F("Shutdown pins inited...")); 
    #endif

    digitalWrite(laserSwitch[0], LOW);
    digitalWrite(laserSwitch[1], LOW);
    digitalWrite(laserSwitch[2], LOW);

    #if DEBUG_LASERS
       Serial.println(F("Starting Laser Sensors...")); 
    #endif

    setI2CDistanceAddresses();
  
    #if DEBUG_LASERS
      Serial.println(F("Lasers Ready.")); 
    #endif
  #endif
  //----------------------------------------------------
 
  //2 Analog input buttons
  pinMode(annotationButton, INPUT);  
  pinMode(secondButton,     INPUT);   


  //--------------------------------------
  //       ETHERNET INITIALIZATION LAST
  //--------------------------------------
  #if USE_ETHERNET

    #if DEBUG_ETHERNET
    Serial.println("\n\n\nBegin Ethernet");
    sram();
    #endif

    //Pin is the ethernet reset pin!
    delay(500);
    digitalWrite(ETHERNET_RESET_PIN, HIGH);  //Bring Ethernet online!
    delay(500);
    //Wait for VIN to stabilize
    
    Ethernet.init(10);      //CS chip select is
    Ethernet.begin(mac,ip); //Don't set other stuff..
    delay(100);
    //Ethernet.begin(mac, ip, dns, gateway, subnet);
    #if DEBUG_ETHERNET
    Serial.print("Asked IP:");
    Serial.print(ip[0]);
    Serial.print(".");
    Serial.print(ip[1]);
    Serial.print(".");
    Serial.print(ip[2]);
    Serial.print(".");
    Serial.println(ip[3]);

    Serial.print("Local IP:");
    Serial.println(Ethernet.localIP());
    Serial.print("H/W Status:");
    Serial.println(Ethernet.hardwareStatus());
    if (Ethernet.hardwareStatus()==0) { Serial.println("ETH Failed"); }
    sram();
    #endif

    //Check with : arp -a
    //Check with : ip neigh show
    
    server.begin();
  #endif
  //--------------------------------------
  //--------------------------------------
  //serialOutputEnabled = true; //Turn on output without waiting for i 
}

void comma()
{
  Serial.print(F(","));
  #if USE_ETHERNET
   ethPrint(F(","));
  #endif
}

void zero()
{
  Serial.print(F("0"));
  #if USE_ETHERNET
   ethPrint(F("0"));
  #endif
}

void one()
{
  Serial.print(F("1"));
  #if USE_ETHERNET
   ethPrint(F("1"));
  #endif
}

void failed()
{
  Serial.print(F("F"));
  #if USE_ETHERNET
   ethPrint(F("F"));
  #endif
}

void high()
{
  Serial.print(F("H"));
  #if USE_ETHERNET
   ethPrint(F("H"));
  #endif
}

void number(int theNumber)
{
  Serial.print(theNumber);
  #if USE_ETHERNET
   ethPrint(theNumber);
  #endif
}

void newline()
{
  Serial.print(F("\n"));
  #if USE_ETHERNET
   ethPrint(F("\n"));
  #endif
}


void flush()
{
  Serial.flush();
  #if USE_ETHERNET
  ethFlush();
  #endif
}
 
void reportState(unsigned long currentTime,unsigned char buttonState,unsigned char button2State)
{
             Serial.print(currentTime);
             #if USE_ETHERNET
              ethPrint(currentTime); 
             #endif

             comma();

             Serial.print(buttonState);
             #if USE_ETHERNET
              ethPrint(buttonState); 
             #endif

             comma();

             Serial.print(button2State);
             #if USE_ETHERNET
              ethPrint(button2State); 
             #endif

             comma();
             //-------------------------------------------------------------------------------
             char i; //Use for both next loops
             //-------------------------------------------------------------------------------
             for (i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) 
             {
               if (!laser_working[i])         { failed(); } else
               if (laser_status[i] != 4)      { 
                                                  int distance_mm = (int) laser_distance_millimeters[i];
                                                   if (distance_mm>6000)
                                                   {
                                                    high();
                                                   } else
                                                   {
                                                    number(distance_mm);
                                                   } 
                                              } else 
                                              { zero(); }
               comma();
             }
             //-------------------------------------------------------------------------------
             for (i = 0; i < NUMBER_OF_LIGHTS; i++) 
             {
               if(i!=lightOn) { zero(); } else
                              { one();  }
               if (i!=NUMBER_OF_LIGHTS-1)
                              { comma(); }
             }
            //-------------------------------------------------------------------------------
             newline();
             flush();
}


//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//                                 LOOP
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
void loop() 
{
  unsigned long currentTime = millis();

  // Read button state
  int buttonRaw1 =  analogRead(annotationButton);
  analogRead(SCL_PIN);
  analogRead(SDA_PIN);
  analogRead(I2C_EXTRA_PIN);
  int buttonRaw2 =  analogRead(secondButton);
  /*
  delay(30);
  Serial.print("Button 1 : ");
  Serial.print((int) buttonRaw1);
  Serial.print(" Button 2 : ");
  Serial.println((int) buttonRaw2);*/

  unsigned char buttonState = 0;
  unsigned char button2State = 0;

  if ( (buttonRaw1<312) || (buttonRaw2<312) )
  { //A button was pressed, not sure which!
  if ( (buttonRaw1 == 0 ) && (buttonRaw2 > buttonRaw1) )  { buttonState = 1; } else
  //if ( (buttonRaw1 == 0 ) && (buttonRaw2 == 0)         )  { buttonState = 1; button2State = 1; } else <- Never accept both
  if ( (buttonRaw2 == 0 ) && (buttonRaw1 > buttonRaw2) )  { button2State = 1; }
  }

  char receivedChar = 0;

  #if USE_ETHERNET
  // Accept new client connections or drop disconnected ones
  if (client && !client.connected()) 
  {
    client.stop();   // Cleanly close
    client = EthernetClient(); // Reset to empty
  }

  if (!client) 
  {
    EthernetClient newClient = server.available();
    if (newClient) 
    {
      client = newClient;
      client.flush(); // Optional: clear any junk input
      Serial.println(F("Ethernet client connected"));
    }
  }

  // Read from Ethernet if connected
  if (client && client.available() > 0) 
  {
    receivedChar = client.read();
  }  
#endif


  
  if (Serial.available() > 0) 
  { 
    receivedChar = Serial.read(); 
  } 

  // Check for serial input
  if (receivedChar != 0) 
    {
        switch (receivedChar)
        {
          case 'v':                version(); break; 
          case 'h':
                                   activate74HC595();
                                   reset_millis(); //Reset millisecond counter in a hacky way
                                   serialOutputEnabled = true;
                                   currentTime = 0;
                                   lightStartTime = 0;
                                   autoLights=1;
                                   lightOn=0;
          break;
          case 'i':
                                   deactivate74HC595();
                                   reset_millis(); //Reset millisecond counter in a hacky way
                                   serialOutputEnabled = true;
                                   currentTime = 0;
                                   lightStartTime = 0;
                                   autoLights=1;
                                   lightOn=0;
                                   //Serial.begin(115200);
                                   //Serial.println(F("dev_ts,B1,L1,L2,L3,L4,L5,L6,D1,D2,D3")); 
                                   //Serial.flush();
                                   //delay(1000); //Give some time to the receiver to not miss the first broadcast
          break;
          case 'o': lightTurnOffMicrosecond  = 0;   break; //Pulse lights
          case 'p': lightTurnOffMicrosecond += 500; break; //Pulse lights
          case 'r': autoLights=1; lightOn = 0; break; 
          case 'a': autoLights=2; break; 
          case 't': autoLights=3; lightOn = 0; break; 
          case 'y': autoLights=4; lightOn = 0; break; 
          case 'z': deactivateLights(); lightOn=NO_LIGHT_ON; autoLights=0; resetFunc(); break;
          case '0': deactivateLights(); lightOn=NO_LIGHT_ON; autoLights=0; break; 
          case '1': activateLight(0);   lightOn=0; autoLights=0; break; 
          case '2': activateLight(1);   lightOn=1; autoLights=0; break; 
          case '3': activateLight(2);   lightOn=2; autoLights=0; break; 
          case '4': activateLight(3);   lightOn=3; autoLights=0; break;  
          case '5': activateLight(4);   lightOn=4; autoLights=0; break; 
          case '6': activateLight(5);   lightOn=5; autoLights=0; break; 
           #if USE_ETHERNET
          case 'x': 
            Serial.print("Local IP:");
            Serial.println(Ethernet.localIP());
            Serial.print("H/W Status:");
            Serial.println(Ethernet.hardwareStatus());
            sram();
            break;
          #endif
          case 'f': serialOutputEnabled = false; break;
          case '+': autoLights=0;
                    //lightOn = (lightOn + 1) % NUMBER_OF_LIGHTS;
                    lightOn = getNextLight(lightOn,NUMBER_OF_LIGHTS,autoLights);
                    activateLight(lightOn);
                    lightStartTime = currentTime;//New light just started
                    reportState(currentTime,buttonState,button2State);
          break;
        };
    }



   // Ensure loop runs at ~50Hz
   if (currentTime - lastUpdateTime >= loopRateMsec) 
    {
        lastUpdateTime = currentTime;

        #if USE_LASER 
         read_triple_sensors();
        #endif

        // Check if it's time to change the light/emmit messages
        if (autoLights != 0)
        {
        if (currentTime - lightStartTime >= lightDurationMsec) 
        {
            lightStartTime = currentTime;  // Reset light timer

            if ( (autoLights==1) || (autoLights>=3) )
            { 
             // Update light index
             //lightOn = (lightOn + 1) % NUMBER_OF_LIGHTS;
             lightOn = getNextLight(lightOn,NUMBER_OF_LIGHTS,autoLights);
             activateLight(lightOn);

             if ( (autoLights==4) && (currentTime > 10000) )
             { 
                //When the light sensor is turned on it begins flashing 
                //for 10 seconds, then it turns off!
                autoLights = 0;
                deactivateLights();
             }
            }
            #if ENABLE_LASER_LIGHT_DECISION
            else if (autoLights==2)
            {
              lightOn = chooseClosestLight(laser_distance_millimeters);
              activateLight(lightOn);
            }
            #endif

            //Serial Output--------------------------------
            if (serialOutputEnabled) 
            {
              reportState(currentTime,buttonState,button2State);
            }
            //-------------------------------------------
        }
      }
    }
   
  
  int sleepDelay = (CPUSleepTimeMilliseconds*1000) - lightTurnOffMicrosecond;
  if (sleepDelay > 0) {
                        delayMicroseconds(sleepDelay);
                      }
}
