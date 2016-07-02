/*============================ Include ============================*/

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <i2cmaster.h>

/*============================ Macro ============================*/

#define msleep(X) delay(X)


#define DEBUG

#ifndef DEBUG
#define PRINTF(...);
#define setupSerial();
#else
#define setupSerial(); {Serial.begin(57600); printf_begin();}
#define PRINTF printf
#endif

#define RELAY_ON {\
  digitalWrite(RELAY_PIN, HIGH);\
  gRelayOn = 1;\
}
#define RELAY_OFF {\
  digitalWrite(RELAY_PIN, LOW);\
  gRelayOn = 0;\
}
#define ERROR(str) {\
  RELAY_OFF\
  PRINTF("ERROR\n", str);\
  gState = ERROR;\
}

#define RELAY_PIN        6
#define LED_PIN          13
#define WATER_SENSOR_PIN A1

static unsigned int  kTemperatureTrigValue = 45;
static unsigned int  kTemperatureMin       = 58;
static unsigned int  kTemperatureMax       = 60;
static unsigned int  kWaterTrigLevel       = 1;
static unsigned long kEndingDelay          = 10000;

/*============================ Type ============================*/

typedef enum{STOP, ONGOING, KEEP, ERROR} tState;

/*============================ Global variable ============================*/

// Set up nRF24L01 radio on SPI pin for CE, CSN
RF24                radio(9,10);
static tState       gState = STOP;
static unsigned int gCoffeeEnable = 0;
static unsigned int gRelayOn = 0;

/*============================ Local Function interface ============================*/

static void         setupNRF24(void);
static void         send(char * iData);
static void         received(char* oData, long iMsTimeOut);
static void         initTemperatureSensor(void);
static unsigned int getTemperature(void);
static unsigned int getWaterLevel(void);


/*============================ Function implementation ============================*/

/*------------------------------- Main functions -------------------------------*/

void setup(void){
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupSerial();
  PRINTF("Coffee machine\n\r");

  initTemperatureSensor();
  setupNRF24();

  // Relay control
  pinMode(RELAY_PIN, OUTPUT);
  RELAY_OFF;
 
  if(getWaterLevel() != 0){
    gState = ONGOING;
    gCoffeeEnable = 1;
    PRINTF("Ongoing at init\n");
  }
  else{
    gState = STOP;
  }
}

void loop(void){
  char                 lStr[32] = {};

  radio.powerUp();
  received(lStr, 10);
  // if received buffer not empty
  if(lStr[0] != '\0'){
    processCmd(lStr);
    sprintf(lStr,"Ack");
    send((char*)lStr);
  }
  radio.powerDown();

  resolveState();
  updateState();
  updateRelay();
}

/*------------------------------- Main functions coffee machine ----------------------*/


static void updateRelay(void){
  unsigned int lTemp;

  if(  (gState == STOP)
     &&(gRelayOn)){
    RELAY_OFF
  }else if(  (gState == ONGOING)
           &&(!gRelayOn)){
    RELAY_ON
  }else if(gState == KEEP){
    lTemp = getTemperature();
    if(  (lTemp < kTemperatureMin)
       &&(!gRelayOn)){
      RELAY_ON
    }else if(  (lTemp > kTemperatureMax)
             &&(gRelayOn)){
      RELAY_OFF
    }
  }
}

static void processCmd(char* str){
  PRINTF("Cmd received : %s\n", str);
}

static void resolveState(void){
  static unsigned long lTimeWhenWaterGoesOff = 0;

  switch(gState){
    case STOP:
      if(gCoffeeEnable){
        if(getWaterLevel() >= kWaterTrigLevel){
          gState = ONGOING;
        }else if(getTemperature() >= kTemperatureTrigValue){
          gState = KEEP;
        }
      }
      break;
    case ONGOING:
      if(gCoffeeEnable == 0){
        lTimeWhenWaterGoesOff = 0;
        gState = STOP;
      }else if(getWaterLevel() == 0){
        if(lTimeWhenWaterGoesOff == 0){
          lTimeWhenWaterGoesOff = millis();
        }else if(millis() >= lTimeWhenWaterGoesOff + kEndingDelay){
          gState = KEEP; // end
        }
        // else --> count down before end
      }else{
        lTimeWhenWaterGoesOff = 0;
      }
      break;
    case KEEP:
      if(gCoffeeEnable == 0){
        gState = STOP;
      }else if(getWaterLevel() >= kWaterTrigLevel){
        gState = ONGOING;
      }
      break;
    case ERROR:
      // chip reset :
      // be carefull it does not reset hardware peripherals
      asm volatile ("  jmp 0");
      break;
    default:
      ERROR("Unknow state")
      break;
  }
}

static void updateState(void){
  static tState lOldState = STOP;
  if(lOldState != gState){
    printState();
    lOldState = gState;
  }
}

/*------------------------------- Debug coffee machine -------------------------------*/

static void printState(void){
  PRINTF("Update State\n");
  switch(gState)
  {
    case STOP:
      PRINTF("Coffee machine stop\n");
      break;
    case ONGOING:
      PRINTF("Coffee in progress\n");
      break;
    case KEEP:
      PRINTF("Coffee ketp warm\n");
      break;
    case ERROR:
      PRINTF("Coffee machine error\n");
      break;
    default:
      PRINTF("unknow ..\n");
      break;
  }
}

/*------------------------------- nRF24 -------------------------------*/

static void setupNRF24(void){
  radio.begin();
  radio.enableDynamicPayloads();
  radio.setRetries(20,10);
  radio.openWritingPipe(0xF0F0F0F0E2LL);
  radio.openReadingPipe(1,0x7365727632LL);
  radio.startListening();
  radio.printDetails();
  msleep(100);
  radio.powerDown(); 
}

static void send(char* iData){
  if(strlen(iData) <= 32){
    radio.stopListening();
    if(!radio.write(iData, strlen(iData))){
      PRINTF("Radio.write failed : %s\n\r", iData);
    }
    radio.startListening();
  }else{
    PRINTF("Data to long  : %s\n\r", iData);
  }
}

static void received(char* oData, long iMsTimeOut){
  long lTimeReference;
  long lTime;

  lTimeReference = millis();
  lTime = lTimeReference;

  //msleep(iMsTimeOut); /* TODO */
  while(  ((lTime - lTimeReference) < iMsTimeOut)
        ||(iMsTimeOut == -1)){
    if(radio.available()){
      uint8_t len = radio.getDynamicPayloadSize();
      if(len <= 32){
        radio.read(oData, len);
        oData[len] = 0;
      }else{
        PRINTF("Error in received function\n\r");
        oData[0] = 0;
      }
      return;
    }
    lTime = millis(); 
  }
  //PRINTF("Received time out %10ld %10ld %10ld\n\r", lTimeReference, lTime, lTime - lTimeReference);
  oData[0] = 0;
}

/*------------------------------- Temperature sensor -------------------------------*/

static void initTemperatureSensor(void){
  // temperature sensor communication bus
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups  
}

static unsigned int getTemperature(void){
  int data_low, data_high;

  PRINTF("---------------------\n- start I2C\n");
  i2c_start_wait((0x5A << 1) + I2C_WRITE);

  PRINTF("- 1er write\n");
  i2c_write(0x07);

  PRINTF("- rep start\n");
  i2c_rep_start((0x5A << 1) + I2C_READ);

  PRINTF("- 1st read\n");
  data_low  = i2c_readAck(); //Read 1 byte and then send ack

  PRINTF("- 2nd read\n");
  data_high = i2c_readAck(); //Read 1 byte and then send ack

  PRINTF("- 3rd read\n");
  i2c_readNak();

  PRINTF("- stop I2C\n");
  i2c_stop();

  return (unsigned int)((((double)(((data_high & 0x007F) << 8) + data_low)) * 0.02 ) - 273.16) ;
}

/*------------------------------- Water sensor -------------------------------*/

static unsigned int getWaterLevel(void){
  return (unsigned int)((500 - analogRead(WATER_SENSOR_PIN)) / 10);
}

