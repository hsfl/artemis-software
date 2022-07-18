#include <Wire.h>
#include <Adafruit_INA219.h>
#include <USBHost_t36.h>
#include <TimeLib.h>

USBHost usb;

/* ----- CREATE CURRENT SENSOR OBJECTS ----- */
Adafruit_INA219 INA219_1(0x40);

/* ----- GLOBAL VARIABLES ----- */
#define RPI_ENABLE 36
bool IS_PI_ON;
time_t PI_TIMER;

#define SW_CMD_LEN 11
String SW_CMD[SW_CMD_LEN] = {"CMD: SW_3V3_1", 
                            "CMD: SW_3V3_2",
                            "CMD: SW_5V_1",
                            "CMD: SW_5V_2",
                            "CMD: SW_5V_3",
                            "CMD: SW_5V_4",
                            "CMD: SW_12V",
                            "CMD: VBATT",
                            "CMD: WDT",
                            "CMD: HBRIDGE1",
                            "CMD: HBRIDGE2"}; 
                               
#define BURN_CMD_LEN 3
String BURN_CMD[BURN_CMD_LEN] = {"CMD: BURN1",
                                "CMD: BURN2",
                                "CMD: BURN_5V"};
                                
#define CMD_3V3_LEN 2
String CMD_3V3[CMD_3V3_LEN] = {"CMD: SW_3V3_1", 
                  "CMD: SW_3V3_2"};
                  
#define CMD_5V_LEN 4
String CMD_5V[CMD_5V_LEN] = {"CMD: SW_5V_1",
                            "CMD: SW_5V_2",
                            "CMD: SW_5V_3",
                            "CMD: SW_5V_4"};

String UART1_RX = "";

enum STATE{SINGLE_SWITCHING, SINGLE_BURN, SET_VBUS, SET_BURN, ALL_CMDS};
enum STATE CURRENT_STATE;

/* ----- DECLARE FUNCTIONS ----- */
void CHECK_PI_STATE();
bool READ_CMD();
void INA219_INIT();
void INA219_GET_DATA();
void PDU_UART_SEND();
bool PDU_UART_RECV();

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  usb.begin();

  pinMode(RPI_ENABLE, OUTPUT);
  digitalWrite(RPI_ENABLE, LOW);
  IS_PI_ON = false;
  PI_TIMER = now();

  INA219_INIT();
  CURRENT_STATE = SINGLE_SWITCHING;
}

void loop() {
  PDU_UART_RECV();
  CHECK_PI_STATE();

//  while(Serial.available() > 0) {
//    String str = Serial.readString();
//    PDU_UART_SEND(str);
//  }
  
  Serial.print("\nRUNNING STATE: ");
  Serial.println(CURRENT_STATE);
  Serial.println();

  switch(CURRENT_STATE) {
    case SINGLE_SWITCHING:
      for(int i = 0; i < SW_CMD_LEN; i++) {
        PDU_UART_SEND(SW_CMD[i] + " ENABLE");
        while(!PDU_UART_RECV());
        delay(1000 * 15);
        PDU_UART_SEND(SW_CMD[i] + " DISABLE");
        while(!PDU_UART_RECV());
        CHECK_PI_STATE();
      }
      CURRENT_STATE = SINGLE_BURN;
      break;
      
    case SINGLE_BURN:
      for(int i = 0; i < BURN_CMD_LEN; i++) {
        PDU_UART_SEND(BURN_CMD[i] + " ENABLE"); 
        while(!PDU_UART_RECV());
        delay(1000 * 60);
        PDU_UART_SEND(BURN_CMD[i] + " DISABLE");
        while(!PDU_UART_RECV());
      }
      CURRENT_STATE = SET_VBUS;
      break;
      
    case SET_VBUS:
      for(int i = 0; i < CMD_3V3_LEN; i++) {
        PDU_UART_SEND(CMD_3V3[i] + " ENABLE");
        while(!PDU_UART_RECV());
      }
      delay(1000 * 60);
      for(int i = 0; i < CMD_3V3_LEN; i++) {
        PDU_UART_SEND(CMD_3V3[i] + " DISABLE");
        while(!PDU_UART_RECV());
      }
      for(int i = 0; i < CMD_5V_LEN; i++) {
        PDU_UART_SEND(CMD_5V[i] + " ENABLE");
        while(!PDU_UART_RECV());
      }
      delay(1000 * 60);
      for(int i = 0; i < CMD_5V_LEN; i++) {
        PDU_UART_SEND(CMD_5V[i] + " DISABLE");
        while(!PDU_UART_RECV());
      }
      PDU_UART_SEND("CMD: SW_12V ENABLE");
      while(!PDU_UART_RECV());
      delay(1000 * 60);
      PDU_UART_SEND("CMD: SW_12V DISABLE");
      while(!PDU_UART_RECV());
      CURRENT_STATE = SET_BURN;
      break;
      
    case SET_BURN:
      for(int i = 0; i < BURN_CMD_LEN; i++) {
        PDU_UART_SEND("BURN ENABLE");
        while(!PDU_UART_RECV());
      }
      delay(1000 * 15);
      for(int i = 0; i < BURN_CMD_LEN; i++) {
        PDU_UART_SEND("BURN DISABLE");
        while(!PDU_UART_RECV());
      }
      CURRENT_STATE = ALL_CMDS;
      break;
      
    case ALL_CMDS:
      PDU_UART_SEND("CMD: GPIO ON ALL");
      while(!PDU_UART_RECV());
      delay(1000 * 60);
      PDU_UART_SEND("CMD: GPIO OFF ALL");
      while(!PDU_UART_RECV());
      CURRENT_STATE = SINGLE_SWITCHING;
      break;
  }
}

/* ----- Switch Pi On/Off ----- */
void CHECK_PI_STATE() {
  if(!IS_PI_ON && now() - PI_TIMER > 10) {
    digitalWrite(RPI_ENABLE, HIGH);
    PI_TIMER = now();
    IS_PI_ON = true;
  } else if(IS_PI_ON && now() - PI_TIMER > 60*30){
    digitalWrite(RPI_ENABLE, LOW);
    PI_TIMER = now();
    IS_PI_ON = false;
  }
}

/* ----- INITIALIZE CURRENT SENSOR ----- */
void INA219_INIT() {
  if(!INA219_1.begin(&Wire2)) {
    Serial.println("Failed to find INA219 with address 0x40");
  } else {
    Serial.println("INA219 0x40 Initialized!");
  }
}

/* ----- GET CURRENT SENSOR DATA ----- */
void INA219_GET_DATA(uint16_t addr) {
  Serial.println("Current Sensor: ");
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  switch(addr) {
    case 0x40:
      shuntvoltage = INA219_1.getShuntVoltage_mV();
      busvoltage = INA219_1.getBusVoltage_V();
      current_mA = INA219_1.getCurrent_mA();
      power_mW = INA219_1.getPower_mW();
      break;
  }

  loadvoltage = busvoltage + (shuntvoltage / 1000);
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
}

/* ----- SEND MESSAGE TO PDU THROUGH UART ----- */
void PDU_UART_SEND(String buf) {
  Serial1.print(buf.c_str());
  Serial1.print('\n');
  Serial.print("SENDING TO PDU: ");
  Serial.print(buf.c_str());
  Serial.print('\n');
}

/* ----- RECEIVE MESSAGE FROM PDU THROUGH UART ----- */
bool PDU_UART_RECV() {
  if (Serial1.available() > 0) {
    UART1_RX = Serial1.readString();
    if(UART1_RX.length() > 0) {
      Serial.print("UART RECV: ");
      Serial.println(UART1_RX);
      return true;
    }
  }
  return false;
}
