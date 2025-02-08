#include <CAN.hpp>

//faster to type
#define p(msg)        Serial.print(msg)
#define pd(msg,fill)  Serial.print(msg,fill)
#define pl(msg)       Serial.println(msg)

uint8_t counter = 0;
uint8_t frameLength = 0;
unsigned long previousMillis = 0;     // stores last time output was updated
const long interval = 1000;           // transmission interval (milliseconds)

void setup() {
  Serial.begin(9600);
 
  bool ret = CANInit(CAN_500KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  // bool ret = CANInit(CAN_500KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_500KBPS, 3);  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
  //bool ret = CANInit(CAN_1000KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  //bool ret = CANInit(CAN_1000KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_1000KBPS, 3);  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
  if (!ret) while(true);
}

void loop() {
  CAN_msg_t CAN_TX_msg;
  CAN_msg_t CAN_RX_msg;

  CAN_TX_msg.data[0] = 0x00;
  CAN_TX_msg.data[1] = 0x01;
  CAN_TX_msg.data[2] = 0x02;
  CAN_TX_msg.data[3] = 0x03;
  CAN_TX_msg.data[4] = 0x04;
  CAN_TX_msg.data[5] = 0x05;
  CAN_TX_msg.data[6] = 0x06;
  CAN_TX_msg.data[7] = 0x07;
  CAN_TX_msg.len = frameLength;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if ( ( counter % 2) == 0) {
      CAN_TX_msg.type = DATA_FRAME;
      if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.id = 0x32F103;
    } else {
      CAN_TX_msg.type = DATA_FRAME;
      if (CAN_TX_msg.len == 0) CAN_TX_msg.type = REMOTE_FRAME;
      CAN_TX_msg.format = STANDARD_FORMAT;
      CAN_TX_msg.id = 0x103;
    }
    CANSend(&CAN_TX_msg);
    pl("Sent CAN message");
    frameLength++;
    if (frameLength == 9) frameLength = 0;
    counter++;
  }
  
  // if(CANMsgAvail()) {
  //   pl("CAN msg available");
  //   CANReceive(&CAN_RX_msg);

  //   if (CAN_RX_msg.format == EXTENDED_FORMAT) {
  //     p("Extended ID: 0x");
  //     if (CAN_RX_msg.id < 0x10000000) p("0");
  //     if (CAN_RX_msg.id < 0x1000000)  p("0");
  //     if (CAN_RX_msg.id < 0x100000)   p("0");
  //     if (CAN_RX_msg.id < 0x10000)    p("0");
  //     if (CAN_RX_msg.id < 0x1000)     p("0");
  //     if (CAN_RX_msg.id < 0x100)      p("0");
  //     if (CAN_RX_msg.id < 0x10)       p("0");
  //     pd(CAN_RX_msg.id, HEX);
  //   } else {
  //     p("Standard ID: 0x");
  //     if (CAN_RX_msg.id < 0x100)      p("0");
  //     if (CAN_RX_msg.id < 0x10)       p("0");
  //     pd(CAN_RX_msg.id, HEX);
  //     p("     ");
  //   }

  //   p(" DLC: ");
  //   p(CAN_RX_msg.len);
  //   if (CAN_RX_msg.type == DATA_FRAME) {
  //     p(" Data: ");
  //     for(int i=0; i<CAN_RX_msg.len; i++) {
  //       p("0x"); 
  //       pd(CAN_RX_msg.data[i], HEX); 
  //       if (i != (CAN_RX_msg.len-1))  p(" ");
  //     }
  //     pl();
  //   } else {
  //     pl(" Data: REMOTE REQUEST FRAME");
  //   }
  // }
    
  

}