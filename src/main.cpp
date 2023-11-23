#include <Arduino.h>
#include <Servo.h>

#define TRANSMITTER true

#include <HT_hamming_encoder.h>
#include <HT_light_modulator.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define RF95_FREQ 433.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#if TRANSMITTER
// inputs
HT_PhotoTransmitter laser;

// outputs
const int xServoPin = 5;
const int yServoPin = 13;
Servo XServo; // 180 servo
Servo YServo; // 360 servo

//globals
bool connected = false;

/////////  INTERRUPT //////////
ISR(TIMER3_COMPA_vect){
  laser.transmit();
}

/////////  SETUP  //////////
void setup() {
  laser.set_speed(500);      // must be 500+ bits/second and less than laser slew rate
  laser.begin();

  XServo.attach(xServoPin);
  XServo.write(84);

  YServo.attach(yServoPin);
  YServo.write(90);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}


int16_t packetnum = 0;  // packet counter, we increment per xmission
bool verify_laser_connection(){
  //SEND LASER
  while(laser.get_send_flag());
  uint16_t msg = hamming_byte_encoder(12);
  laser.manchester_modulate(msg);
  delay(100); // TODO: Delay amount

  //SEND LORA
  char radiopacket[2] = "S";
  radiopacket[1] = 0;
  delay(10);
  rf95.send((uint8_t *)radiopacket, 2);
  delay(10);
  rf95.waitPacketSent();

  //RECEIVE LORA
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  Serial.println("Waiting for reply...");
  bool ret = false;
  if (rf95.waitAvailableTimeout(1000))
  { 
    if (rf95.recv(buf, &len))
   {
      if(buf[0] == 'S') ret = true;
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }

  return ret;
}

/////////  LOOP  //////////
void loop() {

  while(true){
    verify_laser_connection();
  }

  // while(true){
  //   for(int i = 0; i <= 180; i+=5){
  //     XServo.write(i);
  //     for(int j = 0; j <= 180; j+=5){
  //       YServo.write(j);
  //       delay(100);
  //     }
  //     i+=5;
  //     XServo.write(i);
  //     for(int j = 0; j >= 180; j-=5){
  //       YServo.write(j);
  //       delay(100);
  //     }
  //   }
  //   delay(250);
  // }

lost_connection:
  while(!connected){
    for(int i = 0; i <= 180; i+=5){
      XServo.write(i);
      for(int j = 0; j <= 180; j+=5){
        YServo.write(j);
        delay(20);
        if(verify_laser_connection()){
          connected = true;
          goto found_connection;
        }
      }
      i+=5;
      XServo.write(i);
      for(int j = 0; j >= 180; j-=5){
        YServo.write(j);
        delay(20);
        if(verify_laser_connection()){
          connected = true;
          goto found_connection;
        }
      }
    }
  }

found_connection:
  while(connected){
    static int counter = 1;
    if(counter%100 == 0){
      if(!verify_laser_connection()){
        connected = false;
        goto lost_connection;
      }
    }else{
      char incomingByte;
      uint16_t msg;
      incomingByte = Serial.read();
      if(incomingByte != -1){    
        Serial.print(incomingByte);
        while(laser.get_send_flag());
        msg = hamming_byte_encoder(incomingByte);
        laser.manchester_modulate(msg);
      }
    }
    delay(50);
    counter++;
  }
}
#else  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define LED 13

//input
HT_PhotoReceiver pdiode;

ISR(TIMER3_COMPA_vect){
  pdiode.receive();
}

/////////  SETUP  //////////
void setup() {
  pdiode.set_speed(500);      // bits/sec, must match laser bit transfer rate
  pdiode.begin();

  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  Serial.println("GOT SERIAL");

  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setTxPower(23, false);
}

/////////  LOOP  //////////
bool found = false;
void loop() {
  if(rf95.available()){
   // Serial.println("HERE");
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      char p = 'E';
      if(found){
        uint8_t data[] = "S";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }else{
        uint8_t data[] = "E";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
    found = false;
  }else{
    char p;
    while(pdiode.get_byte(&p)){
      if(p == 'S'){
        found = true;
      }
      Serial.print(p);
    }
    delay(10);
  }
}

#endif