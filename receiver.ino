
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "IoShoe.h"


using IoShoe::EventStorage;



#define FIO true

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error This code relies on little endian integers!
#endif

/* 
 * ===== Global Variables ===== 
 */
// IO
int buttonPin = 3;

// ========= Shoelace LED vars =============
long lastLightUpTime = 0;
#define SHOELACE_LED_ON_TIME 1000
int shoelacePin = 6;
bool shoelaceState = false; // determine whether shoelace led is on or off

#define BROADCAST_DELAY 1000

// ========= Tactile Switch & Debouncer ====

int buttonState;
int lastButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;


// === Time of flight ====
volatile uint32_t round_trip_timer = 0;

// ========= MsgStorage ====================
EventStorage evtStorage = EventStorage();

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
// bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);
/**********************************************************/

// Use the same address for both devices
uint8_t address[] = { "radio" };


struct dataStruct{
  unsigned long _micros;
  float value;
}myData;

// Caliberation

uint8_t self_device_id = millis();

struct calliberationStruct{
  unsigned long _micros;
  uint8_t self_device_id;
  uint8_t receiver_device_id;
}calliberationData;

// ========= Main ==========================
void setup() {
  pinMode(shoelacePin, OUTPUT);
  pinMode(buttonPin, INPUT);


  // set initial LED state
  digitalWrite(shoelacePin, shoelaceState);

  Serial.begin(115200);
  Serial.println(F("Light broadcasting using RF24"));

  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses

  radio.openWritingPipe(address);             // communicate back and forth.  One listens on it, the other talks to it.
  radio.openReadingPipe(1,address); 
    // Start the radio listening for data
  radio.startListening();

  attachInterrupt(0, check_radio, LOW);             // Attach interrupt handler to interrupt #0 (using pin 2) on BOTH the sender and receiver
}

void send() {
  radio.stopListening();                                    // First, stop listening so we can talk.

  Serial.println(F("Now sending"));

  myData._micros = micros();

  radio.startWrite(&myData, sizeof(myData),0);

  // if (!radio.write( &myData, sizeof(myData) )){
  //  Serial.println(F("failed"));
  // }
   // Now, continue listening
    Serial.print(F("Sent data "));
    Serial.println(myData._micros);  
    evtStorage.saveEvent(myData._micros);

  }

 void sendCalliberationSignal() {
  radio.stopListening();                                    // First, stop listening so we can talk.

  Serial.println(F("Now sending calliberation"));

  calliberationData.self_device_id = self_device_id;
  calliberationData.receiver_device_id = 0;
  calliberationData._micros = micros();

   radio.startWrite(&calliberationData, sizeof(calliberationData),0);

    Serial.print(F("Sent data "));
    Serial.println(calliberationData._micros);  
    evtStorage.saveEvent(calliberationData._micros);
  }

unsigned long signalStrength = 0;

void loop() {

   /*
   *  Update Shoelace LED state
   */
   if ((millis() - lastLightUpTime) > SHOELACE_LED_ON_TIME) {
      digitalWrite(shoelacePin, LOW); 
      shoelaceState = false;
   } 

  /*
   *  Check gesture/direct input
   */
  if (shoelaceState == false) { 
    // Only when shoelace led is off we will then allow more direct input
    int reading = digitalRead(buttonPin);

    // If the switch changed, due to pressing (or noise):
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // if the button state has changed 
      if(reading != buttonState) {
    
          buttonState = reading; 
    
          if(buttonState == HIGH) {
    
            digitalWrite(shoelacePin, HIGH);
            shoelaceState = true;
            lastLightUpTime = millis();
            send() ;
          }
      
      }
      
    }
    lastButtonState = reading;
  }

   
 }

 void check_radio(void)                                // Receiver role: Does nothing!  All the work is in IRQ
{
  
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);                     // What happened?

 
  // If data is available, handle it accordingly
  if ( rx ){
    
    bool rpd = radio.testRPD();

    // Read in the data
    uint8_t received;
    radio.read( &myData, sizeof(myData) );             // Get the payload

    // If this is a ping, send back a pong
    Serial.print(F("Got signal: "));
      Serial.print(myData._micros); 
      Serial.print(F(", Hops = "));
      Serial.print(myData.value); 
      Serial.print(F(", Strength =  "));
      Serial.println(signalStrength); 
      
    bool exist = evtStorage.checkEventExist(myData._micros);

        if(!exist) {
          digitalWrite(shoelacePin, HIGH);
          shoelaceState = true;
          lastLightUpTime = millis();
          delay(BROADCAST_DELAY);
          myData.value++;
          radio.startWrite( &myData, sizeof(myData), 0);   // 0 is quite important but i dont know why yet
          evtStorage.saveEvent(myData._micros);
        } else {
          Serial.print("signal exists: ");  
          Serial.print(myData._micros);  
          Serial.print(",");
          Serial.print(myData.value);
          Serial.println("");  
        }

  }


  radio.startListening(); 

  // Start listening if transmission is complete
  if( tx || fail ){
     radio.startListening(); 
     Serial.println(tx ? F(":OK") : F(":Fail"));
  }  
}




/****************** Change Roles via Serial Commands ***************************/

  // if ( Serial.available() )
  // {
  //   char c = toupper(Serial.read());
  //   if ( c == 'T' && role == 0 ){      
  //     Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
  //     role = 1;                  // Become the primary transmitter (ping out)
    
  //  }else
  //   if ( c == 'R' && role == 1 ){
  //     Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
  //      role = 0;                // Become the primary receiver (pong back)
  //      radio.startListening();
       
  //   }
  // }


// Detect a direct input event (btn pressed or gesture detected)
void onEventGenerated() {
  // Generate a key as a message, the key is used to 
  // differentiate events. Each event is associated with an unique key/msg
  uint8_t evt_id = evtStorage.createAndSaveEvent();
  // if (!FIO) {
  //   Serial.print("Generate event: ");
  //   Serial.println(evt_id);
  // }
  // delay(BROADCAST_DELAY);
  // broadcastEvent(evt_id);

}


// ========= RF functions ================
void broadcastEvent(uint8_t evt_id) {

    // uint8_t payload[] = {evt_id};

    // // SH + SL Address of receiving XBee
    // XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000FFFF);
    // ZBExplicitTxRequest zbTx(addr64, payload, sizeof(payload));
    // xbee.send(zbTx);
 
}


// Detect a wireless input event (e.g, Receive a XBee message)
void onEventReceived() {
  // got a zb rx packet
      

      
}

