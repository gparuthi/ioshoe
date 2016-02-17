
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
int ledPin = 2;

// ========= Shoelace LED vars =============
long lastLightUpTime = 0;
#define SHOELACE_LED_ON_TIME 1000
int shoelacePin = 2;
bool shoelaceState = false; // determine whether shoelace led is on or off

#define BROADCAST_DELAY 1000

// ========= Tactile Switch & Debouncer ====
int ledState = LOW;
int buttonState;
int lastButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;



// ========= MsgStorage ====================
EventStorage evtStorage = EventStorage();

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
// bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);
/**********************************************************/

byte addresses[][6] = {"RCh","RCh"};

struct dataStruct{
  unsigned long _micros;
  float value;
}myData;


// ========= Main ==========================
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);


  // set initial LED state
  digitalWrite(ledPin, ledState);

  Serial.begin(115200);
  Serial.println(F("Light broadcasting using RF24"));

  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  
  myData.value = 0.0;

  // Start the radio listening for data
  radio.startListening();
}

void send() {
  radio.stopListening();                                    // First, stop listening so we can talk.


  Serial.println(F("Now sending"));

  myData._micros = micros();
  if (!radio.write( &myData, sizeof(myData) )){
   Serial.println(F("failed"));
  }
    radio.startListening();                                    // Now, continue listening
    Serial.print(F("Sent data "));
    Serial.println(myData._micros);  
    // digitalWrite(ledPin, HIGH);
    // // Don't do anything for the delay time
    //  delay(1000);
    //  digitalWrite(ledPin, LOW);

    evtStorage.saveEvent(myData._micros);

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
            //flashLed(ledPin, ON_EVT_GENERATED);
    
            digitalWrite(shoelacePin, HIGH);
            shoelaceState = true;
            lastLightUpTime = millis();
            send() ;
          }
      
      }
      
    }
    lastButtonState = reading;
  }



/****************** Pong Back Role ***************************/


    unsigned long got_time;
    
    radio.startListening();

    if( radio.available()){


      bool rpd = radio.testRPD();
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        signalStrength = rpd + 1;
        radio.read( &myData, sizeof(myData) );             // Get the payload
      }
     
      radio.stopListening();       

      
      Serial.print(F("Got signal: "));
      Serial.print(myData._micros); 
      Serial.print(F(", Hops = "));
      Serial.print(myData.value); 
      Serial.print(F(", Strength =  "));
      Serial.println(signalStrength);  
      
      // if (signalStrength < 2)
      // {
      //   // Serial.print("<far>");
      //   return;
      // }

        // if the gottime is not in hashset then add got time in hashset and set
        // else do nothing
        bool exist = evtStorage.checkEventExist(myData._micros);

        if(!exist) {
          digitalWrite(shoelacePin, HIGH);
          shoelaceState = true;
          lastLightUpTime = millis();

          delay(BROADCAST_DELAY);
          
          myData.value++;

          radio.write( &myData, sizeof(myData) );  

          evtStorage.saveEvent(myData._micros);

        } else {
          Serial.print("signal exists: ");  
          Serial.print(myData._micros);  
          Serial.print(",");
          Serial.print(myData.value);
          Serial.println("");  
        }
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

