
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
int ledState = LOW;

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
  unsigned long sourceDeviceId;
  unsigned long _micros;
  unsigned long value;
  bool signalType; // 0 = range calliberation, 1 = LED trigger
}myData;


// ========= Calliberation vars =============
long lastCalliberationTime = 0;
#define CALLIBERATION_TIME 3000
uint8_t DEVICE_ID;

// ========= Main ==========================
void setup() {
  pinMode(shoelacePin, OUTPUT);
  pinMode(buttonPin, INPUT);


  // set initial LED state
  digitalWrite(shoelacePin, shoelaceState);
  
  randomSeed(analogRead(0));
  DEVICE_ID = random(0, 256);

  Serial.begin(115200);
  Serial.print(F("Light broadcasting using RF24. This device Id = "));
  Serial.println(DEVICE_ID);
  
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

  Serial.print(F("Now sending DATA: "));

  myData._micros = micros();
  myData.signalType = 1;

  radio.startWrite(&myData, sizeof(myData),0);

  // if (!radio.write( &myData, sizeof(myData) )){
  //  Serial.println(F("failed"));
  // }
   // Now, continue listening
    Serial.println(myData._micros);  
    evtStorage.saveEvent(myData._micros);

    radio.startListening();

  }

 void sendCalliberationSignal() {
  radio.stopListening();                                    // First, stop listening so we can talk.

  Serial.print(F("Now sending calliberation: "));

  myData.sourceDeviceId = DEVICE_ID;
  myData.value = 0;
  myData._micros = micros();
  myData.signalType = 0;

  Serial.println(myData._micros); 
  radio.startWrite(&myData, sizeof(myData),0);
  evtStorage.saveEvent(myData._micros);
  radio.startListening();
  }

unsigned long signalStrength = 0;


void loop() {

    // Send Calliberation signal
   if ((millis() - lastCalliberationTime) > CALLIBERATION_TIME) {
      // digitalWrite(shoelacePin, LOW); 
      // shoelaceState = false;
      sendCalliberationSignal();
      lastCalliberationTime = millis();
   } 
  

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
    radio.read( &myData, sizeof(myData) );             // Get the payload
    
    radio.stopListening(); 

    onEventReceived();

    if (myData.signalType){
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


    } else {
      // Serial.println("=========");
      Serial.print("Calliberation signal received from source device: ");
      Serial.println(myData.sourceDeviceId);

        if (myData.sourceDeviceId == DEVICE_ID){
           unsigned long current_time = micros();
           unsigned long time_elapsed = current_time - myData._micros;

          // this is the ping back event that now we can use to calculate TOF
          // time elapsed = micros() - myData._micros
          Serial.print("Return signal received for Device = ");
          Serial.print(myData.value);
          Serial.print(" | Elapsed Time = ");
          Serial.println(time_elapsed);

          // store it in the running means for this DeviceId
          // runningmeans[DEVICE_ID].update(time_elapsed)
          } else {
             if (myData.value==0){

              // pong the signal to the sender
              
              myData.value = DEVICE_ID;
              radio.startWrite( &myData, sizeof(myData), 0);
            }
          }
          // Serial.println("=========");
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


// Detect a wireless input event (e.g, Receive a message)
void onEventReceived() {
  // got a zb rx packet
      Serial.print(F("Got signal: Type="));
      Serial.print(myData.signalType); 
      Serial.print(F(", micros: "));
      Serial.print(myData._micros); 
      Serial.print(F(", Value = "));
      Serial.print(myData.value); 
      Serial.print(F(", Strength =  "));
      Serial.println(signalStrength);        
}

