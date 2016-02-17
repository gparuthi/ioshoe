#include "EventStorage.h"



IOSHOE_NAMESPACE_USING

void EventStorage::init() {
}

bool EventStorage::checkEventExist(long event) {
  for(int i = 0; i < MAX_EVENTS; i++){
    if (events[i] == event){
      return true; // find the key 
    }
  } 

  return false; 
}

long EventStorage::createAndSaveEvent() {
  long event_id = random(255);
  
  events[nextEventIndex] = event_id;
  nextEventIndex++;
  if(nextEventIndex >= MAX_EVENTS)
    nextEventIndex = 0;

  return event_id;
}


void EventStorage::saveEvent(long event) {
  
  events[nextEventIndex] = event;
  nextEventIndex++;
  if(nextEventIndex >= MAX_EVENTS)
    nextEventIndex = 0;

}