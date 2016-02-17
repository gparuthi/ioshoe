#pragma once 

#include "Namespace.h"
// #include <Wire.h>
#include <Arduino.h>

#define MAX_EVENTS 5 //maximum number of events to check if received. 

IOSHOE_NAMESPACE_ENTER

class EventStorage {
    private:
      long events[MAX_EVENTS];
      int nextEventIndex = 0;

    public:
      void init();
      bool checkEventExist(long event);
      long createAndSaveEvent();
      void saveEvent(long event);
};

IOSHOE_NAMESPACE_EXIT