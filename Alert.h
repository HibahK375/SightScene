#pragma once
#include <Arduino.h>
#include "config.h"
#include "Fusion.h"   // EventLevel

void alertInit();                    // once in setup() (short self-test)
void alertOnEvent(EventLevel level); // right after every fusionUpdate()
void alertUpdate();                  // top of every loop()