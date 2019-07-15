/*
 * Config.h
 * Copyright (c) 2016-2017, Valerie Valley RR https://sites.google.com/site/valerievalleyrr/
 * Part of WiThrottle for the Arduino
*/

#include <ESP8266WiFi.h>

/* Maximum WiFi clients that can be connected to WiThrottle */
#define MAX_CLIENTS 3

/* Network parameters */
const int WTServer_Port = 44444;
const int DCCppServer_Port = 2560;
const char hostString[] = "ESPWTServer";


/* Power state on start 0=OFF, 1=ON. 
   WARNING!!! If you use iOS WiThrottle.app this must set only ON*/
bool POWER_ON_START = 1;
