/*
 * Truncated JMRI WiThrottle server implementation for DCC++ command station
 * Software version 1.03b
 * Copyright (c) 2016-2018, Valerie Valley RR https://sites.google.com/site/valerievalleyrr/
 * 
 * Change log:
 * 2017-09-10 - Fixed release responce for WiThrottle.app
 *              Fixed synchronize power status on clients
 * 2017-09-24 - Added mDNS responder
 *              Added start delay to fix connection problem with DCC++
 * 2018-12-11 - Fixed Engine Driver v2.19+ throttle numbers 
 * 
 * DCC++ https://github.com/DccPlusPlus
 * ESP8266 Core https://github.com/esp8266/Arduino
 * JMRI WiFi Throttle Communications Protocol http://jmri.sourceforge.net/help/en/package/jmri/jmrit/withrottle/Protocol.shtml
 * WiThrottle official site http://www.withrottle.com/WiThrottle/Home.html
 * Download WiThrottle on the AppStore https://itunes.apple.com/us/app/withrottle-lite/id344190130
 * Engine Driver official site https://enginedriver.mstevetodd.com/
 * Download Engine Driver on the GooglePlay https://play.google.com/store/apps/details?id=jmri.enginedriver
 * 
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Create a WiFi access point. */
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <WiFiManager.h>

// for debug
#include <PubSubClient.h>
WiFiClient mqttClient;
PubSubClient mqtt("broker.hivemq.com", 1883, mqttClient);

#include "Config.h"
#define maxCommandLength 30

/* Define turnout object structures */
typedef struct {
  int address;  
  int id;
  int tStatus;
} tData;
tData turnoutData[100];

/* The interval of check connections between ESP & WiThrottle app */
const int heartbeatTimeout = 10;
bool heartbeatEnable[MAX_CLIENTS];
unsigned long heartbeat[MAX_CLIENTS*2];

String locoAddesses[] = {"", "", "", "", "", ""};
int locoStates[MAX_CLIENTS*2][31];

String powerStatus;

boolean alreadyConnected[MAX_CLIENTS];

/* Define WiThrottle Server */
WiFiServer server(WTServer_Port);
WiFiClient client[MAX_CLIENTS];

//#define DEBUGS(v) {Serial.println(v);}
void DEBUGS(String v) {mqtt.publish("dccpp/log", v.c_str() );}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Started");

  //Serial1.begin(115200);  

  WiFiManager wifiManager;
  //wifiManager.setConnectTimeout(60); // 1 min
  wifiManager.setConfigPortalTimeout(300); // 5 min
  if (!wifiManager.autoConnect(hostString) ) {
    delay(1000);
    ESP.restart();
  }
  
  mqtt.connect(hostString);

  DEBUGS("Connected");

   server.begin();
  MDNS.begin(hostString);
  MDNS.addService("withrottle","tcp", WTServer_Port);

  drain();
  char powerCmd = POWER_ON_START == 1 ? '1' : '0';
  while(Serial.available()==0) { turnPower(powerCmd); delay(25); }
  processDCCppResponse(readResponse()); // this should be <p1> or <p0>
}


void loop() {
  
  mqtt.loop();
  
  if(Serial.available()>0) {
    String v = readResponse();
    if (v!="") processDCCppResponse(v);
  }
  
  for(int iClient=0; iClient<MAX_CLIENTS; iClient++) {
    WiFiClient& cli = client[iClient];
    if (!cli) {
      cli = server.available();
    }
    else {
      if (cli.status() == CLOSED) {
        throttleStop(iClient);
      }
      else if(!alreadyConnected[iClient]) {
        loadTurnouts();
        throttleStart(iClient);
      }
    }
    if (cli.available()) {
      bool changeSpeed[] = { };
      while(cli.available()>0) { 
        String clientData = cli.readStringUntil('\n'); 
        if (clientData.startsWith("*+")) {
          heartbeatEnable[iClient] = true;
        }
        else if (clientData.startsWith("PPA")){
          turnPower(clientData.charAt(3));
          //notifyPowerChange();
        }
        else if (clientData.startsWith("PTA")){
          String aStatus = clientData.substring(3,4);
          int aAddr = clientData.substring(6).toInt();
          accessoryToggle(aAddr, aStatus);
        }
        else if (clientData.startsWith("N") 
              || clientData.startsWith("*")){
          client[iClient].println("*" + String(heartbeatTimeout));
        }
        else if (clientData.startsWith("MT") 
              || clientData.startsWith("MS") 
              || clientData.startsWith("M0") 
              || clientData.startsWith("M1")) {
          char th = clientData.charAt(1);
          int iThrottle;
          if (th == 'T' || th == '0')
            iThrottle = 0+iClient*2;
          else
            iThrottle = 1+iClient*2;
          char action = clientData.charAt(2);
          String actionData = clientData.substring(3);
          int delimiter = actionData.indexOf(";");
          String actionKey = actionData.substring(0, delimiter-1);
          String actionVal = actionData.substring(delimiter+2);
          if (action == '+') {
            locoAdd(th, actionKey, iThrottle, iClient);
          }
          else if (action == '-') {
            locoRelease(th, actionKey, iThrottle, iClient);
          }
          else if (action == 'A') {
            if(actionVal.startsWith("V")) { // velocity
              changeSpeed[iThrottle] = true;
              locoStates[iThrottle][29] = actionVal.substring(1).toInt();
            }
            else {
              DEBUGS("Action on loco: Val="+actionVal);
              locoAction(th, actionKey, actionVal, iThrottle, iClient);
            }
          }
          heartbeat[iThrottle] = millis();
        }
      }
      for(int iThrottle=0+iClient*2; iThrottle<2+iClient*2; iThrottle++){
        if(changeSpeed[iThrottle] && locoAddesses[iThrottle]!=""){
          String locoAddress = locoAddesses[iThrottle].substring(1);
          sendDCCppCmd("t "+String(iThrottle+1)+" "+ locoAddress
              +" " + String(locoStates[iThrottle][29])
              +" " + String(locoStates[iThrottle][30])  );
          String response = loadResponse();
        }
      }
      if (heartbeatEnable[iClient]) {
        checkHeartbeat(iClient);
      }
    }
  }
}

void processDCCppResponse(String resp) {
  if(resp=="p0" || resp=="p1") {
    powerStatus = resp.charAt(1);
    notifyPowerStatus();
  }
}

void notifyPowerStatus() {
  for(int p=0; p<MAX_CLIENTS; p++) {
    if (alreadyConnected[p]) {
      client[p].println("PPA"+powerStatus);
    }
  }
}


int invert(int value) {
  return value == 0 ? 1 : 0;
}

void drain() {
  String v = "";
  while(Serial.available()>0) v += (char)Serial.read();
  DEBUGS("<< "+v);
}

void waitForDCCpp() {
  while (Serial.available() <= 0) delay(25);
}

void sendDCCpp(String v) {
	Serial.println(v);
	DEBUGS(">> "+v);
}

void sendDCCppCmd(String v) {
	sendDCCpp("<"+v+">");
}

void turnPower(char v) {
  //powerStatus = v;
  sendDCCppCmd(String(v));
//  while (Serial.available() <= 0) {    
//    delay(25);
//  }
//  drain();
}

String readResponse() {
  char resp[maxCommandLength+1];
  char c;
  while(Serial.available() > 0) {
    c = Serial.read();
    if(c == '<') {
      sprintf(resp, "");
      while( (c = Serial.read()) != '>') {
        if(strlen(resp)<maxCommandLength)
          sprintf(resp, "%s%c", resp, c);
        else break;
      }
      DEBUGS("<< "+String(resp) );
      return String(resp);
    }
  }
  return "";
}


String loadResponse() {
  waitForDCCpp();
  return readResponse();
}

void loadTurnouts() {
  sendDCCppCmd("T");
  waitForDCCpp();
  int t = 0;
  while(Serial.available()>0)
  {
    char data[maxCommandLength];
    sprintf(data, "%s", readResponse().c_str() );
      String s;
      char *str = data;
      char *pch;
      pch = strtok(str, " ");
      s = (char*)pch;
      int id = s.substring(1).toInt();
      pch = strtok (NULL, " ");
      s = (char*)pch;
      int x = s.toInt();
      pch = strtok (NULL, " ");
      s = (char*)pch;
      int y = s.toInt();
      pch = strtok (NULL, " ");
      s = (char*)pch;
      int z=2;
      if(s=="1") z=4;
      int a = (x-1)*4+y+1;
      turnoutData[t]={a,id,z};
      t++;
  }
}

void throttleStart(int iClient) {
  client[iClient].flush();
  client[iClient].setTimeout(500);
  sendDCCpp("\nNew client");
  client[iClient].println("VN2.0");
  client[iClient].println("RL0");
  client[iClient].println("PPA"+powerStatus);
  client[iClient].println("PTT]\\[Turnouts}|{Turnout]\\[Closed}|{2]\\[Thrown}|{4");
  client[iClient].print("PTL");
  for (int t = 0 ; turnoutData[t].address != 0; t++) {
    client[iClient].print("]\\[LT"+String(turnoutData[t].address)+"}|{"+turnoutData[t].id+"}|{"+turnoutData[t].tStatus);
  }
  client[iClient].println("");
  client[iClient].println("*"+String(heartbeatTimeout));
  alreadyConnected[iClient] = true;
}

void throttleStop(int iClient) {
  client[iClient].stop();
  sendDCCpp("\nClient lost");
  alreadyConnected[iClient] = false;
  heartbeatEnable[iClient] = false;
  locoStates[0+iClient*2][29] = 0;   heartbeat[0+iClient*2] = 0;
  locoStates[1+iClient*2][29] = 0;   heartbeat[1+iClient*2] = 0;
}

void locoAdd(char th, String locoAddr, int iThrottle, int iClient) {
  locoAddesses[iThrottle] = locoAddr;
  client[iClient].println(String("M")+th+"+"+locoAddr+"<;>");
  for(int fKey=0; fKey<29; fKey++){
    locoStates[iThrottle][fKey] =0;
    client[iClient].println(String("M")+th+"A"+locoAddr+"<;>F0"+String(fKey));
  }
  locoStates[iThrottle][29] =0;
  locoStates[iThrottle][30] =1;
  client[iClient].println(String("M")+th+"+"+locoAddr+"<;>V0");
  client[iClient].println(String("M")+th+"+"+locoAddr+"<;>R1");
  client[iClient].println(String("M")+th+"+"+locoAddr+"<;>s1");
}

void locoRelease(char th, String locoAddr, int iThrottle, int iClient) {
  String locoAddress = locoAddesses[iThrottle].substring(1);
  heartbeat[iThrottle] =0;
  locoAddesses[iThrottle] = "";
  sendDCCppCmd(String("t ")+String(iThrottle+1)+" "+locoAddress+" 0 "+String(locoStates[iThrottle][30]));
  String response = loadResponse();
  client[iClient].println(String("M")+th+"-"+locoAddr+"<;>");
}

void locoAction(char th, String locoAddr, String actionVal, int iThrottle, int i){
  String response;
  if(locoAddr == "*") {
    locoAddr = locoAddesses[iThrottle];
  }
  String dccLocoAddr = locoAddr.substring(1);
  int *locoState = locoStates[iThrottle];
  if(actionVal.startsWith("F1")) {
    int fKey = actionVal.substring(2).toInt();
    locoState[fKey] = invert(locoState[fKey]);
    client[i].println(String("M")+th+"A"+locoAddr+"<;>" + "F"+String(locoState[fKey])+String(fKey));
    byte func;
    switch(fKey){
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
        func = 128
          + locoState[1]*1
          + locoState[2]*2
          + locoState[3]*4
          + locoState[4]*8
          + locoState[0]*16;
        sendDCCppCmd("f "+dccLocoAddr+" "+String(func));
      break;
      case 5:
      case 6:
      case 7:
      case 8:
        func = 176
          + locoState[5]*1
          + locoState[6]*2
          + locoState[7]*4
          + locoState[8]*8;
        sendDCCppCmd("f "+dccLocoAddr+" "+String(func));
      break;
      case 9:
      case 10:
      case 11:
      case 12:
        func = 160
         + locoState[9]*1
         + locoState[10]*2
         + locoState[11]*4
         + locoState[12]*8;
        sendDCCppCmd("f "+dccLocoAddr+" "+String(func));
      break;
      case 13:
      case 14:
      case 15:
      case 16:
      case 17:
      case 18:
      case 19:
      case 20:
        func = locoState[13]*1 
          + locoState[14]*2
          + locoState[15]*4
          + locoState[16]*8
          + locoState[17]*16
          + locoState[18]*32
          + locoState[19]*64
          + locoState[20]*128;
        sendDCCppCmd("f "+dccLocoAddr+" "+String(222)+" "+String(func));
      break;
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
      case 28:
        func = locoState[21]*1
          + locoState[22]*2
          + locoState[23]*4
          + locoState[24]*8
          + locoState[25]*16
          + locoState[26]*32
          + locoState[27]*64
          + locoState[28]*128;
        sendDCCppCmd("f "+dccLocoAddr+" "+String(223)+" "+String(func));
      break;
    }
  }
  else if(actionVal.startsWith("qV")){
    client[i].println(String("M")+th+"A"+locoAddr+"<;>" + "V"+String(locoState[29]));              
  }
  else if(actionVal.startsWith("V")){
    locoState[29] = actionVal.substring(1).toInt();
    sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"  "+String(locoState[29])+" "+String(locoState[30]));
    response = loadResponse();
  }
  else if(actionVal.startsWith("qR")){
    client[i].println(String("M")+th+"A"+locoAddr+"<;>" + "R"+String(locoState[30]));              
  }
  else if(actionVal.startsWith("R")){
    locoState[30] = actionVal.substring(1).toInt();
    sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" "+String(locoState[29])+"  "+String(locoState[30]));
    response = loadResponse();
  }
  else if(actionVal.startsWith("X")){
    locoState[29] = 0;
    sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" -1 "+String(locoState[30]));
    response = loadResponse();
  }
  else if(actionVal.startsWith("I")){
    locoState[29] = 0;
    sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
    response = loadResponse();
  }
  else if(actionVal.startsWith("Q")){
    locoState[29] = 0;
    sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
    response = loadResponse();
  }
}

void checkHeartbeat(int iClient) {
  for(int iThrottle=0; iThrottle<2; iThrottle++) {
    int ii = iThrottle + iClient*2;
    if(heartbeat[ii] > 0 && heartbeat[ii] + heartbeatTimeout * 1000 < millis()) {
      // stop loco
      //TODO: is it sent to track?
      locoStates[ii][29] = 0;
      heartbeat[ii] = 0;
      client[iClient].println( (iThrottle==0?"MTA":"MSA")+locoAddesses[ii]+"<;>" + "V0");
    }
  }
}

void accessoryToggle(int aAddr, String aStatus){
  int newStat;
  if(aStatus=="T") 
    newStat=1;
  else if(aStatus=="C")
    newStat=0;
  else
    newStat=-1;
  int t=0;
  for (t = 0 ; turnoutData[t].address!=0 && turnoutData[t].address!=aAddr; t++);
  if(turnoutData[t].address==0 && newStat>-1){
    int addr=((aAddr-1)/4)+1;
    int sub=aAddr-addr*4+3;
    for(int i=0; i<MAX_CLIENTS; i++){
      client[i].println("PTA2LT"+String(aAddr));
    }
    sendDCCppCmd("a "+String(addr)+" "+String(sub)+" "+String(newStat));
  }
  else {
    if(newStat==-1){
      switch(turnoutData[t].tStatus){
        case 2:  turnoutData[t].tStatus=4;  newStat=0;  break;
        case 4:  turnoutData[t].tStatus=2;  newStat=1;  break;
      }
    }
    else {
      switch(newStat){
        case 0:  turnoutData[t].tStatus=2;  break;
        case 1:  turnoutData[t].tStatus=4;  break;
      }
    }
    for(int i=0; i<MAX_CLIENTS; i++){
      client[i].println("PTA"+String(turnoutData[t].tStatus)+"LT"+String(turnoutData[t].address));
    }
    sendDCCppCmd("T "+String(turnoutData[t].id)+" "+String(newStat));
    String response = loadResponse();
  }
}
