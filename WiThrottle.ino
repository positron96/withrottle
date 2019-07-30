/*
 * Truncated JMRI WiThrottle server implementation for DCC++ command station
 * Software version 1.03b
 * Copyright (c) 2016-2018, Valerie Valley RR https://sites.google.com/site/valerievalleyrr/
 * 
 * Change log:
 * 2017-09-10 - Fixed release responce for WiThrottle.app
 *							Fixed synchronize power status on clients
 * 2017-09-24 - Added mDNS responder
 *							Added start delay to fix connection problem with DCC++
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
 *	 list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *	 list of conditions and the following disclaimer in the documentation and/or
 *	 other materials provided with the distribution.
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

#define TURNOUT_PREF "LT"
#define TURNOUT_CLOSED 2
#define TURNOUT_THROWN 4


/* Define turnout object structures */
typedef struct {
	unsigned short address;	
	unsigned int id;
	byte tStatus;
} tData;
tData turnoutData[100];

/* The interval of check connections between ESP & WiThrottle app */
const unsigned int heartbeatTimeout = 10;
bool heartbeatEnable[MAX_CLIENTS];
unsigned long heartbeat[MAX_CLIENTS*2];

String locoAddesses[MAX_CLIENTS*2] = {""};
int locoStates[MAX_CLIENTS*2][31];

String powerStatus;

boolean alreadyConnected[MAX_CLIENTS];

/* Define WiThrottle Server */
WiFiServer server(WTServer_Port);
WiFiClient client[MAX_CLIENTS];

WiFiServer dccppServer(DCCppServer_Port);
WiFiClient dccppClient;

//#define DEBUGS(v) {Serial.println(v);}
void DEBUGS(String v) {mqtt.publish("dccpp/log", v.c_str() );}

#define LOG_WIFI 0
#define LOG_DCC 0

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
	dccppServer.begin();
	MDNS.begin(hostString);
	MDNS.addService("withrottle","tcp", WTServer_Port);
	MDNS.addService("http","tcp", DCCppServer_Port);
	MDNS.setInstanceName("DCC++ Network Interface");

	while(Serial.available()>0) Serial.read();
	char powerCmd = POWER_ON_START == 1 ? '1' : '0';
	while(Serial.available()==0) { turnPower(powerCmd); delay(250); }
	processDCCppResponse(readResponse()); // this should be <p1> or <p0>
}


void loop() {
	
	mqtt.loop();
 
	if (Serial.available()>0) {
		String v = readResponse();
		if (v!="") processDCCppResponse(v);
	}

	if (!dccppClient) dccppClient = dccppServer.available();
	if (dccppClient) {
		while(dccppClient.available()>0) {
			Serial.write(dccppClient.read());
		}
	}

	
	for (int iClient=0; iClient<MAX_CLIENTS; iClient++) {
		WiFiClient& cli = client[iClient];
		if (!cli) {
			cli = server.available();
		}
		else {
			if (cli.status() == CLOSED) {
				throttleStop(iClient);
			}
			else if (!alreadyConnected[iClient]) {
				loadTurnouts();
				throttleStart(iClient);
			}
		}
		if (cli.available()) {
			//bool changeSpeed[] = { };
			while(cli.available()>0) { 
				String clientData = cli.readStringUntil('\n'); 
				if (LOG_WIFI) DEBUGS("WF>> "+clientData);
				if (clientData.startsWith("*+")) {
					heartbeatEnable[iClient] = true;
				}
				else if (clientData.startsWith("PPA")) {
					turnPower(clientData.charAt(3));
					//notifyPowerChange();
				}
				else if (clientData.startsWith("PTA")) {
					char aStatus = clientData.charAt(3);
          int aAddr;
          bool named;
          if(clientData.substring(4,6)==TURNOUT_PREF) {
            // named turnout
            aAddr = clientData.substring(6).toInt();
            named = true;
          } else {
            aAddr = clientData.substring(4).toInt();
            named = false;
          }
          accessoryToggle(aAddr, aStatus, named);
					
				}
				else if (clientData.startsWith("N") 
					  || clientData.startsWith("*")) {
					wifiPrintln(iClient, "*" + String(heartbeatTimeout));
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
						locoAction(th, actionKey, actionVal, iThrottle, iClient);
					}
					heartbeat[iThrottle] = millis();
				}
			}
			if (heartbeatEnable[iClient]) {
				checkHeartbeat(iClient);
			}
		}
	}
}

void processDCCppResponse(String resp) {
	if (resp=="p0" || resp=="p1") {
		powerStatus = resp.charAt(1);
		notifyPowerStatus();
	}
}

void notifyPowerStatus() {
	for (int p=0; p<MAX_CLIENTS; p++) {
		if (alreadyConnected[p]) {
			client[p].println("PPA"+powerStatus);
		}
	}
}


int invert(int value) {
	return value == 0 ? 1 : 0;
}

void waitForDCCpp() {
	while (Serial.available() <= 0) delay(25);
}

void sendDCCpp(String v) {
	if (LOG_DCC) DEBUGS("DCC<< "+v);
	Serial.println(v);
}

void sendDCCppCmd(String v) {
	sendDCCpp("<"+v+">");
}

void turnPower(char v) {
	sendDCCppCmd(String(v));
}

int dccReadRelayed() {
	int v = Serial.read();
	if (v>=0 && dccppClient) { dccppClient.write(v); }
	return v;
}

String readResponse() {
	char resp[maxCommandLength+1];
	char c;
	while(Serial.available() > 0) {
		c = dccReadRelayed();
		if (c == '<') {
			sprintf(resp, "");
			while( (c = dccReadRelayed()) != '>') {
				if (strlen(resp)<maxCommandLength)
					sprintf(resp, "%s%c", resp, c);
				else break;
			}
			if (LOG_DCC) DEBUGS("DCC>> "+String(resp) );
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
	while(Serial.available()>0) {
		char data[maxCommandLength];
		sprintf(data, "%s", readResponse().c_str() );
		if (strlen(data)==0) break;
		int addr, sub, stat, id;
		int ret = sscanf(data, "%*c %d %d %d %d", &id, &addr, &sub, &stat );
		turnoutData[t] = { ((addr-1)*4+1) + (sub&0x3) , id, stat==0 ? 2 : 4};
		t++;
	}
}

void wifiPrintln(int iClient, String v) {
	client[iClient].println(v);
	if (LOG_WIFI) DEBUGS("WF<< "+v);
}
void wifiPrint(int iClient, String v) {
	client[iClient].print(v);
	if (LOG_WIFI) DEBUGS("WF<< "+v);
}

void throttleStart(int iClient) {
	client[iClient].flush();
	client[iClient].setTimeout(500);
	DEBUGS("New client");

	wifiPrintln(iClient, "VN2.0");
	wifiPrintln(iClient, "RL0");
	wifiPrintln(iClient, "PPA"+powerStatus);
	wifiPrintln(iClient, "PTT]\\[Turnouts}|{Turnout]\\[Closed}|{"+String(TURNOUT_CLOSED)+"]\\[Thrown}|{"+String(TURNOUT_THROWN) );
	wifiPrint(iClient, "PTL");
	for (int t = 0 ; turnoutData[t].address != 0; t++) {
		wifiPrint(iClient, String("]\\[")+TURNOUT_PREF+turnoutData[t].address+"}|{"+turnoutData[t].id+"}|{"+turnoutData[t].tStatus);
	}
	wifiPrintln(iClient, "");
	wifiPrintln(iClient, "*"+String(heartbeatTimeout));
	alreadyConnected[iClient] = true;
}

void throttleStop(int iClient) {
	client[iClient].stop();
	DEBUGS("Client lost");
	alreadyConnected[iClient] = false;
	heartbeatEnable[iClient] = false;
	locoStates[0+iClient*2][29] = 0;	 heartbeat[0+iClient*2] = 0;
	locoStates[1+iClient*2][29] = 0;	 heartbeat[1+iClient*2] = 0;
}

void locoAdd(char th, String locoAddr, int iThrottle, int iClient) {
	locoAddesses[iThrottle] = locoAddr;
	wifiPrintln(iClient, String("M")+th+"+"+locoAddr+"<;>");
	for (int fKey=0; fKey<29; fKey++) {
		locoStates[iThrottle][fKey] =0;
		wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>F0"+String(fKey));
	}
	locoStates[iThrottle][29] =0;
	locoStates[iThrottle][30] =1;
	wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>V0");
	wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>R1");
	wifiPrintln(iClient, String("M")+th+"A"+locoAddr+"<;>s1");
	DEBUGS("loco add thr="+String(iThrottle)+"; addr"+String(locoAddr) );
}

void locoRelease(char th, String locoAddr, int iThrottle, int iClient) {
	String locoAddress = locoAddesses[iThrottle].substring(1);
	heartbeat[iThrottle] =0;
	locoAddesses[iThrottle] = "";
	wifiPrintln(iClient, String("M")+th+"-"+locoAddr+"<;>");
	DEBUGS("loco release thr="+String(iThrottle)+"; addr"+String(locoAddr) );
	// stop now
	sendDCCppCmd(String("t ")+String(iThrottle+1)+" "+locoAddress+" 0 "+String(locoStates[iThrottle][30]));
	String response = loadResponse();
}

void locoAction(char th, String locoAddr, String actionVal, int iThrottle, int i) {
	String response;
	if (locoAddr == "*") {
		locoAddr = locoAddesses[iThrottle];
	}
	String dccLocoAddr = locoAddr.substring(1);
	DEBUGS("loco action thr="+String(iThrottle)+"; action="+actionVal+"; DCC"+String(dccLocoAddr) );
	int *locoState = locoStates[iThrottle];
	if (actionVal.startsWith("F1")) {
		int fKey = actionVal.substring(2).toInt();
		locoState[fKey] = invert(locoState[fKey]);
		wifiPrintln(i, String("M")+th+"A"+locoAddr+"<;>" + "F"+String(locoState[fKey])+String(fKey));
		byte func;
		switch(fKey) {
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
	else if (actionVal.startsWith("qV")) {
		//DEBUGS("query speed for loco "+String(dccLocoAddr) );
		wifiPrintln(i, String("M")+th+"A"+locoAddr+"<;>" + "V"+String(locoState[29]));							
	}
	else if (actionVal.startsWith("V")) {
		//DEBUGS("Sending velocity to addr "+String(dccLocoAddr) );
		locoState[29] = actionVal.substring(1).toInt();
		sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+"	"+String(locoState[29])+" "+String(locoState[30]));
		response = loadResponse();
	}
	else if (actionVal.startsWith("qR")) {
		//DEBUGS("query dir for loco "+String(dccLocoAddr) );
		wifiPrintln(i, String("M")+th+"A"+locoAddr+"<;>" + "R"+String(locoState[30]));							
	}
	else if (actionVal.startsWith("R")) {
		//DEBUGS("Sending dir to addr "+String(dccLocoAddr) );
		locoState[30] = actionVal.substring(1).toInt();
		sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" "+String(locoState[29])+"	"+String(locoState[30]));
		response = loadResponse();
	}
	else if (actionVal.startsWith("X")) {
		locoState[29] = 0;
		sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" -1 "+String(locoState[30]));
		response = loadResponse();
	}
	else if (actionVal.startsWith("I")) {
		locoState[29] = 0;
		sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
		response = loadResponse();
	}
	else if (actionVal.startsWith("Q")) {
		locoState[29] = 0;
		sendDCCppCmd("t "+String(iThrottle+1)+" "+dccLocoAddr+" 0 "+String(locoState[30]));
		response = loadResponse();
	}
}

void checkHeartbeat(int iClient) {
	for (int iThrottle=0; iThrottle<2; iThrottle++) {
		int ii = iThrottle + iClient*2;
		if (heartbeat[ii] > 0 && heartbeat[ii] + heartbeatTimeout * 1000 < millis()) {
			// stop loco
			//TODO: is it sent to track?
			locoStates[ii][29] = 0;
			heartbeat[ii] = 0;
			wifiPrintln(iClient, (iThrottle==0?"MTA":"MSA")+locoAddesses[ii]+"<;>" + "V0");
		}
	}
}

void accessoryToggle(int aAddr, char aStatus, bool namedTurnout) {
	int newStat = -1;
	switch(aStatus) {
		case 'T': newStat=1; break;
		case 'C': newStat=0; break;
	}

  DEBUGS(String("turnout, addr= ")+aAddr );

  int wStat = 3; // unknown
  if(namedTurnout) {
	  int t=0;
	  for (t = 0 ; turnoutData[t].address!=0 && turnoutData[t].address!=aAddr; t++);

  	if (turnoutData[t].address!=0)  {
  		// turnout command
  		if (newStat==-1) newStat = turnoutData[t].tStatus==TURNOUT_CLOSED ? 1 : 0;
  		
  		sendDCCppCmd("T "+String(turnoutData[t].id)+" "+newStat);		
  		String response = loadResponse();
  		sscanf(response.c_str(), "%*c %*d %d", &newStat);
  		//DEBUGS(String("parsed new status "+newStat) );
  		wStat = newStat==0 ? TURNOUT_CLOSED : TURNOUT_THROWN;
      turnoutData[t].tStatus = wStat;
  	}  
  } else {
    if(newStat==-1) newStat=1;
    // accessory command    
    int addr = ((aAddr-1) >> 2) + 1; 
    int sub  = (aAddr-1) & 0x3; 
    sendDCCppCmd("a "+String(addr)+" "+sub+" "+newStat);
    wStat = newStat==0 ? TURNOUT_CLOSED : TURNOUT_THROWN;
	}

  for (int i=0; i<MAX_CLIENTS; i++) {
      if(client[i])
        wifiPrintln(i, String("PTA")+wStat+(namedTurnout?TURNOUT_PREF:"")+aAddr);
  }

}
