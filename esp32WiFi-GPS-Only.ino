#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiSTA.h>
#include <WiFiUdp.h>

//#include <SoftwareSerial.h>

#include <AsyncUDP.h>

#define LED_BUILTIN  2
//SoftwareSerial SerialNano(4,5);// RX, TX

char ssid[24] = "ARL-Tractor";
char password[24] = "channer1";

unsigned int GPSSendPort = 9999; //GPS recieving port
unsigned int GPSSendPort2 = 7999; //GPS recieving port for yield monitor
unsigned int GPSSendPortArraySize = 2;//modify this if you modify above line to match number of elements
unsigned int AOGNtripPort = 2232; //Port set in AOG Ntrip UDP

unsigned int MessageSendPort = 9990;

//io pins on first F9P
byte F9P_RX1 = 22;
byte F9P_TX1 = 23;

unsigned int espResetTime = 0;


bool newGPSString = false;// used to sort the serial data comming through
String sGPSMessage = "";//used to collect the gps message

byte ipDestination[4] = { 192, 168, 2, 255 };

//Variables
int lastMessageTime = 0;
int differenceInMessageTime = 0;

//UDP instances
AsyncUDP UdpSender; //create Udp instance for sending.
AsyncUDP UdpSender2; //create Udp instance for sending on second port.

AsyncUDP ntripUdp;
unsigned int ntripPort = 2232;

//WiFiUDP moduleUdp;

void setup() {
  //Lets delay 5 seconds to let everything else restart
  delay(2000);
  Serial.begin(38400); //output to serial terminal USB
  while(!Serial){}
  delay(500);
  //SerialNano.begin(38400);
  //delay(500);
  //bring up WiFi
  initWiFi(ssid, password);
  delay(500);
  //Serial setups
  Serial.println("Waiting before bringing up F9P...");
  delay(500);
  Serial1.begin(115200, SERIAL_8N1, F9P_RX1, F9P_TX1); //connection to 1st F9P
  while(!Serial1){}
  Serial.println("Waiting to settle down...");
  delay(500);
  //pin setups
  pinMode(LED_BUILTIN, OUTPUT);//set the blue light

  ntripUdp.listen(ntripPort); //start listening for UDP on ntrip port
  delay(50); //wait a little

  ntripUdp.onPacket([](AsyncUDPPacket packet) //when we recieve a packet
  {
      for (unsigned int i = 0; i < packet.length(); i++) //loop through the packet
            {
                Serial1.write(packet.data()[i]); //send each byte to the F9P
                //Serial.print(packet.data()[i]);
                //Serial.println();            
            }       
  }); 
}


void initWiFi(char ssid[24], char password[24])
{
    Serial.println("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        //Serial.print(".");
        // espResetTime = 0;
        //espResetTimeLimit = 5;
        espResetTime++;
        Serial.print("Reset Counter : ");
        Serial.println(espResetTime);
        delay(1000);
        if (espResetTime == 10)
        {
          //reboot the device
          Serial.println("Restarting");
          ESP.restart();
        }
    }
    digitalWrite(LED_BUILTIN,HIGH);
    Serial.println(WiFi.localIP());
}

void loop() 
{
    // put your main code here, to run repeatedly:
    //check to see if we are connected and if not try to connect again    
    if(WiFi.status() == WL_CONNECTED)
    {
        differenceInMessageTime = millis() - lastMessageTime; //only send serial every 10 seconds
        //check if suitable time has passed to stop blitzing the serial with messages.
        if (differenceInMessageTime > 20000) // every 5 seconds
        {
            IPAddress myIPAddress = WiFi.localIP();
            String myStrIPAddress = String(myIPAddress[0])+ "." + String(myIPAddress[1])+ "." + String(myIPAddress[2])+ "." + String(myIPAddress[3]);
            SendOutMessage(1, "WiFi Connected - IP Address : " + myStrIPAddress, false, true);
            if (LED_BUILTIN == HIGH) //flash the light
            {
                digitalWrite(LED_BUILTIN,LOW);
            } else {
                digitalWrite(LED_BUILTIN,HIGH);
            }
            
            lastMessageTime = millis();
        }

        //try to read from F9P number 1
        while (Serial1.available())
        {
            GetGPSMessage(Serial1.read());
        }
      
     
    }else{// try to reconnect
        digitalWrite(LED_BUILTIN,LOW);
        Serial.println("WiFi Disconnected. Reconnecting...");
        initWiFi(ssid,password);
    }
}

void SendOutMessage (int messageType, String sMessage, bool toUDP, bool toSerial)
{
  //toSerial == false;
  
  if(toSerial == true){
    Serial.println(sMessage);
  }
  if (toUDP == true){
    //send to UDP port
    char messageBuffer[sMessage.length()+1]; //byte buffer to hold the message   
    sMessage.toCharArray(messageBuffer, sMessage.length()+1); //load the message into the buffer
    
    switch(messageType){
      case 1: //information 
        UdpSender.broadcastTo(messageBuffer, MessageSendPort);
        
      break;
      
      case 2: //GPS
            
        UdpSender.broadcastTo(messageBuffer, GPSSendPort);// send GPS message out via UDP
        UdpSender2.broadcastTo(messageBuffer, GPSSendPort2);// send GPS message out via UDP for yield monitor
      break;
    }
  }  
}

//method to get the GPS NMEA messages from the serial connection
void GetGPSMessage(byte serialByte)
{
    if (serialByte == '$') //start of the message
    {
        newGPSString = true;
    }
    if (serialByte == 10) //end of the message
    {
        newGPSString = false;
    } 
    if (newGPSString == true)
    {
        sGPSMessage.concat((char)serialByte);//add the char to the message
    } else 
    {
        sGPSMessage.concat((char)serialByte); //add the carriage return
        //check the message starts with a $ and is at least 6 characters long
        if ((sGPSMessage.charAt(0) == '$') && (serialByte == 10))
        {
            //lets do some filtering
            //Serial.print(sGPSMessage);
            ProcessedMessage(sGPSMessage);
            //SendOutMessage(2,ProcessedMessage(sGPSMessage),true,false);//send the message  x,y,UDP,SERIAL
        }
        sGPSMessage = "";//clear the message
    }
}

String ProcessedMessage(String sGPSMessage)
{
  //only get GGA and VTG messages
  String messageHead = sGPSMessage.substring(1,6);
  //Serial.println (messageHead);
  if((messageHead == "GNGGA") || (messageHead == "GNVTG"))
  {
      //Serial.println (messageHead);
      int stringLen = sGPSMessage.length();
      int starLocation = 0;
      char nmeaString[stringLen];
      long checkSum = 0;
      sGPSMessage.toCharArray(nmeaString,stringLen);
      //= sGPSMessage;
      //Serial.print("nmeaString Length : ");
      //Serial.println(stringLen);
      
      for(int i = 0; i <= stringLen; i++){ //
        //Serial.print("I : ");
        //Serial.println(i);
        //Serial.println(nmeaString[i]);
        if (nmeaString[i] == '*')
        {
          starLocation = i;
          i = stringLen + 1;
        }
        if((nmeaString[i] != '$') && (i <= stringLen))
        {
          //lets checksum
          if (checkSum == 0)
          {
            checkSum = nmeaString[i];
        
          } else{
            checkSum = checkSum ^ nmeaString[i];
          }
        }       
      }

      //pull the check sum from the string
      String stringChecksum = sGPSMessage.substring(starLocation+1,sGPSMessage.length()-2);
      int receivedChecksumLength = stringChecksum.length();
      char receivedChecksumChars[receivedChecksumLength];
      stringChecksum.toCharArray(receivedChecksumChars,receivedChecksumLength+1);
      long receivedChecksumLong = strtol(receivedChecksumChars, NULL, 16);
      
      if(receivedChecksumLong == checkSum){
        SendOutMessage(2,sGPSMessage,true,false);//send the message  x,y,UDP,SERIAL
      }
      
  }
  return sGPSMessage;
}
