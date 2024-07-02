#include <WiFi.h>//ESP32 wifi library
#include <PubSubClient.h>//library used for mqtt
#include <TinyGPSPlus.h> //GPS library to extract gps data
#include <SPI.h> //spi connection to rfid module
#include <MFRC522.h>//rfid library
#include <string.h>

TinyGPSPlus gps;
//Hardware Serial pins/
#define RXD2 16
#define TXD2 17
//spi pins rfid-esp32/
#define SS_PIN  5  // ESP32 pin GPIO5 
#define RST_PIN 15 // ESP32 pin GPIO27

unsigned int restart=0;
MFRC522 rfid(SS_PIN, RST_PIN);
 String CardID ="";
   byte nuidPICC[4];

//wifi connection credentials
const char* ssid = "gpstracker";
const char* password = "Wemade@iQube";
//mqtt connection credentials
const char* mqtt_server = "broker.hivemq.com";
// const int* mqtt_port = "1883";
// const char* username = "";
// const char* password = "";

WiFiClient espClient; //create an object for WiFiclient
PubSubClient client(espClient);
unsigned long lastMsg = 0; //create a last msg variable
#define MSG_BUFFER_SIZE 350
char msg[MSG_BUFFER_SIZE]={0};
char data[MSG_BUFFER_SIZE]={0};
int value = 0;

void setup_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println("\n");
  // delay(1000);
  int i = 0;
  while(WiFi.status()!=WL_CONNECTED){
    delay(i);
    if(i>350){
      // i=0;
      ESP.restart();
    }
    
    i++;
    // Serial.print(i);
  }
  Serial.print(i);
  Serial.println(WiFi.localIP());
  Serial.println("\nConnection Established");
}

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived");
}

void reconnect()
{
  //Loop until we're connected
  while(!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    //Create a random client ID
    String clientID = "ESP266client-";
    clientID += String(random(0xffff),HEX);
    //Attempt to connect
    if(client.connect(clientID.c_str()))
    {
      Serial.println("Connected");
      digitalWrite(12,HIGH);
      digitalWrite(27,HIGH);
      digitalWrite(33,LOW);
      //Once connected, publish an announcement...
      // client.publish("kctgps","hello");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 3 seconds");
      delay(3000);
    }
  }
}

void getgps()
{
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      // displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  if(!gps.location.isValid())
  {
   restart++;
   Serial.println(restart);
   if(restart>75000)
      ESP.restart();
  }
  // Serial.println(gps.location.isValid());
  if(gps.location.isValid())
  {
    digitalWrite(12,HIGH);
    digitalWrite(27,LOW);
    digitalWrite(33,HIGH);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  // if (gps.location.isValid()){
      Serial.print("Lat: ");
       Serial.print(gps.location.lat(), 6);
 Serial.print(F(","));
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
    Serial.println();
 
}

String rfidread()
{
  String lastval;
  if (rfid.PICC_IsNewCardPresent()) 
  { // new tag is available
    if (rfid.PICC_ReadCardSerial()) 

    { // NUID has been readed
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      Serial.print("RFID/NFC Tag Type: ");
      Serial.println(rfid.PICC_GetTypeName(piccType));

      // print UID in Serial Monitor in the hex format
      Serial.print("UID:");
      for (int i = 0; i < rfid.uid.size; i++)//rfid.uid.size
      {
        //Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
        //Serial.print(rfid.uid.uidByte[i], HEX);
        CardID += rfid.uid.uidByte[i];
        // Serial.print(CardID);
      }
      Serial.println();
      
      rfid.PICC_HaltA(); // halt PICC
      rfid.PCD_StopCrypto1(); // stop encryption on PCD
      // rfid.uid.size = 0;
    }

    if (rfid.uid.uidByte[0] != nuidPICC[0] || 
rfid.uid.uidByte[1] != nuidPICC[1] || 
rfid.uid.uidByte[2] != nuidPICC[2] || 
rfid.uid.uidByte[3] != nuidPICC[3] ) {
Serial.println(F("A new card has been detected."));
      digitalWrite(12,LOW);
      digitalWrite(27,HIGH);
      digitalWrite(33,HIGH);
      delay(1000);
      digitalWrite(12,HIGH);
      digitalWrite(27,LOW);
      digitalWrite(33,LOW);
// delay(1500);
// Store NUID into nuidPICC array
for (byte i = 0; i < 4; i++) {
  nuidPICC[i] = rfid.uid.uidByte[i];
}
if(lastval != CardID)
    {
    lastval = CardID;
    CardID = "";
    digitalWrite(12,LOW);
    digitalWrite(27,HIGH);
    digitalWrite(33,HIGH);
    delay(1000);
    digitalWrite(12,HIGH);
    digitalWrite(27,LOW);
    digitalWrite(33,LOW);
    
    return lastval;
    }
}
else
     {
      Serial.println(F("Card read previously."));
      digitalWrite(12,LOW);
      digitalWrite(27,HIGH);
      digitalWrite(33,HIGH);
      delay(300);
      digitalWrite(12,HIGH);
      digitalWrite(27,LOW);
      digitalWrite(33,LOW);
      return "null";
     }  
  }
  digitalWrite(12,HIGH);
  digitalWrite(27,LOW);
  digitalWrite(33,LOW);
  return "null";
  // CardID = "0000000000";
  // digitalWrite(2,LOW);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // pinMode(2,OUTPUT); //rfid indicator(BLUE)
  pinMode(35,OUTPUT);//Buzzer
  pinMode(12,OUTPUT);//Mqtt Connect(RED)
  pinMode(27,OUTPUT);//GPS Connect(GREEN)
  pinMode(33,OUTPUT);//RFID Read Connect(BLUE)
  pinMode(35,LOW);//Initally turn off the buzzer
  digitalWrite(12,HIGH);
  digitalWrite(27,HIGH);
  digitalWrite(33,HIGH);//GPS Connect(RED)
  Serial.print("Initializing all the setup:\n");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  Serial.println("\n");
  SPI.begin(); // init SPI bus
  rfid.PCD_Init(); // init MFRC522
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);//Serial comm b/w gps and esp32
}

void loop() 
{  
  if(!client.connected())
  {
    reconnect();
  }
  
  client.loop();
  getgps();
  String rfid="null";
  String latitude = String(gps.location.lat(),7);
  String longitude = String(gps.location.lng(),7);
  unsigned long now = millis();

  if(now - lastMsg > 2000)
  {
    lastMsg = now;
    ++value;
    snprintf(msg,MSG_BUFFER_SIZE,"[lat:%s,long:%s]",latitude,longitude);//gps.location.lat(),gps.location.lng());
    // snprintf(buffer, 6, "%s\n", s);
    Serial.print("Publish message:");
    Serial.println(msg);
    client.publish("kctgps",msg);
    String previous;
    rfid = rfidread();
    previous = rfid;
    Serial.print("RFID value:");
    Serial.println(rfid);
    
    if(rfid!="null")//&& rfid!=previous
    {
      snprintf(data,MSG_BUFFER_SIZE,"[lat:%s,long:%s,UserID:%s]",latitude,longitude,rfid);
      Serial.print("Publish user message:");
      Serial.println(data);
      client.publish("user",data);
      }
      }

}