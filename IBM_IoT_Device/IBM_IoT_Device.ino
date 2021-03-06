#include <SoftwareSerial.h>
#include <DFRobot_EC.h>
#include <EEPROM.h>
#include <OneWire.h>

#define EC_PIN A1
#define DEVICE_ID          "deviceID"
#define EC                 "EC"
#define LAT                "19.771703"
#define LON                "105.892790"
int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
float voltageEC, ecValue, temperature, salValue;
float lat, lon;
DFRobot_EC ec;
int POWERKEY = 5;
SoftwareSerial mySerial(3,4); // RX, TX
OneWire ds(DS18S20_Pin);  // on digital pin 2

char* PublishTopic = "iot-2/evt/salinity/fmt/json"; //Publish Topic
char* MQTTServer = "tcp://tl3hjn.messaging.internetofthings.ibmcloud.com:1883";
char* MQTTClientID = "d:tl3hjn:salinity:dev2";
char* MQTTUser = "use-token-auth";
char* MQTTPass = "123456789";
char* MSG_FORMAT = "{\"deviceID\":\"dev2\",\"EC\":";

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);
    PowerOn(POWERKEY);
    MQTT_Init(MQTTServer, MQTTClientID, MQTTUser, MQTTPass);
    ec.begin();
}

void loop() {
    readEC();
}

float readEC() {
    static unsigned long timepoint = millis();
    if(millis()-timepoint > 60000){  //time interval: 1min
        char msg[200];
        String st_msg = MSG_FORMAT;
        
        timepoint = millis();
        temperature = readTemperature();
		if (temperature < 50) {
            voltageEC = analogRead(EC_PIN)/1024.0*5000;
            ecValue    = ec.readEC(voltageEC,temperature) / 10 - 0.75;
            Serial.print(", EC:");
            Serial.print(ecValue, 2);
            Serial.println("ms/cm");
            salValue = (ecValue * 640) / 1000;
            
            GPS_Positioning();
            st_msg += String(ecValue);
            st_msg += ",\"Salitiny\":";
            st_msg += String(salValue);
            st_msg += ",\"Temp\":";
            st_msg += String(temperature);
			st_msg += ",\"Latitude\":";
            st_msg += String(lat);
			st_msg += ",\"Longitude\":";
            st_msg += String(lon);
            st_msg += "}";
            
            st_msg.toCharArray(msg, st_msg.length() + 1);
            Serial.print("PUB MEG: "); Serial.println(msg);
            MQTT_Pub(PublishTopic, msg);
	    }
    }
    ec.calibration(voltageEC,temperature);
}

float readTemperature() {
    //returns the temperature from one DS18S20 in DEG Celsius
    byte data[12];
    byte addr[8];
    
    if ( !ds.search(addr)) {
        //no more sensors on chain, reset search
        ds.reset_search();
        return -1000;
    }
    
    if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return -1000;
    }
    
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
        Serial.print("Device is not recognized");
        return -1000;
    }
    
    ds.reset();
    ds.select(addr);
    ds.write(0x44,1); // start conversion, with parasite power on at the end

    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    for (int i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds.read();
    }
    
    ds.reset_search();
    
    byte MSB = data[1];
    byte LSB = data[0];
    
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    float TemperatureSum = tempRead / 16;
    
	Serial.print("Temperature: "); Serial.print(TemperatureSum);
    return TemperatureSum;
}

void GPS_Positioning(){
    char response[100];
    float lat_t, log_t;
    char LatDD[3], LatMM[10], LogDD[4], LogMM[10];
    uint8_t x = 0;
    
    memset(response, '\0', 300); // Initialize the string
    memset(LatDD, '\0', 3); // Initialize the string
    memset(LatMM, '\0', 10); // Initialize the string
    memset(LogDD, '\0', 4); // Initialize the string
    memset(LogMM, '\0', 10); // Initialize the string
    
    Serial.println("Get GPS Positioning");
    
    while( mySerial.available() > 0) mySerial.read();
    mySerial.println("AT+CGPSINFO");
    delay(2000);
    
    while( mySerial.available() > 0){
        response[x] = mySerial.read();
        Serial.print((char)response[x]);
        x++;
    }
    
    if (x < 25 ) return;
    
    strncpy(LatDD, response + 13, 2);
    LatDD[2] = '\0';

    strncpy(LatMM, response + 15, 9);
    LatMM[9] = '\0';
    
    lat_t = atoi(LatDD) + (atof(LatMM) / 60);
    
    if (response[12+13] == 'N') {
        Serial.print("\r\n> Latitude: ");
        Serial.print(lat_t);
        Serial.print(" N");
    } else if (response[12+13] == 'S') {
        Serial.print("\r\n> Latitude: ");
        Serial.print(lat_t);
        Serial.print(" S");
    } else {
        return;
    }

    strncpy(LogDD, response + 14+13, 3);
    LogDD[3] = '\0';

    strncpy(LogMM, response + 17+13, 9);
    LogMM[9] = '\0';

    log_t = atoi(LogDD) + (atof(LogMM) / 60);
    
    if (response[27+13] == 'E') {
        Serial.print("\r\n> Longitude: ");
        Serial.print(log_t);
        Serial.print(" E");
    } else if (response[27+13] == 'W') {
        Serial.print("\r\n> Longitude: ");
        Serial.print(log_t);
        Serial.print(" W");
    } else {
        return;
    }
    
    lat = lat_t;
    lon = log_t;
}

void MQTT_Init(char* server, char* clientID, char* userName, char* pass) {
    char aux_str[40];
    
    Serial.println("Connecting To Server........");
    delay(2000);
    sendATcommand("ATE0", "OK", 2000);
    delay(2000);
    sendATcommand("AT+CMQTTSTART", "OK", 2000);
    delay(2000);
    Serial.print("Client ID: "); Serial.println(clientID);
    Serial.print("Server ID: "); Serial.println(server);
    
    memset(aux_str, '\0', 40);
    sprintf(aux_str, "AT+CMQTTACCQ=0,\"%s\"", clientID);
    Serial.println(aux_str);
    sendATcommand(aux_str, "OK", 2000);
    memset(aux_str, '\0', 40);
    delay(2000);
    //sprintf(aux_str, "AT+CMQTTCONNECT=0,\"%s\",90,1", server);
    sprintf(aux_str, "AT+CMQTTCONNECT=0,\"%s\",90,0,\"%s\",\"%s\"", server, userName, pass);
    Serial.println(aux_str);
    sendATcommand(aux_str, "OK", 2000);
    delay(2000);
}

void MQTT_Pub(char* topic, char* mesg) {
    char aux_str[100];
    
    Serial.print("Publishing Message: "); mySerial.println(mesg);
    sprintf(aux_str, "AT+CMQTTTOPIC=0,%d", strlen( topic ));
    Serial.println(aux_str);
    sendATcommand(aux_str, ">", 2000);
    memset(aux_str, '\0', 40);
    delay(2000);
    mySerial.println(topic);
    delay(2000);
    
    sprintf(aux_str, "AT+CMQTTPAYLOAD=0,%d", strlen( mesg ));
    Serial.println(aux_str);
    sendATcommand(aux_str, ">", 2000);
    
    delay(2000);
    mySerial.println(mesg); //Payload message
    delay(500);
    delay(2000);
    sendATcommand("AT+CMQTTPUB=0,1,60", "OK", 2000);
    delay(2000);
}

void PowerOn(int PowerKey) {
    uint8_t answer = 0;
    
    Serial.print("Starting up...\n");
    
    pinMode(PowerKey, OUTPUT);
    // power on pulse
    digitalWrite(PowerKey, HIGH);
    delay(500);
    digitalWrite(PowerKey, LOW);
    
    // waits for an answer from the module
    while (answer == 0) {     // Send AT every two seconds and wait for the answer
        answer = sendATcommand("AT", "OK", 2000);
        delay(1000);
    }
    
    delay(5000);
    
    while (sendATcommand("AT+CREG?", "+CREG: 0,1", 500) == 0)
        delay(500);
    Serial.print("Running...\n");
}

uint8_t sendATcommand(const char* ATcommand, const char* expected_answer, unsigned int timeout) {
    uint8_t x = 0, answer = 0;
    char response[100];
    unsigned long previous;

    memset(response, '\0', 100);
    delay(100);

    while( mySerial.available() > 0) mySerial.read();
    mySerial.println(ATcommand);
    Serial.print("Send command: "); Serial.println(ATcommand);
    x = 0;
    previous = millis();
    Serial.print("Respones: ");
    // this loop waits for the answer
    do{
        if(mySerial.available() != 0){    
            response[x] = mySerial.read();
            Serial.print((char)response[x]);
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL)    
                answer = 1;
        }
        // Waits for the asnwer with time out
    }while((answer == 0) && ((millis() - previous) < timeout));
    Serial.println("");
    Serial.print("Asnwer: "); Serial.println(answer);
    return answer;
}
