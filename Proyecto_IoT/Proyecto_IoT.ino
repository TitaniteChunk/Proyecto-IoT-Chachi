//LIBRERIAS QUE SE VAN A UTILIZAR Y ACTIVACIÓN DE ADC_MODE
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 
#include <ArduinoJson.hpp> 
#include "DHTesp.h"
WiFiClient wClient;
PubSubClient mqtt_client(wClient);
DHTesp dht;
ADC_MODE(ADC_VCC)

// VARIABLES GLOBALES PARA CONFIGURAR ACTUALIZACIÓN
#define HTTP_OTA_ADDRESS      F("172.16.53.112")          // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update")    // Path to update firmware
#define HTTP_OTA_PORT         1880                        // Port of update server
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu" 



// VARIABLES GLOBALES PARA CONFIGURAR WIFI

const char* ssid = "infind";
const char* password = "1518wifi";
const char* mqtt_server = "iot.ac.uma.es";
const char* mqtt_user = "II1";
const char* mqtt_pass = "7o56oYsu";

// VARIABLES GLOBALES PARA CONFIGURAR MQTT

char topic_P_conexion[256];
char topic_P_datos[256];
char topic_P_ledstatus[256];
char topic_P_switchstatus[256];
char topic_P_error[256];
char topic_S_config[256];
char topic_S_ledcmd[256];
char topic_S_switchcmd[256];
char topic_S_FOTA[256];

char GRUPO[16] = "1";
#define TAMANHO_MENSAJE 128
unsigned long ultimo_mensaje = 0;
unsigned long ultima_actualizacion = 0;
unsigned long ultimo_cambioled = 0;
unsigned long ultima_recepcion = 0;

// VARIABLES GLOBALES PARA ALMACENAR VARIABLES MQTT

char CHIP_ID[16];
bool online = true;


char ERRORES[32] = "nada";
unsigned long inicializacion;
unsigned long Uptime;
int Vcc;
float temp;
float hum;
int LED;
int SWITCH;
char SSId[16];
char IP[16];
int RSSi;
int envia = 10;
int actualiza = 0;
char actualizar[16] = "false";
int velocidad = 50;
char origen[16] = "MQTT";
int level_led;
int objetivo_led = 0;
int estado_led = 0;
int level_switch;
char ID[16];

// VARIABLES GLOBALES PARA CONTROLAR LEDS Y BOTÓN
int LED1 = 2;  
int LED2 = 16;

int boton_flash=0;                                  // GPIO0 = boton flash
int estado_polling=HIGH;                            // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
int estado_int=HIGH;                                // por defecto HIGH (PULLUP). Cuando se pulsa se pone a LOW
int pulsacion = 1;
int pulso1 = 0;
int pulso2;

unsigned long inicio_pulso_polling = 0;
unsigned long ultima_int = 0;
volatile int lectura;
int lectura2;
volatile unsigned long ahora;

// DECLARACIÓN DE FUNCIONES PARA OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);



//-----------------------------------------------------
void intenta_OTA()
{ 
  Serial.println( "--------------------------" );  
  Serial.println( "Comprobando actualización:" );
  Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH);
  Serial.println( "--------------------------" );  
  ESPhttpUpdate.setLedPin(LED2, LOW);
  ESPhttpUpdate.onStart(inicio_OTA);
  ESPhttpUpdate.onError(error_OTA);
  ESPhttpUpdate.onProgress(progreso_OTA);
  ESPhttpUpdate.onEnd(final_OTA);
  WiFiClient wClient;
  switch(ESPhttpUpdate.update(wClient, HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F(" El dispositivo ya está actualizado"));
      break;
    case HTTP_UPDATE_OK:
      Serial.println(F(" OK"));
      break;
    }
pulsacion = 1;
}

//·····················································

void final_OTA()
{
  Serial.println("Fin OTA. Reiniciando...");
}

void inicio_OTA()
{
  Serial.println("Nuevo Firmware encontrado. Actualizando...");
}

void error_OTA(int e)
{
  char cadena[64];
  snprintf(cadena,64,"ERROR: %d",e);
  Serial.println(cadena);
}

void progreso_OTA(int x, int todo)
{
  char cadena[256];
  int progress=(int)((x*100)/todo);
  if(progress%10==0)
  {
    snprintf(cadena,256,"Progreso: %d%% - %dK de %dK",progress,x/1024,todo/1024);
    Serial.println(cadena);
  }
}

//-----------------------------------------------------

void conecta_wifi() {
  Serial.printf("\nConnecting to %s:\n", ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
}

//-----------------------------------------------------

void conecta_mqtt() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //(CHIPID, mqtt_user, mqtt_pass)) 
    if (mqtt_client.connect(CHIP_ID, mqtt_user, mqtt_pass, topic_P_conexion, '1', true, "{\"online\":false}", true)) {
      Serial.printf(" conectado a broker: %s\n",mqtt_server);
      mqtt_client.subscribe(topic_S_config);
      mqtt_client.subscribe(topic_S_ledcmd);
      mqtt_client.subscribe(topic_S_switchcmd);
      mqtt_client.subscribe(topic_S_FOTA);
      
    } else {
      Serial.printf("failed, rc=%d  try again in 5s\n", mqtt_client.state());
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//-----------------------------------------------------

void procesa_mensaje(char* topic, byte* payload, unsigned int length) {
  char *mensaje = (char *)malloc(length+1);                                 // Reservar memoria para copia del mensaje
  strncpy(mensaje, (char*)payload, length);                                 // Copiar el mensaje en cadena de caracteres
  mensaje[length]='\0';                                                     // Caracter cero marca el final de la cadena

  Serial.print("Mensaje recibido\n" );
  Serial.println(mensaje);
  Serial.println(topic);
  StaticJsonDocument<512> root;
  DeserializationError error = deserializeJson(root, mensaje,length);     // Deserializar Json y generar error en su caso

  if (error) {                                                            // Compruebo si hubo error
    Serial.print("Error deserializeJson() failed: ");
    Serial.println(error.c_str());
  }

//·-·-·-·-·-·-·-·-·-·SUBSCRIPCIONES··-·-·-·-·-·-·-·-·-·

  if(strcmp(topic,topic_S_config)==0)
  {
    if(root.containsKey("envia"))                                            // Comprobar si existe el campo/clave "envia"
    { 
      if(strcmp(root["envia"],"null")==0){}                                  // Comprobar que el contenido del mensaje no sea "null"
      else {envia = root["envia"];} 
    }
   
    if(root.containsKey("actualiza"))                                        // Comprobar si existe el campo/clave "actualiza"
    { 
      if(strcmp(root["actualiza"],"null")==0){}                              // Comprobar que el contenido del mensaje no sea "null"
      else 
      {
        actualiza = root["actualiza"];
        if (actualiza != 0)
        {
        sprintf(actualizar,"true");
        }
      } 
    }
 
    if(root.containsKey("velocidad"))                                        // Comprobar si existe el campo/clave "velocidad"
    { 
      if(strcmp(root["velocidad"],"null")==0){}                              // Comprobar que el contenido del mensaje no sea "null"
      else {velocidad = root["velocidad"];} 
    }

    if(root.containsKey("LED"))                                              // Comprobar si existe el campo/clave "LED"
    { 
      if(strcmp(root["LED"],"null")==0){}                                    // Comprobar que el contenido del mensaje no sea "null"
      else {LED = root["LED"];} 
    }

    if(root.containsKey("SWITCH"))                                           // Comprobar si existe el campo/clave "SWITCH"
    { 
      if(strcmp(root["SWITCH"],"null")==0){}                                 // Comprobar que el contenido del mensaje no sea "null"
      else {SWITCH = root["SWITCH"];} 
    }
  }
  
//·····················································
  
  else if(strcmp(topic,topic_S_ledcmd)==0)
  {
    
    if(root.containsKey("id"))                                            
    { 
      sprintf(ID, root["id"]); 
    }     
    
    if(root.containsKey("level"))
    {
      if (pulsacion == 1){objetivo_led = root["level"];}
      level_led = root["level"];         
    }
  }
  
//·····················································

  else if(strcmp(topic,topic_S_switchcmd)==0)
  {
    if(root.containsKey("id"))                                           
    { 
      sprintf(ID, root["id"]); 
    }     
    if(root.containsKey("level"))
    {
      level_switch = root["level"];         
    }
  }

//·····················································

  else if(strcmp(topic,topic_S_FOTA)==0)
  {
    if(root.containsKey("actualiza"))                                            // Comprobar si existe el campo/clave "envia"
    { 
      sprintf(actualizar,root["actualiza"]); 
    }     
  }

//·····················································
  
  else
  {
    Serial.println("Error: Topic desconocido");
  }

free(mensaje);                                                              // Liberar la memoria del mensaje

}

//-----------------------------------------------------

void SerializeComplex(char* topic, StaticJsonDocument<300> doc)             // Serializar Json con datos de los sensores
{
    char json[256];
  
  serializeJson(doc,json);
  Serial.println(json);

  mqtt_client.publish(topic,json);
}

//-----------------------------------------------------

void setup() {
  
  pinMode(boton_flash, INPUT_PULLUP);                                       // Parte dedicada al botón
  attachInterrupt(digitalPinToInterrupt(boton_flash), RTI, CHANGE);

  inicializacion = millis();
  
  char cadena[512];
  Serial.begin(115200);
  Serial.println();
  Serial.println("Empieza setup...");
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  pinMode(LED1, OUTPUT);                                                    // inicializa GPIO como salida
  digitalWrite(LED1, HIGH);                                                 // apaga el led
  dht.setup(5, DHTesp::DHT11);                                              // Inicializar conexiones de los sensores

//·····················································  
// CREACIÓN DE TOPICS

  sprintf(CHIP_ID, "ESP_%d", ESP.getChipId());

  sprintf(topic_P_conexion, "II%s/ESP_%d/conexion", GRUPO, ESP.getChipId());
  sprintf(topic_P_datos, "II%s/ESP_%d/datos", GRUPO, ESP.getChipId());
  sprintf(topic_P_ledstatus, "II%s/ESP_%d/led/status", GRUPO, ESP.getChipId());
  sprintf(topic_P_switchstatus, "II%s/ESP_%d/switch/status", GRUPO, ESP.getChipId());
  sprintf(topic_P_error, "II%s/ESP_%d/error", GRUPO, ESP.getChipId());
  
  sprintf(topic_S_config, "II%s/ESP_%d/config", GRUPO, ESP.getChipId());
  sprintf(topic_S_ledcmd, "II%s/ESP_%d/led/cmd", GRUPO, ESP.getChipId());
  sprintf(topic_S_switchcmd, "II%s/ESP_%d/swich/cmd", GRUPO, ESP.getChipId());   
  sprintf(topic_S_FOTA, "II%s/ESP_%d/FOTA", GRUPO, ESP.getChipId()); 
 
//·····················································
  
  bool wifi = WiFi.status();
  conecta_wifi();                                                           // Conectar al Wifi
  intenta_OTA();                                                            // Comprobar actualizaciones
  mqtt_client.setServer(mqtt_server, 1883);                                 // Definir credenciales del servidor MQTT
  mqtt_client.setBufferSize(512);                                           // Ajustar tamaño de Buffer
  mqtt_client.setCallback(procesa_mensaje);                                 // Definir función de Callback
  conecta_mqtt();                                                           // Conectarse al servidor MQTT
  Serial.printf("Identificador placa: %s\n", CHIP_ID );
  Serial.printf("Topic publicacion  : %s\n", topic_P_conexion);
  Serial.printf("Topic publicacion  : %s\n", topic_P_datos);
  Serial.printf("Topic publicacion  : %s\n", topic_P_ledstatus);
  Serial.printf("Topic publicacion  : %s\n", topic_P_switchstatus);
  
  Serial.printf("Topic subscripcion : %s\n", topic_S_config);
  Serial.printf("Topic subscripcion : %s\n", topic_S_ledcmd);
  Serial.printf("Topic subscripcion : %s\n", topic_S_switchcmd);
  Serial.printf("Topic subscripcion : %s\n", topic_S_FOTA);
  
  Serial.printf("Termina setup en %lu ms\n\n",millis());

//·····················································

StaticJsonDocument<300> conexion;

conexion["Online"]=wifi;                                                    // Comprueba si se ha conectado al Wifi

SerializeComplex(topic_P_conexion,conexion);                                // Serializar Json con información sobre la conexión

}

//-----------------------------------------------------

ICACHE_RAM_ATTR void RTI() {                                                // Manejo de la interrupción del botón
  lectura=digitalRead(boton_flash);
  ahora = millis();
  // descomentar para eliminar rebotes
  if(lectura==estado_int || ahora-ultima_int<50) return;                    // filtro rebotes 50ms
  if(lectura==LOW)
  { 
   estado_int=LOW;
   pulso2 = pulso1;
   pulso1 = ahora;
  }
  else
  {
   estado_int=HIGH;
  }

ultima_int = ahora;

}

//-----------------------------------------------------

void loop() {

  if(lectura!=lectura2){
    lectura2 = lectura;
      if(lectura==LOW)
        { 
          
        }
      else
        {
        Serial.print("Int dura: ");
        Serial.println(ultima_int-pulso1);
        if((ultima_int-pulso1)<1000){pulsacion = 1;};                      // Se ha pulsado el botón
        if((pulso1-pulso2)<400){pulsacion = 2;};                           // La pulsación es doble
        if((ultima_int-pulso1)>1000){pulsacion = 3;};                      // La pulsación es prolongada
        Serial.print(pulsacion);
        }
  }

  if(pulsacion == 1)
    {
      objetivo_led = level_led;
      //analogWrite(LED1,(level_led*(-255.0)-100.0*-255.0)*(1/100.0));        // Si la pulsación es simple poner el Led1 al valor que le llega por MQTT
      sprintf(origen, "MQTT");
    }
    else if (pulsacion == 2)
    {
      analogWrite(LED1,0);                                                  // Si la pulsación es doble poner el Led1 al máximo (0)
      objetivo_led = 100;
      sprintf(origen, "Pulsador");
    }                         
    else if (pulsacion == 3){intenta_OTA();}                                // Si la pulsación es prolongada comprobar actualización
    
  if (!mqtt_client.connected()) conecta_mqtt();                             // Comprobar conexión al servidor MQTT, y realizarla en caso negativo
  mqtt_client.loop();                                                       // La librería MQTT recupera el control
  unsigned long ahora_mensaje = millis();                                   // Temporización para el envío de mensajes
  
  if (ahora_mensaje - ultimo_cambioled >= velocidad)
  {
    ultimo_cambioled = ahora_mensaje;
    if (estado_led < objetivo_led)
    {
      estado_led = estado_led + 1;
      analogWrite(LED1,(estado_led*(-255.0)-100.0*-255.0)*(1/100.0));   
    }
    else if (estado_led > objetivo_led)
    {
      estado_led = estado_led - 1;
      analogWrite(LED1,(estado_led*(-255.0)-100.0*-255.0)*(1/100.0));   
    }
  }
  
  
  if (ahora_mensaje - ultimo_mensaje >= envia*1000) 
  {
    delay(dht.getMinimumSamplingPeriod());                                  // Delay para no sobrecargar los sensores

    hum = dht.getHumidity();                                      // Lectura de la humedad
    if (hum<0||hum>100){printf(ERRORES, "El sensor de humedad está fallando");}
    temp = dht.getTemperature();                               // Lectura de la temperatura
    if (temp<10||temp>70){printf(ERRORES, "El sensor de temperatura está fallando");}
    Vcc = ESP.getVcc()/1000;                                      // Lectura del nivel del voltaje en milivoltios
    if (Vcc<10||Vcc>70){printf(ERRORES, "El sensor de temperatura está fallando");}
    bool wifi = WiFi.status();                                              // Comprueba conexión
    RSSi = WiFi.RSSI();                                               // Comprueba RSSI
    sprintf(IP, WiFi.localIP().toString().c_str());                         // Guarda IP
    unsigned long tiempo = millis();                                                  // Guarda momento de la lecura
    Uptime = tiempo - inicializacion;
    ultimo_mensaje = ahora_mensaje;                                                 // Actualizar instancia del último mensaje
    
//·····················································

    StaticJsonDocument<64> json_conexion;
    json_conexion["CHIPID"] = CHIP_ID;
    json_conexion["online"] = online;
    SerializeComplex(topic_P_conexion,json_conexion);
    
    StaticJsonDocument<256> json_datos;
    json_datos["CHIPID"] = CHIP_ID;
    json_datos["Uptime"] = Uptime;
    json_datos["Vcc"]  = Vcc;
    JsonObject DHT11 = json_datos.createNestedObject("DHT11");
      DHT11["temp"] = temp;
      DHT11["hum"] = hum;
      json_datos["LED"] = level_led;
      json_datos["SWITCH"] = level_switch;
    JsonObject Wifi = json_datos.createNestedObject("Wifi");
      Wifi["SSId"] = "infind";
      Wifi["IP"] = IP;
      Wifi["RSSI"] = RSSi;
    SerializeComplex(topic_P_datos,json_datos);

    StaticJsonDocument<96> json_led_status;
    json_led_status["CHIPID"] = CHIP_ID;
    json_led_status["LED"] = level_led;
    json_led_status["origen"] = origen;
    SerializeComplex(topic_P_ledstatus,json_led_status);

    StaticJsonDocument<96> json_switch_status;
    json_switch_status["CHIPID"] = CHIP_ID;
    json_switch_status["SWITCH"] = level_switch;
    json_switch_status["origen"] = origen;
    SerializeComplex(topic_P_switchstatus,json_switch_status);

    if (strcmp(ERRORES,"nada")!=0)
    {
    StaticJsonDocument<96> json_error;
    json_switch_status["ERROR"] = ERRORES;
    SerializeComplex(topic_P_switchstatus,json_error);
    }
  }
  if (strcmp(actualizar,"true"))
  {
    if (actualiza != 0)
    {
      if (ahora_mensaje - ultima_actualizacion >= actualiza*1000)
      {
        ultima_actualizacion = ahora_mensaje;
        intenta_OTA();
      }
    }
  }   
}
