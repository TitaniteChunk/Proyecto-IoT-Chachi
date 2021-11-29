/*
 Basic ESP8266 MQTT example
 
 Principales diferencias con el ejemplo de la librería PubSubClient:
   Utiliza el ID del chip en setup() para:
     - construir un identificador único para conectar con MQTT (ESP_???????)
     - construir topics únicos para esta placa (infind/ESP_???????/+)
   Conecta a MQTT usando usuario/contraseña (conecta_mqtt())
   Configura el servidor MQTT para admitir mensajes hasta 512 bytes (setup())
   En callback():
     - copia el mensaje a una cadena de caracteres (gestión de memoria con malloc/free)
     - comprueba el valor del topic
   Al enviar un mensaje enciende el LED correspondiente al GPIO16
*/

#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 
#include <ArduinoJson.hpp> 
#include "DHTesp.h"

ADC_MODE(ADC_VCC)

// datos para actualización   >>>> SUSTITUIR IP <<<<<
#define HTTP_OTA_ADDRESS      F("172.16.53.111")         // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Path to update firmware
#define HTTP_OTA_PORT         1880                     // Port of update server
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu" 

// funciones para progreso de OTA
void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);

// INCLUIMOS LAS LIBRERIAS QUE SE VAN A UTILIZAR
WiFiClient wClient;
PubSubClient mqtt_client(wClient);
DHTesp dht;
// Update these with values suitable for your network.
const char* ssid = "infind";
const char* password = "1518wifi";
const char* mqtt_server = "172.16.53.111";
const char* mqtt_user = "infind";
const char* mqtt_pass = "zancudo";

// cadenas para topics e ID
char ID_PLACA[16];
char topic_PUB[256];
char topic_PUB1[256];
char topic_PUB2[256];
char topic_PUB3[256];
char topic_PUB4[256];
char topic_SUB[256];
char GRUPO[16] = "GRUPO1";

// GPIOs
int LED1 = 2;  
int LED2 = 16;
int valor;
int LED_blink= 2;  
int LED_OTA = 16; 

//-----------------------------------------------------
void intenta_OTA()
{ 
  Serial.println( "------------V2------------" );  
  Serial.println( "Comprobando actualización:" );
  Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH);
  Serial.println( "--------------------------" );  
  ESPhttpUpdate.setLedPin(LED_OTA, LOW);
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
}

//-----------------------------------------------------
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
    //(ID_PLACA, mqtt_user, mqtt_pass)) 
    if (mqtt_client.connect(ID_PLACA, mqtt_user, mqtt_pass, topic_PUB3, '1', true, "{\"online\":false}", true)) {
      Serial.printf(" conectado a broker: %s\n",mqtt_server);
      mqtt_client.subscribe(topic_SUB);
    } else {
      Serial.printf("failed, rc=%d  try again in 5s\n", mqtt_client.state());
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}




//-----------------------------------------------------


//-----------------------------------------------------
void procesa_mensaje(char* topic, byte* payload, unsigned int length) {
  char *mensaje = (char *)malloc(length+1); // reservo memoria para copia del mensaje
  strncpy(mensaje, (char*)payload, length); // copio el mensaje en cadena de caracteres
  mensaje[length]='\0'; // caracter cero marca el final de la cadena
  
  Serial.printf("Mensaje de LED recibido [%s] %s\n", topic, mensaje);

  // compruebo el topic
  if(strcmp(topic,"infind/GRUPO1/led/cmd")==0) 
  {
    StaticJsonDocument<512> root;
    DeserializationError error = deserializeJson(root, mensaje,length);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    
    else
    if(root.containsKey("level"))  // comprobar si existe el campo/clave que estamos buscando
    { 
      
     int valor = root["level"];
     Serial.print("Mensaje OK \n");
     
//    led=String(mensaje).toInt();

     analogWrite(LED1,(valor*(-255.0)-100.0*-255.0)*(1/100.0));    // turn the LED off by making the voltage LOW
    
     Serial.print("Mensaje OK, level del 0 al 100 = ");
     Serial.println(valor);
     StaticJsonDocument<300> estado_led;
     estado_led["Led"] = valor;
     SerializeComplex(topic_PUB1,estado_led);

     
    }
    else
    {
      Serial.print("Error : ");
      Serial.println("\"level\" key not found in JSON");
    }
  } // if topic
  else
  {
    Serial.println("Error: Topic desconocido");
  }
 
 free(mensaje);
}


//-------------------------------------------------------------------------------------

//.........................SERIALIZACIÓN...DE...MENSAJES..............................

//-------------------------------------------------------------------------------------

void SerializeComplex(char* topic, StaticJsonDocument<300> doc)
{
    char json[256];
  
  serializeJson(doc,json);
  Serial.println(json);

  mqtt_client.publish(topic,json);
  }



//-----------------------------------------------------
//     SETUP
//-----------------------------------------------------
void setup() {
  char cadena[512];
  Serial.begin(115200);
  Serial.println();
  Serial.println("Empieza setup...");
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  pinMode(LED1, OUTPUT);    // inicializa GPIO como salida
  digitalWrite(LED1, HIGH); // apaga el led
 
  procesa_mensaje("infind/GRUPO1/led/cmd", (byte*)cadena, strlen(cadena));
 
  
  // CREACIÓN DE TOPICS
  sprintf(ID_PLACA, "ESP_%d", ESP.getChipId());
  sprintf(topic_SUB, "infind/%s/led/cmd", GRUPO);
  sprintf(topic_PUB1, "infind/%s/led/status", GRUPO);    
  sprintf(topic_PUB2, "infind/%s/datos", GRUPO);
  sprintf(topic_PUB3, "infind/%s/conexion", GRUPO);  

  bool wifi = WiFi.status();
  
  conecta_wifi();
  intenta_OTA();
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes
  mqtt_client.setCallback(procesa_mensaje);
  conecta_mqtt();
  Serial.printf("Identificador placa: %s\n", ID_PLACA );
  Serial.printf("Topic publicacion  : %s\n", topic_PUB1);
  Serial.printf("Topic publicacion  : %s\n", topic_PUB2);
  Serial.printf("Topic publicacion  : %s\n", topic_PUB3);
  Serial.printf("Topic subscripcion : %s\n", topic_SUB);
  Serial.printf("Termina setup en %lu ms\n\n",millis());

  // Segundo topic: conexión

StaticJsonDocument<300> conexion;

conexion["Online"]=wifi;

SerializeComplex(topic_PUB3,conexion);



  
//--------------------------------------------------------------------------------------------------
//                                      LECTURA DEL SENSOR
//--------------------------------------------------------------------------------------------------

 dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5
}

//-----------------------------------------------------
#define TAMANHO_MENSAJE 128
unsigned long ultimo_mensaje=0;
unsigned long ultima_recepcion=0;


//--------------------------------------------------------------------------------------------------
//                                          LOOP
//--------------------------------------------------------------------------------------------------
void loop() {
  if (!mqtt_client.connected()) conecta_mqtt();
  mqtt_client.loop(); // esta llamada para que la librería recupere el control
  unsigned long ahora = millis();
  if (ahora - ultimo_mensaje >= 30000) {

 delay(dht.getMinimumSamplingPeriod());

  float hum = dht.getHumidity();
  char cadena[512];
  float temp = dht.getTemperature();
  unsigned volt = ESP.getVcc()/1000;     //Lectura del nivel del voltaje en milivoltios
  int led;
  bool wifi = WiFi.status();
  float uptime;
  long rssi_ = WiFi.RSSI();
  String  IP = WiFi.localIP().toString().c_str();
    ultimo_mensaje = ahora;

    int tiempo = millis();



// Primer topic: datos

StaticJsonDocument<300> datos;
datos["Uptime"] = tiempo;
datos["Vcc"] = volt;

JsonObject DTH11 = datos.createNestedObject("DTH11");
DTH11["Temperatura"] = temp;
DTH11["Humedad"] = hum;

JsonObject Wifi = datos.createNestedObject("Wifi");
Wifi["SSId"] = ssid;
Wifi["IP"] = IP;
Wifi["RSSI"] = rssi_;

SerializeComplex(topic_PUB2,datos);

    

  }
  

}
