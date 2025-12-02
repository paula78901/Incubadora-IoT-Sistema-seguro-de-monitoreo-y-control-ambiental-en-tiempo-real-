#include <WiFi.h>
#include <AsyncMqttClient.hpp>
#include <AsyncTCP.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"
#include <Q2HX711.h>
#include <Preferences.h>

// -------------------------
// CONFIGURACIÓN WIFI Y MQTT
// -------------------------
const char* WIFI_SSID = "pcpuma";
const char* WIFI_PASS = "mango2022";

const char* MQTT_HOST = "10.59.183.127";
const uint16_t MQTT_PORT = 1884;

// Topics MQTT - Publicación
const char* TOPIC_TEMP = "incubadora/temperatura";
const char* TOPIC_HUM = "incubadora/humedad";
const char* TOPIC_CO2 = "incubadora/CO2";
const char* TOPIC_PRESION = "incubadora/presion";
const char* TOPIC_RANGOS = "incubadora/rangos";
const char* TOPIC_LEDS_ESTADO = "incubadora/leds/estado";

// Topics MQTT - Suscripción (Solo Configuración de Rangos)
const char* TOPIC_CONFIG_TEMP_MIN = "incubadora/config/tempMin";
const char* TOPIC_CONFIG_TEMP_MAX = "incubadora/config/tempMax";
const char* TOPIC_CONFIG_HUM_MIN = "incubadora/config/humMin";
const char* TOPIC_CONFIG_HUM_MAX = "incubadora/config/humMax";
const char* TOPIC_CONFIG_CO2_MIN = "incubadora/config/co2Min";
const char* TOPIC_CONFIG_CO2_MAX = "incubadora/config/co2Max";
const char* TOPIC_CONFIG_PRES_MIN = "incubadora/config/presMin";
const char* TOPIC_CONFIG_PRES_MAX = "incubadora/config/presMax";

// -------------------------
// SENSOR DE PRESIÓN
// -------------------------
const byte DOUT_PIN = 13;
const byte SCK_PIN = 27;
Q2HX711 hx711(DOUT_PIN, SCK_PIN);

const float PENDIENTE = 0.003047;
const long PUNTO_CERO = 16665945;

const float DENSIDAD_AGUA = 1000.0;
const float GRAVEDAD = 9.81;

// -------------------------
// OBJETOS
// -------------------------
AsyncMqttClient mqttClient;
Adafruit_AHTX0 aht;
ScioSense_ENS160 ens160(0x53);
Preferences prefs;

TimerHandle_t wifiReconnectTimer;
TimerHandle_t mqttReconnectTimer;

// -------------------------
// UART hacia ESP32-2
// -------------------------
#define TX_PIN 18
#define RX_PIN 19
HardwareSerial SerialESP2(1);
String uartBuffer = "";

// -------------------------
// VARIABLES DE ESTADO
// -------------------------
// Mediciones
float T = 0, H = 0, CO2 = 0, PkPa = 0;

// Rangos (guardados en NVS)
float tempMin = 20, tempMax = 30;
float humMin = 30, humMax = 60;
float co2Min = 400, co2Max = 3000;
float presMin = -10, presMax = 10;

// LEDs (5 LEDs) - Solo guardamos el estado ON/OFF actual
const int LED_PINS[5] = {15, 4, 16, 17, 5};
bool ledStates[5] = {false, false, false, false, false};

// Timers
unsigned long lastPublishMQTT = 0;
unsigned long lastPublishUART = 0;
unsigned long lastSensorRead = 0;
const unsigned long MQTT_INTERVAL = 2000;
const unsigned long UART_INTERVAL = 2000;
const unsigned long SENSOR_INTERVAL = 1000;

// *** NUEVO: Configuración de Ciclos (5s ON / 5s OFF) ***
const long T_ON = 5000;     // 5 segundos prendido
const long T_CICLO = 10000; // 10 segundos total del ciclo

// -------------------------
// PERSISTENCIA
// -------------------------
void guardarRangos() {
  prefs.putFloat("tempMin", tempMin);
  prefs.putFloat("tempMax", tempMax);
  prefs.putFloat("humMin", humMin);
  prefs.putFloat("humMax", humMax);
  prefs.putFloat("co2Min", co2Min);
  prefs.putFloat("co2Max", co2Max);
  prefs.putFloat("presMin", presMin);
  prefs.putFloat("presMax", presMax);
  Serial.println("Rangos guardados en NVS");
}

void cargarRangos() {
  tempMin = prefs.getFloat("tempMin", tempMin);
  tempMax = prefs.getFloat("tempMax", tempMax);
  humMin = prefs.getFloat("humMin", humMin);
  humMax = prefs.getFloat("humMax", humMax);
  co2Min = prefs.getFloat("co2Min", co2Min);
  co2Max = prefs.getFloat("co2Max", co2Max);
  presMin = prefs.getFloat("presMin", presMin);
  presMax = prefs.getFloat("presMax", presMax);
  Serial.println("Rangos cargados desde NVS");
}

// -------------------------
// FUNCIONES DE SENSORES
// -------------------------
float leerPresion() {
  long bits_raw = hx711.read();
  long bits = bits_raw - PUNTO_CERO;
  float altura_mm = PENDIENTE * bits;
  
  if (altura_mm < 0) altura_mm = 0;
  
  float altura_m = altura_mm / 1000.0;
  float presion_pa = DENSIDAD_AGUA * GRAVEDAD * altura_m;
  float presion_kpa = presion_pa / 1000.0;
  
  return presion_kpa;
}

void leerSensores() {
  // AHT
  sensors_event_t tempEvent, humEvent;
  if (aht.getEvent(&humEvent, &tempEvent)) {
    T = tempEvent.temperature;
    H = humEvent.relative_humidity;
  }
  
  // ENS160
  if (ens160.measure(true)) {
    CO2 = ens160.geteCO2();
  }
  
  // Presión
  PkPa = leerPresion();
}

// -------------------------
// CONTROL DE LEDs (MODIFICADO: REGLA 2 y 8)
// -------------------------
void actualizarLEDs() {
  // Calculamos el tiempo para el ciclo (0 a 10000 ms)
  unsigned long currentMillis = millis();
  unsigned long timeInCycle = currentMillis % T_CICLO; 

  // --- LED 0: CALEFACTOR (Pin 15) ---
  if (T < tempMin) {
    ledStates[0] = true; // ESTADO LÓGICO: "1" (Enviamos esto a la pantalla SIEMPRE que falte calor)
    
    // CONTROL FÍSICO (Pin): Ciclo 5s ON / 5s OFF
    if (timeInCycle < T_ON) {
      digitalWrite(LED_PINS[0], HIGH); 
    } else {
      digitalWrite(LED_PINS[0], LOW);
    }
  } else {
    ledStates[0] = false;
    digitalWrite(LED_PINS[0], LOW);
  }
  
  // --- LED 1: VENTILADOR (Pin 2) --- (Normal)
  ledStates[1] = (T > tempMax);
  digitalWrite(LED_PINS[1], ledStates[1] ? HIGH : LOW);
  
  // --- LED 2: HUMIDIFICADOR (Pin 16) --- (Ciclo 5s/5s)
  if (H < humMin) {
    ledStates[2] = true; // ESTADO LÓGICO: "1" (La pantalla lo ve prendido)
    
    // CONTROL FÍSICO (Pin): Ciclo 5s ON / 5s OFF
    if (timeInCycle < T_ON) {
      digitalWrite(LED_PINS[2], HIGH);
    } else {
      digitalWrite(LED_PINS[2], LOW);
    }
  } else {
    ledStates[2] = false;
    digitalWrite(LED_PINS[2], LOW);
  }
  
  // --- LED 3: CO2 (Pin 17) --- (Normal)
  ledStates[3] = (CO2 < co2Min || CO2 > co2Max);
  digitalWrite(LED_PINS[3], ledStates[3] ? HIGH : LOW);
  
  // --- LED 4: PRESIÓN (Pin 5) --- (Normal)
  ledStates[4] = (PkPa < presMin || PkPa > presMax);
  digitalWrite(LED_PINS[4], ledStates[4] ? HIGH : LOW);
}

// -------------------------
// COMUNICACIÓN UART
// -------------------------
void enviarDatosUART() {
  // Formato: DATA:T,H,CO2,P,TMin,TMax,HMin,HMax,CO2Min,CO2Max,PMin,PMax,L0,L1,L2,L3,L4
  SerialESP2.print("DATA:");
  SerialESP2.print(T, 1); SerialESP2.print(",");
  SerialESP2.print(H, 1); SerialESP2.print(",");
  SerialESP2.print(CO2, 0); SerialESP2.print(",");
  SerialESP2.print(PkPa, 3); SerialESP2.print(",");
  SerialESP2.print(tempMin, 1); SerialESP2.print(",");
  SerialESP2.print(tempMax, 1); SerialESP2.print(",");
  SerialESP2.print(humMin, 1); SerialESP2.print(",");
  SerialESP2.print(humMax, 1); SerialESP2.print(",");
  SerialESP2.print(co2Min, 0); SerialESP2.print(",");
  SerialESP2.print(co2Max, 0); SerialESP2.print(",");
  SerialESP2.print(presMin, 1); SerialESP2.print(",");
  SerialESP2.print(presMax, 1); SerialESP2.print(",");
  
  for (int i = 0; i < 5; i++) {
    // ENVIAMOS EL ESTADO LÓGICO (ledStates), NO EL FÍSICO
    SerialESP2.print(ledStates[i] ? "1" : "0");
    if (i < 4) SerialESP2.print(",");
  }
  SerialESP2.println();
}

void procesarComandoUART(String cmd) {
  Serial.print("<- UART: ");
  Serial.println(cmd);
  
  // *** ESTO ES LO QUE ARREGLA QUE LA OTRA PANTALLA NO RECIBA DATOS ***
  if (cmd.startsWith("GET:ALL")) {
    enviarDatosUART();
    return;
  }
  
  // SET:TempMin:22.5
  if (cmd.startsWith("SET:")) {
    cmd = cmd.substring(4);
    int colon = cmd.indexOf(':');
    String variable = cmd.substring(0, colon);
    float valor = cmd.substring(colon + 1).toFloat();
    
    if (variable == "TempMin") tempMin = valor;
    else if (variable == "TempMax") tempMax = valor;
    else if (variable == "HumMin") humMin = valor;
    else if (variable == "HumMax") humMax = valor;
    else if (variable == "CO2Min") co2Min = valor;
    else if (variable == "CO2Max") co2Max = valor;
    else if (variable == "PresMin") presMin = valor;
    else if (variable == "PresMax") presMax = valor;
    
    guardarRangos();
    publicarRangosMQTT(); // Publicar a MQTT cuando viene de UART
    actualizarLEDs();     // Verificar si el nuevo rango activa algo
    enviarDatosUART();    // Enviar confirmación al otro ESP32
    
    Serial.print("  Actualizado: ");
    Serial.print(variable);
    Serial.print(" = ");
    Serial.println(valor);
  }
  // IGNORAMOS CUALQUIER COMANDO "LEDMODE" o "LED" QUE LLEGUE DE UART
}

void leerUART() {
  while (SerialESP2.available()) {
    char c = SerialESP2.read();
    
    if (c == '\n') {
      if (uartBuffer.length() > 0) {
        procesarComandoUART(uartBuffer);
        uartBuffer = "";
      }
    } else {
      uartBuffer += c;
    }
  }
}

// -------------------------
// COMUNICACIÓN MQTT
// -------------------------
void publicarDatosMQTT() {
  if (!mqttClient.connected()) return;
  
  mqttClient.publish(TOPIC_TEMP, 1, false, String(T, 2).c_str());
  mqttClient.publish(TOPIC_HUM, 1, false, String(H, 2).c_str());
  mqttClient.publish(TOPIC_CO2, 1, false, String(CO2, 0).c_str());
  mqttClient.publish(TOPIC_PRESION, 1, false, String(PkPa, 3).c_str());
}

void publicarRangosMQTT() {
  if (!mqttClient.connected()) return;
  
  String json = "{";
  json += "\"tempMin\":" + String(tempMin, 1) + ",";
  json += "\"tempMax\":" + String(tempMax, 1) + ",";
  json += "\"humMin\":" + String(humMin, 1) + ",";
  json += "\"humMax\":" + String(humMax, 1) + ",";
  json += "\"co2Min\":" + String(co2Min, 0) + ",";
  json += "\"co2Max\":" + String(co2Max, 0) + ",";
  json += "\"presMin\":" + String(presMin, 1) + ",";
  json += "\"presMax\":" + String(presMax, 1);
  json += "}";
  
  mqttClient.publish(TOPIC_RANGOS, 1, true, json.c_str()); // retained
  Serial.println("  Rangos publicados a MQTT");
}

void publicarEstadoLEDs() {
  if (!mqttClient.connected()) return;
  
  // Publicar como JSON (Siempre AUTO)
  String json = "{";
  for (int i = 0; i < 5; i++) {
    json += "\"led" + String(i) + "\":{";
    json += "\"state\":" + String(ledStates[i] ? "true" : "false") + ",";
    json += "\"mode\":\"AUTO\""; 
    json += "}";
    if (i < 4) json += ",";
  }
  json += "}";
  
  mqttClient.publish(TOPIC_LEDS_ESTADO, 1, true, json.c_str()); // retained
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  
  String topicStr = String(topic);

  char temp[len + 1];       
  memcpy(temp, payload, len); 
  temp[len] = '\0';         
  String payloadStr = String(temp); 
  
  Serial.print("Recibido ["); Serial.print(topicStr); Serial.print("]: ");
  Serial.println(payloadStr);

  float valor = payloadStr.toFloat();
  bool rangosCambiaron = false;

  if (topicStr == TOPIC_CONFIG_TEMP_MIN) { tempMin = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_TEMP_MAX) { tempMax = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_HUM_MIN) { humMin = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_HUM_MAX) { humMax = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_CO2_MIN) { co2Min = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_CO2_MAX) { co2Max = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_PRES_MIN) { presMin = valor; rangosCambiaron = true; }
  else if (topicStr == TOPIC_CONFIG_PRES_MAX) { presMax = valor; rangosCambiaron = true; }

  // IGNORAMOS CUALQUIER MENSAJE DE CONTROL DE LEDS (MANUAL)

  if (rangosCambiaron) {
    guardarRangos();      
    actualizarLEDs();     
    enviarDatosUART();    
    publicarEstadoLEDs(); 
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("MQTT Conectado");
  
  // Suscripciones SOLO a configuración
  mqttClient.subscribe("incubadora/config/#", 1);

  mqttClient.subscribe(TOPIC_CONFIG_TEMP_MIN, 1);
  mqttClient.subscribe(TOPIC_CONFIG_TEMP_MAX, 1);
  mqttClient.subscribe(TOPIC_CONFIG_HUM_MIN, 1);
  mqttClient.subscribe(TOPIC_CONFIG_HUM_MAX, 1);
  mqttClient.subscribe(TOPIC_CONFIG_CO2_MIN, 1);
  mqttClient.subscribe(TOPIC_CONFIG_CO2_MAX, 1);
  mqttClient.subscribe(TOPIC_CONFIG_PRES_MIN, 1);
  mqttClient.subscribe(TOPIC_CONFIG_PRES_MAX, 1);
  
  Serial.println("Suscrito a topics de configuracion");
  
  publicarRangosMQTT();
  publicarEstadoLEDs();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("MQTT desconectado");
  if (WiFi.isConnected()) {
    Serial.println("Reintentando MQTT en 2s...");
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("WiFi conectado. IP: ");
      Serial.println(WiFi.localIP());
      mqttClient.connect();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi desconectado");
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// -------------------------
// SETUP
// -------------------------
void setup() {
  Serial.begin(115200);
  SerialESP2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(200);
  
  Serial.println("\n=================================");
  Serial.println("ESP32-1 Maestro (Full Auto) v6.0 Fixed");
  Serial.println("=================================\n");
  
  // Configurar LEDs
  for (int i = 0; i < 5; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  
  // Inicializar sensores
  if (!aht.begin()) {
    Serial.println("Error iniciando AHT");
  } else {
    Serial.println("AHT OK");
  }
  
  if (!ens160.begin()) {
    Serial.println("Error iniciando ENS160");
  } else {
    Serial.println("ENS160 OK");
    ens160.setMode(ENS160_OPMODE_STD);
  }
  
  Serial.println("Sensor de Presion OK");
  
  // Cargar configuración
  prefs.begin("incubadora", false);
  cargarRangos();
  // NO cargamos modos LED
  
  // Leer sensores iniciales
  Serial.println("\nLeyendo sensores iniciales...");
  leerSensores();
  
  // Actualizar LEDs INMEDIATAMENTE
  actualizarLEDs();
  Serial.println("LEDs en modo AUTOMATICO FORZADO");
  
  // Timers WiFi/MQTT
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, NULL,
                                    [](TimerHandle_t xTimer){ WiFi.begin(WIFI_SSID, WIFI_PASS); });
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, NULL,
                                    [](TimerHandle_t xTimer){ mqttClient.connect(); });
  
  // Configurar WiFi
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Configurar MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  
  Serial.println("\nSistema iniciado (Full Auto)!\n");
}

// -------------------------
// LOOP
// -------------------------
void loop() {
  // Leer comandos de ESP32-2
  leerUART();
  
  // Leer sensores periódicamente
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
    leerSensores();
    actualizarLEDs(); // SIEMPRE actualiza basado en lectura
  }
  
  // Publicar a MQTT cada 2 segundos
  if (millis() - lastPublishMQTT >= MQTT_INTERVAL) {
    lastPublishMQTT = millis();
    publicarDatosMQTT();
    publicarEstadoLEDs();
  }
  
  // Enviar datos a ESP32-2 cada 2 segundos
  if (millis() - lastPublishUART >= UART_INTERVAL) {
    lastPublishUART = millis();
    enviarDatosUART();
  }
  
  // Debug en Serial
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 5000) {
    lastDebug = millis();
    Serial.println("\n------------------------");
    Serial.print("T: "); Serial.print(T, 1); Serial.print("C ["); Serial.print(tempMin, 1); Serial.print("-"); Serial.print(tempMax, 1); Serial.println("]");
    Serial.print("H: "); Serial.print(H, 1); Serial.print("% ["); Serial.print(humMin, 1); Serial.print("-"); Serial.print(humMax, 1); Serial.println("]");
    Serial.print("CO2: "); Serial.print(CO2, 0); Serial.print(" ppm ["); Serial.print(co2Min, 0); Serial.print("-"); Serial.print(co2Max, 0); Serial.println("]");
    Serial.print("P: "); Serial.print(PkPa, 3); Serial.print(" kPa ["); Serial.print(presMin, 1); Serial.print("-"); Serial.print(presMax, 1); Serial.println("]");
    Serial.print("LEDs (SIEMPRE AUTO): ");
    for (int i = 0; i < 5; i++) {
      Serial.print(ledStates[i] ? "[ON]" : "[OFF]");
      Serial.print(" ");
    }
    Serial.println("\n------------------------");
  }
  
  delay(50);
}