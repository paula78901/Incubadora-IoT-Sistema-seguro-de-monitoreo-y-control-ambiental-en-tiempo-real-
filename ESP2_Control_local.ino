#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

// ==========================================
// HARDWARE
// ==========================================
LiquidCrystal_PCF8574 lcd(0x27);

// Botones
const int BTN_OK = 14;         // Confirmar / OK
const int BTN_CONFIG = 25;     // Ciclar menús
const int BTN_UP = 33;         // Arriba / Incrementar
const int BTN_DOWN = 32;       // Abajo / Decrementar
const int BTN_MIN = 27;        // Editar Mínimo
const int BTN_MAX = 12;        // Editar Máximo

// LEDs
const int LED_PINS[] = {15, 2, 16, 17, 5};

// Comunicación
#define TX_PIN 18
#define RX_PIN 19
HardwareSerial SerialESP1(1);

// ==========================================
// VARIABLES DE ESTADO
// ==========================================
float T = 0, H = 0, CO2 = 0, PkPa = 0;
float tempMin = 20, tempMax = 30;
float humMin = 30, humMax = 60;
float co2Min = 400, co2Max = 3000;
float presMin = -10, presMax = 10;

// Estado LEDs
bool ledStates[5] = {false, false, false, false, false};
bool ledModes[5] = {true, true, true, true, true}; // true=AUTO

// Control de Pantallas
enum Pantalla {
  LECTURAS,      // 0: Monitor
  MENU_CONFIG,   // 1: Selección Variable + Ver Rangos
  EDIT_RANGOS,   // 2: Editando con Botones UP/DOWN
  MENU_LEDS      // 3: Control Manual LEDs
};

Pantalla pantallaActual = LECTURAS;
int menuIndex = 0; // 0=Temp, 1=Hum, 2=CO2, 3=Pres
int ledIndex = 0;  // 0-4

// Edición
bool editandoMin = true; // true=Min, false=Max
float valorEdit = 0;
float valorInicial = 0; // Para restaurar si cancela

// Incrementos según variable
float incrementoPequeno = 0.5;

// Repetición automática de botones
unsigned long btnHoldTime = 0;
int btnHoldPin = -1;
bool btnHolding = false;
const unsigned long holdThreshold = 600;  // Tiempo para activar repetición
const unsigned long holdRepeatInterval = 150; // Intervalo de repetición

// Timers
String uartBuffer = "";
bool needsUpdate = true;

// Debounce
unsigned long lastBtnTime = 0;

// Control de actualización de pantalla
unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 100;

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================

void enviarComando(String cmd) {
  SerialESP1.println(cmd);
  Serial.println("TX: " + cmd);
}

void incrementarValor(bool aumentar) {
  float paso = 0;
  
  // Determinar paso según variable
  if (menuIndex == 0) { // Temperatura: 0.5°C
    paso = 0.5;
    valorEdit += aumentar ? paso : -paso;
    valorEdit = constrain(valorEdit, 0, 50);
    valorEdit = round(valorEdit * 2) / 2.0; // Redondear a 0.5
  }
  else if (menuIndex == 1) { // Humedad: 1%
    paso = 1.0;
    valorEdit += aumentar ? paso : -paso;
    valorEdit = constrain(valorEdit, 0, 100);
    valorEdit = round(valorEdit);
  }
  else if (menuIndex == 2) { // CO2: 100 ppm
    paso = 100.0; // CAMBIO: Intervalo fijo de 100
    valorEdit += aumentar ? paso : -paso;
    valorEdit = constrain(valorEdit, 400, 5000);
    // Asegurar que siempre sea múltiplo de 100
    valorEdit = ((int)valorEdit / 100) * 100; 
  }
  else if (menuIndex == 3) { // Presión: 0.5 kPa
    paso = 0.5;
    valorEdit += aumentar ? paso : -paso;
    valorEdit = constrain(valorEdit, -20, 20);
    valorEdit = round(valorEdit * 2) / 2.0; // Redondear a 0.5
  }
  
  needsUpdate = true;
}

bool checkBtn(int pin, bool allowHold = false) {
  bool pressed = (digitalRead(pin) == LOW);
  
  if (pressed) {
    // Primera presión
    if (btnHoldPin != pin) {
      if (millis() - lastBtnTime > 200) { // Debounce
        lastBtnTime = millis();
        btnHoldPin = pin;
        btnHoldTime = millis();
        btnHolding = false;
        return true;
      }
    }
    // Mantener presionado (solo si allowHold está activo)
    else if (allowHold && !btnHolding && millis() - btnHoldTime > holdThreshold) {
      btnHolding = true;
      return true;
    }
    // Repetición automática
    else if (allowHold && btnHolding && millis() - lastBtnTime > holdRepeatInterval) {
      lastBtnTime = millis();
      return true;
    }
  } else {
    // Soltar botón
    if (btnHoldPin == pin) {
      btnHoldPin = -1;
      btnHolding = false;
    }
  }
  
  return false;
}

// ==========================================
// RECEPCIÓN DE DATOS
// ==========================================
void parsearDatos(String data) {
  if (data.startsWith("DATA:")) {
    data = data.substring(5);
    
    float valores[17];
    int idx = 0, lastPos = 0;
    
    for(int i=0; i<=data.length(); i++) {
      if(i == data.length() || data[i] == ',') {
        String val = data.substring(lastPos, i);
        valores[idx] = val.toFloat();
        lastPos = i + 1;
        idx++;
        if(idx >= 17) break;
      }
    }
    
    // Actualizar solo si no estamos editando
    if (pantallaActual != EDIT_RANGOS) {
      T = valores[0];
      H = valores[1];
      CO2 = valores[2];
      PkPa = valores[3];
      tempMin = valores[4];
      tempMax = valores[5];
      humMin = valores[6];
      humMax = valores[7];
      co2Min = valores[8];
      co2Max = valores[9];
      presMin = valores[10];
      presMax = valores[11];
    }
    
    // LEDs siempre se actualizan
    for(int i=0; i<5; i++) {
      ledStates[i] = (valores[12+i] == 1);
      digitalWrite(LED_PINS[i], ledStates[i]);
    }
    
    if (pantallaActual == LECTURAS || pantallaActual == MENU_CONFIG) {
      needsUpdate = true;
    }
  }
}

// ==========================================
// PANTALLAS
// ==========================================

void mostrarLecturas() {
  lcd.setCursor(0, 0);
  lcd.print("T:");
  if(T < 10) lcd.print(" ");
  lcd.print(T, 1); lcd.print((char)223);
  lcd.print(" H:");
  if(H < 10) lcd.print(" ");
  lcd.print(H, 0); lcd.print("%  ");
  
  lcd.setCursor(0, 1);
  lcd.print("CO2:");
  if(CO2 < 1000) lcd.print(" ");
  lcd.print((int)CO2);
  lcd.print(" P:");
  if(PkPa >= 0 && PkPa < 10) lcd.print(" ");
  lcd.print(PkPa, 1); lcd.print("  ");
}

void mostrarMenu() {
  String opts[] = {"Temp", "Hum", "CO2", "Pres"};
  float minVal, maxVal;
  String unit;
  
  // Obtener valores actuales
  if(menuIndex == 0) { minVal = tempMin; maxVal = tempMax; unit = "C"; }
  else if(menuIndex == 1) { minVal = humMin; maxVal = humMax; unit = "%"; }
  else if(menuIndex == 2) { minVal = co2Min; maxVal = co2Max; unit = ""; }
  else { minVal = presMin; maxVal = presMax; unit = "kPa"; }
  
  lcd.setCursor(0, 0);
  lcd.print(">");
  lcd.print(opts[menuIndex]);
  lcd.print(" [MIN/MAX] ");
  
  lcd.setCursor(0, 1);
  if(menuIndex == 2) {
    lcd.print((int)minVal);
    lcd.print("-");
    lcd.print((int)maxVal);
  } else {
    lcd.print(minVal, 1);
    lcd.print("-");
    lcd.print(maxVal, 1);
    lcd.print(unit);
  }
  lcd.print("      ");
}

void mostrarEdicion() {
  String opts[] = {"Temp", "Hum", "CO2", "Pres"};
  String unit[] = {"C", "%", "ppm", "kPa"};
  String pasos[] = {"0.5", "1", "100", "0.5"}; // Visualización actualizada a 100
  
  lcd.setCursor(0, 0);
  lcd.print(opts[menuIndex]);
  lcd.print(editandoMin ? " MIN" : " MAX");
  lcd.print(" +/-");
  lcd.print(pasos[menuIndex]);
  lcd.print("  ");
  
  lcd.setCursor(0, 1);
  lcd.print(">");
  
  if (menuIndex == 2) {
    lcd.print((int)valorEdit);
  } else {
    lcd.print(valorEdit, 1);
  }
  lcd.print(" ");
  lcd.print(unit[menuIndex]);
  lcd.print("        ");
}

void mostrarLeds() {
  String nombres[] = {"T.Baja", "T.Alta", "Humed", "CO2", "Pres"};
  
  lcd.setCursor(0, 0);
  lcd.print("LED"); lcd.print(ledIndex+1); lcd.print(":");
  lcd.print(nombres[ledIndex]);
  lcd.print("     ");
  
  lcd.setCursor(0, 1);
  lcd.print(ledStates[ledIndex] ? "[ON] " : "[OFF]");
  lcd.print(ledModes[ledIndex] ? "AUTO" : "MANU");
  lcd.print("    ");
}

// ==========================================
// SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  SerialESP1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Wire.begin(21, 22);
  
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_CONFIG, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_MIN, INPUT_PULLUP);
  pinMode(BTN_MAX, INPUT_PULLUP);
  
  for(int i=0; i<5; i++) pinMode(LED_PINS[i], OUTPUT);
  
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.clear();
  lcd.print(" SISTEMA LISTO ");
  delay(1500);
  lcd.clear();
  
  // Solicitar datos iniciales
  enviarComando("GET:ALL");
}

void loop() {
  // Leer UART
  while (SerialESP1.available()) {
    char c = SerialESP1.read();
    if (c == '\n') {
      parsearDatos(uartBuffer);
      uartBuffer = "";
    } else if (c != '\r') {
      uartBuffer += c;
    }
  }

  // BOTÓN CONFIG: Ciclar entre pantallas
  if (checkBtn(BTN_CONFIG)) {
    lcd.clear();
    
    if (pantallaActual == LECTURAS) {
      pantallaActual = MENU_CONFIG;
      menuIndex = 0;
    }
    else if (pantallaActual == MENU_CONFIG) {
      pantallaActual = MENU_LEDS;
      ledIndex = 0;
    }
    else if (pantallaActual == MENU_LEDS) {
      pantallaActual = LECTURAS;
    }
    else if (pantallaActual == EDIT_RANGOS) {
      // Cancelar edición
      pantallaActual = MENU_CONFIG;
      lcd.print("  CANCELADO!    ");
      delay(600);
      lcd.clear();
    }
    
    needsUpdate = true;
  }

  // MAQUINA DE ESTADOS
  switch (pantallaActual) {
    
    case LECTURAS:
      if (needsUpdate && millis() - lastLcdUpdate > lcdUpdateInterval) {
        mostrarLecturas();
        needsUpdate = false;
        lastLcdUpdate = millis();
      }
      break;

    case MENU_CONFIG:
      // Navegar entre variables
      if (checkBtn(BTN_UP)) {
        menuIndex--;
        if(menuIndex < 0) menuIndex = 3;
        needsUpdate = true;
      }
      
      if (checkBtn(BTN_DOWN)) {
        menuIndex++;
        if(menuIndex > 3) menuIndex = 0;
        needsUpdate = true;
      }
      
      // Entrar a Editar MIN
      if (checkBtn(BTN_MIN)) {
        pantallaActual = EDIT_RANGOS;
        editandoMin = true;
        
        if(menuIndex==0) valorEdit = valorInicial = tempMin;
        else if(menuIndex==1) valorEdit = valorInicial = humMin;
        else if(menuIndex==2) valorEdit = valorInicial = co2Min;
        else valorEdit = valorInicial = presMin;
        
        lcd.clear();
        needsUpdate = true;
      }
      
      // Entrar a Editar MAX
      if (checkBtn(BTN_MAX)) {
        pantallaActual = EDIT_RANGOS;
        editandoMin = false;
        
        if(menuIndex==0) valorEdit = valorInicial = tempMax;
        else if(menuIndex==1) valorEdit = valorInicial = humMax;
        else if(menuIndex==2) valorEdit = valorInicial = co2Max;
        else valorEdit = valorInicial = presMax;
        
        lcd.clear();
        needsUpdate = true;
      }
      
      if (needsUpdate && millis() - lastLcdUpdate > lcdUpdateInterval) {
        mostrarMenu();
        needsUpdate = false;
        lastLcdUpdate = millis();
      }
      break;

    case EDIT_RANGOS:
      // Ajustar valor con botones UP/DOWN (con soporte para mantener presionado)
      if (checkBtn(BTN_UP, true)) {
        incrementarValor(true);
      }
      
      if (checkBtn(BTN_DOWN, true)) {
        incrementarValor(false);
      }
      
      // BOTÓN OK: Guardar y volver
      if (checkBtn(BTN_OK)) {
        lcd.clear();
        lcd.print("  GUARDANDO...  ");
        
        String vars[] = {"Temp", "Hum", "CO2", "Pres"};
        String tipo = editandoMin ? "Min" : "Max";
        enviarComando("SET:" + vars[menuIndex] + tipo + ":" + String(valorEdit, 1));
        
        // Actualizar valor local
        if(editandoMin) {
          if(menuIndex==0) tempMin=valorEdit;
          else if(menuIndex==1) humMin=valorEdit;
          else if(menuIndex==2) co2Min=valorEdit;
          else presMin=valorEdit;
        } else {
          if(menuIndex==0) tempMax=valorEdit;
          else if(menuIndex==1) humMax=valorEdit;
          else if(menuIndex==2) co2Max=valorEdit;
          else presMax=valorEdit;
        }
        
        delay(400);
        lcd.clear();
        lcd.print("   GUARDADO!    ");
        delay(600);
        
        pantallaActual = MENU_CONFIG;
        lcd.clear();
        needsUpdate = true;
      }
      
      if (needsUpdate && millis() - lastLcdUpdate > lcdUpdateInterval) {
        mostrarEdicion();
        needsUpdate = false;
        lastLcdUpdate = millis();
      }
      break;

    case MENU_LEDS:
      if (checkBtn(BTN_UP)) {
        ledIndex--;
        if(ledIndex < 0) ledIndex = 4;
        needsUpdate = true;
      }
      
      if (checkBtn(BTN_DOWN)) {
        ledIndex++;
        if(ledIndex > 4) ledIndex = 0;
        needsUpdate = true;
      }
      
      // BTN_MIN: Toggle AUTO/MANUAL
      if (checkBtn(BTN_MIN)) {
        ledModes[ledIndex] = !ledModes[ledIndex];
        enviarComando("LEDMODE:" + String(ledIndex) + "," + (ledModes[ledIndex]?"1":"0"));
        needsUpdate = true;
      }
      
      // BTN_MAX o BTN_OK: Toggle ON/OFF (solo en MANUAL)
      if (checkBtn(BTN_MAX) || checkBtn(BTN_OK)) {
        if (!ledModes[ledIndex]) {
          bool newState = !ledStates[ledIndex];
          ledStates[ledIndex] = newState;
          digitalWrite(LED_PINS[ledIndex], newState);
          enviarComando("LED:" + String(ledIndex) + ":" + (newState?"ON":"OFF"));
          
          // Pequeña pausa para dar feedback visual
          delay(100);
          needsUpdate = true;
        }
      }
      
      if (needsUpdate && millis() - lastLcdUpdate > lcdUpdateInterval) {
        mostrarLeds();
        needsUpdate = false;
        lastLcdUpdate = millis();
      }
      break;
  }
  
  // Refresco automático en monitor (cada 2 segundos)
  static unsigned long lastRefresh = 0;
  if (pantallaActual == LECTURAS && millis() - lastRefresh > 2000) {
    needsUpdate = true;
    lastRefresh = millis();
  }
  
  // Solicitar datos periódicamente (cada 3 segundos)
  static unsigned long lastRequest = 0;
  if (millis() - lastRequest > 3000) {
    enviarComando("GET:ALL");
    lastRequest = millis();
  }
}