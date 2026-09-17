#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"

// ==========================================
// DEFINICIÓN DE PINES
// ==========================================
#define PIN_LED_OK       12  // LED 1: Acceso autorizado / Sesión activa
#define PIN_LED_FAIL     14  // LED 2: Acceso denegado / Alarma
#define PIN_BUZZER       15  // Buzzer acústico

#define ONE_WIRE_BUS     4   // Bus OneWire (DS18B20)

// Bus I2C #0: MAX30102
#define I2C0_SDA         21
#define I2C0_SCL         22

// Bus I2C #1: MPU6050
#define I2C1_SDA         32
#define I2C1_SCL         33
#define MPU_ADDR         0x68

// Bus SPI: RFID-RC522
#define RC522_SS_PIN     5
#define RC522_RST_PIN    27

// ==========================================
// INSTANCIAS Y VARIABLES GLOBALES
// ==========================================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorTemp(&oneWire);
MAX30105 sensorOptico;
MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);
TwoWire I2Cone = TwoWire(1);

QueueHandle_t colaEventosAcceso;
enum TipoAcceso { ACCESO_VALIDO, ACCESO_INVALIDO };
const byte UID_AUTORIZADO[4] = {0x61, 0x2A, 0x88, 0x17};

// Control de sesión clínica y telemetría protegida
volatile bool g_sesionActiva = false;
volatile float g_tempC = 0.0;
volatile float g_aTotal = 1.0;
volatile float g_aPicoMax = 1.0;
volatile bool  g_alertaImpacto = false;
volatile long  g_irValue = 0;
volatile float g_bpmInst = 0.0;
volatile int   g_bpmAvg = 0;
volatile int   g_spo2 = 0;

// ==========================================
// TAREA 1: MAX30102 (Core 0, 100 Hz FIFO)
// ==========================================
void TareaMAX30102(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(10);

  long lastBeat = 0;
  const byte RATE_SIZE = 4;
  byte rates[RATE_SIZE] = {0};
  byte rateSpot = 0;

  long minRed = 999999, maxRed = 0;
  long minIR = 999999, maxIR = 0;
  int muestras = 0;

  for (;;) {
    sensorOptico.check();

    while (sensorOptico.available()) {
      long irVal = sensorOptico.getFIFOIR();
      long redVal = sensorOptico.getFIFORed();
      sensorOptico.nextSample();

      g_irValue = irVal;

      if (irVal > 50000 && irVal < 250000) {
        if (redVal < minRed) minRed = redVal;
        if (redVal > maxRed) maxRed = redVal;
        if (irVal < minIR) minIR = irVal;
        if (irVal > maxIR) maxIR = irVal;
        muestras++;

        if (checkForBeat(irVal)) {
          long delta = millis() - lastBeat;
          lastBeat = millis();

          float bpmCalculado = 60.0 / (delta / 1000.0);

          if (bpmCalculado >= 45 && bpmCalculado <= 200) {
            g_bpmInst = bpmCalculado;
            rates[rateSpot++] = (byte)bpmCalculado;
            rateSpot %= RATE_SIZE;

            int suma = 0;
            for (byte i = 0; i < RATE_SIZE; i++) suma += rates[i];
            g_bpmAvg = suma / RATE_SIZE;

            long acRed = maxRed - minRed;
            long dcRed = (maxRed + minRed) / 2;
            long acIR = maxIR - minIR;
            long dcIR = (maxIR + minIR) / 2;

            if (dcRed > 0 && dcIR > 0 && acIR > 0) {
              float ratio = ((float)acRed / (float)dcRed) / ((float)acIR / (float)dcIR);
              int calcSpO2 = (int)(110.0 - (25.0 * ratio));
              if (calcSpO2 > 100) calcSpO2 = 99;
              if (calcSpO2 >= 85) g_spo2 = calcSpO2;
            }

            minRed = 999999; maxRed = 0;
            minIR = 999999;  maxIR = 0;
            muestras = 0;
          }
        }
      } else {
        g_bpmInst = 0.0;
        g_bpmAvg = 0;
        g_spo2 = 0;
        minRed = 999999; maxRed = 0;
        minIR = 999999;  maxIR = 0;
        muestras = 0;
      }

      if (muestras > 250) {
        minRed = 999999; maxRed = 0;
        minIR = 999999;  maxIR = 0;
        muestras = 0;
      }
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 2: MPU6050 (Core 0, 50 Hz)
// ==========================================
void TareaMPU6050(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(20);

  for (;;) {
    I2Cone.beginTransmission(MPU_ADDR);
    I2Cone.write(0x3B);
    byte err = I2Cone.endTransmission(false);

    if (err == 0) {
      byte len = I2Cone.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);
      if (len == 6) {
        int16_t ax = I2Cone.read() << 8 | I2Cone.read();
        int16_t ay = I2Cone.read() << 8 | I2Cone.read();
        int16_t az = I2Cone.read() << 8 | I2Cone.read();

        float atotal = sqrt(pow(ax / 16384.0, 2) + pow(ay / 16384.0, 2) + pow(az / 16384.0, 2));
        g_aTotal = atotal;

        if (atotal > g_aPicoMax) g_aPicoMax = atotal;
        if (atotal > 2.30) {
          g_alertaImpacto = true;
        }
      }
    } else {
      I2Cone.beginTransmission(MPU_ADDR);
      I2Cone.write(0x6B);
      I2Cone.write(0x00);
      I2Cone.endTransmission();
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 3: DS18B20 (Core 0, 1 Hz)
// ==========================================
void TareaTemperatura(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000);

  for (;;) {
    sensorTemp.requestTemperatures();
    float tC = sensorTemp.getTempCByIndex(0);
    if (tC != DEVICE_DISCONNECTED_C) {
      g_tempC = tC;
    }
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 4: RFID-RC522 (Core 1, 150 ms)
// ==========================================
void TareaRFID(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(150);

  for (;;) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      bool autorizado = true;
      for (byte i = 0; i < 4; i++) {
        if (rfid.uid.uidByte[i] != UID_AUTORIZADO[i]) autorizado = false;
      }

      TipoAcceso evento = autorizado ? ACCESO_VALIDO : ACCESO_INVALIDO;
      xQueueSend(colaEventosAcceso, &evento, 0);

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 5: ACTUADORES LOCALES Y SESIÓN (Core 1)
// ==========================================
void TareaActuadores(void *pvParameters) {
  TipoAcceso eventoRecibido;

  for (;;) {
    if (xQueueReceive(colaEventosAcceso, &eventoRecibido, portMAX_DELAY) == pdTRUE) {
      if (eventoRecibido == ACCESO_VALIDO) {
        g_sesionActiva = !g_sesionActiva; // Alternar estado de sesión

        if (g_sesionActiva) {
          digitalWrite(PIN_LED_OK, HIGH); // LED encendido permanente durante sesión activa
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(150)); // Pip corto de confirmación
          digitalWrite(PIN_BUZZER, LOW);
        } else {
          digitalWrite(PIN_LED_OK, LOW);  // Apagar LED al cerrar sesión
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(PIN_BUZZER, LOW);
          vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(PIN_BUZZER, LOW);
        }
      } else {
        // Acceso denegado: bloqueo forzado inmediato
        g_sesionActiva = false;
        digitalWrite(PIN_LED_OK, LOW);

        for (int i = 0; i < 3; i++) {
          digitalWrite(PIN_LED_FAIL, HIGH);
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(100));
          digitalWrite(PIN_LED_FAIL, LOW);
          digitalWrite(PIN_BUZZER, LOW);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      }
    }
  }
}

// ==========================================
// TAREA 6: TELEMETRÍA CONDICIONADA (Core 1, 1 Hz)
// ==========================================
void TareaTelemetria(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000);

  for (;;) {
    if (g_sesionActiva) {
      char strCaida[25];
      if (g_alertaImpacto) {
        snprintf(strCaida, sizeof(strCaida), "ALERTA (IMPACTO)");
        g_alertaImpacto = false;
      } else {
        snprintf(strCaida, sizeof(strCaida), "Estable");
      }

      char strBpm[30];
      if (g_bpmAvg > 0) {
        snprintf(strBpm, sizeof(strBpm), "%.0f BPM (Media: %d)", g_bpmInst, g_bpmAvg);
      } else {
        snprintf(strBpm, sizeof(strBpm), "Sin pulso detectado");
      }

      Serial.printf("[SESION ACTIVA] Temp: %.2f °C | Aceleracion: %.2f g (Pico: %.2f g) [%s] | Oximetro: [Dedo: %s | Ritmo: %s | Oxigeno: %d%%]\n",
                    g_tempC,
                    g_aTotal,
                    g_aPicoMax,
                    strCaida,
                    (g_irValue > 50000) ? "DETECTADO" : "NO DETECTADO",
                    strBpm,
                    g_spo2);

      g_aPicoMax = g_aTotal;
    } else {
      Serial.println("[ACCESO BLOQUEADO] Sistema en reposo. Acerque credencial autorizada para iniciar monitoreo clinico...");
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_FAIL, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_FAIL, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  Wire.begin(I2C0_SDA, I2C0_SCL);
  Wire.setTimeOut(50);

  I2Cone.begin(I2C1_SDA, I2C1_SCL);
  I2Cone.setTimeOut(50);

  SPI.begin();
  sensorTemp.begin();
  sensorTemp.setWaitForConversion(false);
  rfid.PCD_Init();

  // MPU6050
  I2Cone.beginTransmission(MPU_ADDR);
  I2Cone.write(0x6B);
  I2Cone.write(0x00);
  I2Cone.endTransmission();

  // MAX30102
  if (sensorOptico.begin(Wire, I2C_SPEED_STANDARD)) {
    sensorOptico.setup(0x1F, 4, 2, 100, 411, 4096);
    sensorOptico.setPulseAmplitudeRed(0x1F);
    sensorOptico.setPulseAmplitudeIR(0x1F);
  }

  colaEventosAcceso = xQueueCreate(5, sizeof(TipoAcceso));

  if (colaEventosAcceso != NULL) {
    xTaskCreatePinnedToCore(TareaMAX30102,    "Task_MAX",    1024 * 4, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TareaMPU6050,     "Task_MPU",    1024 * 3, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TareaTemperatura, "Task_Temp",   1024 * 4, NULL, 1, NULL, 0);

    xTaskCreatePinnedToCore(TareaRFID,        "Task_RFID",   1024 * 4, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TareaActuadores,  "Task_Actua",  1024 * 3, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TareaTelemetria,  "Task_Telem",  1024 * 3, NULL, 1, NULL, 1);
  }
}

void loop() {
  vTaskDelete(NULL);
}
