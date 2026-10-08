// =====================================================================
// Laboratorio de Electromedicina III (CBM413) - Entrega inicial
// Nodo hardware ESP32: MAX30102 + MPU6050 + DS18B20 + RFID-RC522
//
// Las líneas marcadas con [v2] son cambios respecto de la versión del
// avance. Antes de entregar: reescribir estos comentarios con palabras
// propias del grupo y subir cada cambio en un commit separado.
// =====================================================================
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

// [v2] Registros del MPU6050 usados en la configuración
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75

// [v2] Rango ±8 g -> 4096 LSB/g (con ±2 g el sensor se satura en 2 g
//      y nunca alcanzaría un umbral de impacto de 2,5 g en un solo eje)
const float MPU_LSB_POR_G = 4096.0f;

// [v2] Frecuencia efectiva del MAX30102 = 400 sps / promedio de 4 = 100 Hz
const float FS_MAX_HZ = 100.0f;

// [v2] Umbrales del algoritmo cinemático de caída (a 50 Hz, 1 muestra = 20 ms)
//      Valores iniciales: deben ajustarse con las pruebas del grupo.
const float    UMBRAL_CAIDA_LIBRE_G      = 0.50f;
const float    UMBRAL_IMPACTO_G          = 2.50f;
const float    TOLERANCIA_INMOVIL_G      = 0.25f;
const uint16_t MIN_MUESTRAS_CAIDA_LIBRE  = 3;    //  60 ms bajo 0,5 g
const uint16_t VENTANA_IMPACTO_MUESTRAS  = 25;   // 500 ms para que llegue el impacto
const uint16_t MUESTRAS_REBOTE           = 25;   // 500 ms de rebote tras el impacto
const uint16_t VENTANA_INMOVIL_MUESTRAS  = 100;  //   2 s de observación posterior
const uint16_t MIN_MUESTRAS_QUIETAS      = 80;   //  80 % de la ventana sin movimiento

// ==========================================
// INSTANCIAS Y VARIABLES GLOBALES
// ==========================================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorTemp(&oneWire);
DeviceAddress dirDS18B20;          // [v2] se busca una sola vez en setup()
MAX30105 sensorOptico;
MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);
TwoWire I2Cone = TwoWire(1);

// [v2] La cola de actuadores ahora también recibe la alarma de caída
enum EventoLocal { ACCESO_VALIDO, ACCESO_INVALIDO, ALERTA_CAIDA };
QueueHandle_t colaEventosAcceso;

// [v2] Lista de credenciales autorizadas (agregar una fila por operario)
const byte UIDS_AUTORIZADOS[][4] = {
  {0x61, 0x2A, 0x88, 0x17},
};
const size_t N_UIDS = sizeof(UIDS_AUTORIZADOS) / sizeof(UIDS_AUTORIZADOS[0]);

// [v2] Eventos de caída: viajan por cola para que ninguno se pierda
enum TipoEventoCaida { EVENTO_IMPACTO, EVENTO_CAIDA_CONFIRMADA };
typedef struct {
  TipoEventoCaida tipo;
  bool     huboCaidaLibre;
  float    picoG;
  uint32_t t_ms;
} EventoCaida;
QueueHandle_t colaEventosCaida;

// [v2] Los datos compartidos entre núcleos se agrupan en una estructura
//      protegida por un mutex (antes eran variables volatile sueltas)
typedef struct {
  float    tempC;
  float    aTotal;
  float    aPicoMax;
  long     irValue;
  float    bpmInst;
  int      bpmAvg;
  int      spo2;
  uint32_t erroresMPU;
  uint32_t lecturasTempInvalidas;
  uint32_t eventosCaidaPerdidos;
} DatosClinicos;

DatosClinicos g_datos = {0.0f, 1.0f, 1.0f, 0, 0.0f, 0, 0, 0, 0, 0};
SemaphoreHandle_t mutexDatos;
const TickType_t ESPERA_MUTEX = pdMS_TO_TICKS(2);   // nunca se espera indefinidamente

// Estado de sesión: un solo escritor (TareaActuadores), lectura atómica (bool)
volatile bool g_sesionActiva = false;

// [v2] Handles para monitorear el uso de stack de cada tarea
TaskHandle_t hMAX, hMPU, hTemp, hRFID, hActua, hTelem;

// ==========================================
// [v2] FUNCIONES AUXILIARES MPU6050
// ==========================================
bool escribirRegistroMPU(uint8_t reg, uint8_t valor) {
  I2Cone.beginTransmission(MPU_ADDR);
  I2Cone.write(reg);
  I2Cone.write(valor);
  return I2Cone.endTransmission() == 0;
}

bool configurarMPU6050() {
  bool ok = true;
  ok &= escribirRegistroMPU(MPU_REG_PWR_MGMT_1, 0x01);   // despierta, reloj PLL eje X
  ok &= escribirRegistroMPU(MPU_REG_CONFIG, 0x04);       // DLPF 21 Hz < fs/2 = 25 Hz (antialiasing)
  ok &= escribirRegistroMPU(MPU_REG_SMPLRT_DIV, 19);     // 1 kHz / (1+19) = 50 Hz
  ok &= escribirRegistroMPU(MPU_REG_ACCEL_CONFIG, 0x10); // AFS_SEL = 2 -> ±8 g
  return ok;
}

// ==========================================
// TAREA 1: MAX30102 (Core 1, sondeo cada 10 ms, FIFO a 100 Hz)
// ==========================================
void TareaMAX30102(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(10);

  // [v2] El tiempo entre latidos se mide contando muestras de la FIFO
  //      (1 muestra = 10 ms) y no con millis(), que depende de cuándo
  //      se vació la FIFO y por lo tanto introduce jitter.
  uint32_t nMuestra = 0;
  uint32_t muestraUltimoLatido = 0;
  const byte RATE_SIZE = 4;
  byte rates[RATE_SIZE] = {0};
  byte rateSpot = 0;

  long minRed = 999999, maxRed = 0;
  long minIR = 999999, maxIR = 0;
  int muestras = 0;

  // Copias locales: se publican una vez por ciclo bajo el mutex
  long  irLocal = 0;
  float bpmInstLocal = 0.0f;
  int   bpmAvgLocal = 0;
  int   spo2Local = 0;

  for (;;) {
    sensorOptico.check();

    while (sensorOptico.available()) {
      long irVal = sensorOptico.getFIFOIR();
      long redVal = sensorOptico.getFIFORed();
      sensorOptico.nextSample();
      nMuestra++;
      irLocal = irVal;

      if (irVal > 50000 && irVal < 250000) {
        if (redVal < minRed) minRed = redVal;
        if (redVal > maxRed) maxRed = redVal;
        if (irVal < minIR) minIR = irVal;
        if (irVal > maxIR) maxIR = irVal;
        muestras++;

        if (checkForBeat(irVal)) {
          uint32_t deltaMuestras = nMuestra - muestraUltimoLatido;
          muestraUltimoLatido = nMuestra;

          float bpmCalculado = 60.0f * FS_MAX_HZ / (float)deltaMuestras;

          if (bpmCalculado >= 45 && bpmCalculado <= 200) {
            bpmInstLocal = bpmCalculado;
            rates[rateSpot++] = (byte)bpmCalculado;
            rateSpot %= RATE_SIZE;

            int suma = 0;
            for (byte i = 0; i < RATE_SIZE; i++) suma += rates[i];
            bpmAvgLocal = suma / RATE_SIZE;

            long acRed = maxRed - minRed;
            long dcRed = (maxRed + minRed) / 2;
            long acIR = maxIR - minIR;
            long dcIR = (maxIR + minIR) / 2;

            if (dcRed > 0 && dcIR > 0 && acIR > 0) {
              // Aproximación empírica SpO2 = 110 - 25R (no calibrada clínicamente)
              float ratio = ((float)acRed / (float)dcRed) / ((float)acIR / (float)dcIR);
              int calcSpO2 = (int)(110.0 - (25.0 * ratio));
              if (calcSpO2 > 100) calcSpO2 = 99;
              if (calcSpO2 >= 85) spo2Local = calcSpO2;
            }

            minRed = 999999; maxRed = 0;
            minIR = 999999;  maxIR = 0;
            muestras = 0;
          }
        }
      } else {
        bpmInstLocal = 0.0f;
        bpmAvgLocal = 0;
        spo2Local = 0;
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

    // [v2] Publicación protegida; si el mutex está ocupado se reintenta
    //      en el siguiente ciclo (10 ms) sin bloquear la adquisición.
    if (xSemaphoreTake(mutexDatos, ESPERA_MUTEX) == pdTRUE) {
      g_datos.irValue = irLocal;
      g_datos.bpmInst = bpmInstLocal;
      g_datos.bpmAvg  = bpmAvgLocal;
      g_datos.spo2    = spo2Local;
      xSemaphoreGive(mutexDatos);
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 2: MPU6050 + algoritmo de caída (Core 1, 50 Hz)
// ==========================================
enum EstadoCaida { REPOSO, ESPERA_IMPACTO, VERIFICA_INMOVILIDAD };

void TareaMPU6050(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(20);

  // [v2] Máquina de estados: caída libre -> impacto -> inmovilidad
  EstadoCaida estado = REPOSO;
  uint16_t contCaidaLibre = 0;
  uint16_t contVentana = 0;
  uint16_t contQuietas = 0;
  bool     huboCaidaLibre = false;
  float    picoEvento = 0.0f;

  for (;;) {
    I2Cone.beginTransmission(MPU_ADDR);
    I2Cone.write(MPU_REG_ACCEL_XOUT_H);
    byte err = I2Cone.endTransmission(false);

    bool lecturaOk = false;
    float atotal = 1.0f;

    if (err == 0 && I2Cone.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true) == 6) {
      // [v2] Cada byte se lee en su propia sentencia: en la expresión
      //      read() << 8 | read() el orden de evaluación no está garantizado
      uint8_t b[6];
      for (uint8_t i = 0; i < 6; i++) b[i] = I2Cone.read();
      int16_t ax = (int16_t)((b[0] << 8) | b[1]);
      int16_t ay = (int16_t)((b[2] << 8) | b[3]);
      int16_t az = (int16_t)((b[4] << 8) | b[5]);

      float gx = ax / MPU_LSB_POR_G;
      float gy = ay / MPU_LSB_POR_G;
      float gz = az / MPU_LSB_POR_G;
      atotal = sqrtf(gx * gx + gy * gy + gz * gz);
      lecturaOk = true;
    }

    if (!lecturaOk) {
      configurarMPU6050();   // reintento de inicialización (p. ej. tras un corte)
      if (xSemaphoreTake(mutexDatos, ESPERA_MUTEX) == pdTRUE) {
        g_datos.erroresMPU++;
        xSemaphoreGive(mutexDatos);
      }
      vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
      continue;
    }

    // ---- Algoritmo cinemático de caída ----
    switch (estado) {
      case REPOSO:
        if (atotal < UMBRAL_CAIDA_LIBRE_G) {
          if (++contCaidaLibre >= MIN_MUESTRAS_CAIDA_LIBRE) {
            estado = ESPERA_IMPACTO;
            contVentana = 0;
          }
        } else {
          contCaidaLibre = 0;
          if (atotal > UMBRAL_IMPACTO_G) {          // impacto sin caída libre previa
            huboCaidaLibre = false;
            picoEvento = atotal;
            contVentana = 0;
            contQuietas = 0;
            estado = VERIFICA_INMOVILIDAD;
          }
        }
        break;

      case ESPERA_IMPACTO:
        if (atotal > UMBRAL_IMPACTO_G) {
          huboCaidaLibre = true;
          picoEvento = atotal;
          contVentana = 0;
          contQuietas = 0;
          estado = VERIFICA_INMOVILIDAD;
        } else if (++contVentana > VENTANA_IMPACTO_MUESTRAS) {
          estado = REPOSO;                           // caída libre sin impacto: se descarta
          contCaidaLibre = 0;
        }
        break;

      case VERIFICA_INMOVILIDAD:
        contVentana++;
        if (contVentana <= MUESTRAS_REBOTE) {
          if (atotal > picoEvento) picoEvento = atotal;   // el pico puede llegar en el rebote
        } else if (fabsf(atotal - 1.0f) < TOLERANCIA_INMOVIL_G) {
          contQuietas++;
        }

        if (contVentana >= MUESTRAS_REBOTE + VENTANA_INMOVIL_MUESTRAS) {
          EventoCaida ev;
          ev.tipo = (contQuietas >= MIN_MUESTRAS_QUIETAS) ? EVENTO_CAIDA_CONFIRMADA : EVENTO_IMPACTO;
          ev.huboCaidaLibre = huboCaidaLibre;
          ev.picoG = picoEvento;
          ev.t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

          if (xQueueSend(colaEventosCaida, &ev, 0) != pdTRUE) {
            if (xSemaphoreTake(mutexDatos, ESPERA_MUTEX) == pdTRUE) {
              g_datos.eventosCaidaPerdidos++;
              xSemaphoreGive(mutexDatos);
            }
          }
          if (ev.tipo == EVENTO_CAIDA_CONFIRMADA) {
            EventoLocal alarma = ALERTA_CAIDA;
            xQueueSend(colaEventosAcceso, &alarma, 0);
          }
          estado = REPOSO;
          contCaidaLibre = 0;
        }
        break;
    }

    if (xSemaphoreTake(mutexDatos, ESPERA_MUTEX) == pdTRUE) {
      g_datos.aTotal = atotal;
      if (atotal > g_datos.aPicoMax) g_datos.aPicoMax = atotal;
      xSemaphoreGive(mutexDatos);
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 3: DS18B20 (Core 1, 1 Hz)
// ==========================================
void TareaTemperatura(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000);

  // [v2] Se lanza la primera conversión y en cada ciclo se LEE la anterior
  //      (ya terminada: 750 ms a 12 bits < 1000 ms) y se lanza la siguiente.
  sensorTemp.requestTemperaturesByAddress(dirDS18B20);

  for (;;) {
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);

    float tC = sensorTemp.getTempC(dirDS18B20);
    sensorTemp.requestTemperaturesByAddress(dirDS18B20);

    // [v2] 85,0 °C es el valor de encendido del DS18B20 y -127 indica
    //      desconexión: ninguno se publica como dato clínico.
    bool valida = (tC != DEVICE_DISCONNECTED_C) && (tC != 85.0f) && (tC > -10.0f) && (tC < 60.0f);

    if (xSemaphoreTake(mutexDatos, ESPERA_MUTEX) == pdTRUE) {
      if (valida) g_datos.tempC = tC;
      else        g_datos.lecturasTempInvalidas++;
      xSemaphoreGive(mutexDatos);
    }
  }
}

// ==========================================
// TAREA 4: RFID-RC522 (Core 0, 150 ms)
// ==========================================
// [v2] Compara también el largo del UID (4, 7 o 10 bytes)
bool uidAutorizado(const MFRC522::Uid &uid) {
  if (uid.size != 4) return false;
  for (size_t k = 0; k < N_UIDS; k++) {
    if (memcmp(uid.uidByte, UIDS_AUTORIZADOS[k], 4) == 0) return true;
  }
  return false;
}

void TareaRFID(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(150);

  for (;;) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      EventoLocal evento = uidAutorizado(rfid.uid) ? ACCESO_VALIDO : ACCESO_INVALIDO;
      xQueueSend(colaEventosAcceso, &evento, 0);

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// TAREA 5: ACTUADORES LOCALES Y SESIÓN (Core 0)
// ==========================================
// [v2] Los patrones de LED/buzzer se reproducen sin vTaskDelay(): la tarea
//      espera eventos como máximo 10 ms y en cada vuelta avanza el patrón.
//      Un evento nuevo reemplaza al patrón en curso de inmediato.
//      Pasos pares = salida encendida, pasos impares = apagada (en ms).
const uint16_t PATRON_INICIO[]   = {150};
const uint16_t PATRON_CIERRE[]   = {80, 80, 80};
const uint16_t PATRON_DENEGADO[] = {100, 100, 100, 100, 100};
const uint16_t PATRON_CAIDA[]    = {300, 200, 300, 200, 300, 200, 300, 200, 300};

void TareaActuadores(void *pvParameters) {
  EventoLocal eventoRecibido;

  const uint16_t *patron = NULL;
  uint8_t  nPasos = 0;
  uint8_t  paso = 0;
  bool     patronUsaLedFail = false;
  TickType_t tInicioPaso = 0;

  for (;;) {
    if (xQueueReceive(colaEventosAcceso, &eventoRecibido, pdMS_TO_TICKS(10)) == pdTRUE) {
      switch (eventoRecibido) {
        case ACCESO_VALIDO:
          g_sesionActiva = !g_sesionActiva;   // Alternar estado de sesión
          digitalWrite(PIN_LED_OK, g_sesionActiva ? HIGH : LOW);
          patron = g_sesionActiva ? PATRON_INICIO : PATRON_CIERRE;
          nPasos = g_sesionActiva ? 1 : 3;
          patronUsaLedFail = false;
          break;

        case ACCESO_INVALIDO:
          // [v2] Una credencial inválida se rechaza pero NO cierra la sesión
          //      abierta: un tercero no puede detener el monitoreo del paciente.
          patron = PATRON_DENEGADO;
          nPasos = 5;
          patronUsaLedFail = true;
          break;

        case ALERTA_CAIDA:
          patron = PATRON_CAIDA;
          nPasos = 9;
          patronUsaLedFail = true;
          break;
      }
      paso = 0;
      tInicioPaso = xTaskGetTickCount();
      digitalWrite(PIN_BUZZER, HIGH);
      digitalWrite(PIN_LED_FAIL, patronUsaLedFail ? HIGH : LOW);
    }

    // Avance del patrón en curso
    if (patron != NULL && (xTaskGetTickCount() - tInicioPaso) >= pdMS_TO_TICKS(patron[paso])) {
      paso++;
      tInicioPaso = xTaskGetTickCount();
      if (paso >= nPasos) {
        patron = NULL;
        digitalWrite(PIN_BUZZER, LOW);
        digitalWrite(PIN_LED_FAIL, LOW);
      } else {
        bool encendido = (paso % 2 == 0);
        digitalWrite(PIN_BUZZER, encendido ? HIGH : LOW);
        digitalWrite(PIN_LED_FAIL, (encendido && patronUsaLedFail) ? HIGH : LOW);
      }
    }
  }
}

// ==========================================
// TAREA 6: TELEMETRÍA (Core 0, 1 Hz)
// ==========================================
void TareaTelemetria(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000);
  uint32_t ciclo = 0;

  for (;;) {
    // [v2] Las alarmas de caída se informan SIEMPRE, con o sin sesión
    EventoCaida ev;
    while (xQueueReceive(colaEventosCaida, &ev, 0) == pdTRUE) {
      Serial.printf("[ALERTA] %s | pico %.2f g | caída libre previa: %s | t = %lu ms\n",
                    ev.tipo == EVENTO_CAIDA_CONFIRMADA ? "CAIDA CONFIRMADA (impacto + inmovilidad)"
                                                       : "IMPACTO con movimiento posterior",
                    ev.picoG, ev.huboCaidaLibre ? "si" : "no", (unsigned long)ev.t_ms);
    }

    // [v2] Copia consistente de todos los datos en una sola sección crítica
    DatosClinicos d = {};
    bool copiaOk = false;
    if (xSemaphoreTake(mutexDatos, ESPERA_MUTEX) == pdTRUE) {
      d = g_datos;
      g_datos.aPicoMax = g_datos.aTotal;   // reinicia el pico de la ventana de 1 s
      xSemaphoreGive(mutexDatos);
      copiaOk = true;
    }

    if (g_sesionActiva && copiaOk) {
      char strBpm[30];
      if (d.bpmAvg > 0) {
        snprintf(strBpm, sizeof(strBpm), "%.0f BPM (Media: %d)", d.bpmInst, d.bpmAvg);
      } else {
        snprintf(strBpm, sizeof(strBpm), "Sin pulso detectado");
      }

      Serial.printf("[SESION ACTIVA] Temp: %.2f °C | Aceleracion: %.2f g (Pico: %.2f g) | Oximetro: [Dedo: %s | Ritmo: %s | Oxigeno: %d%%]\n",
                    d.tempC, d.aTotal, d.aPicoMax,
                    (d.irValue > 50000) ? "DETECTADO" : "NO DETECTADO",
                    strBpm, d.spo2);
    } else if (!g_sesionActiva) {
      Serial.println("[ACCESO BLOQUEADO] Sistema en reposo. Acerque credencial autorizada para iniciar monitoreo clinico...");
    }

    // [v2] Cada 10 s: diagnóstico de robustez (errores de bus y stack libre)
    if (++ciclo % 10 == 0 && copiaOk) {
      Serial.printf("[DIAG] Errores I2C MPU: %lu | Temp invalidas: %lu | Eventos perdidos: %lu | Heap libre: %u B\n",
                    (unsigned long)d.erroresMPU, (unsigned long)d.lecturasTempInvalidas,
                    (unsigned long)d.eventosCaidaPerdidos, (unsigned)ESP.getFreeHeap());
      Serial.printf("[DIAG] Stack libre (B) MAX:%u MPU:%u TEMP:%u RFID:%u ACT:%u TEL:%u\n",
                    (unsigned)uxTaskGetStackHighWaterMark(hMAX), (unsigned)uxTaskGetStackHighWaterMark(hMPU),
                    (unsigned)uxTaskGetStackHighWaterMark(hTemp), (unsigned)uxTaskGetStackHighWaterMark(hRFID),
                    (unsigned)uxTaskGetStackHighWaterMark(hActua), (unsigned)uxTaskGetStackHighWaterMark(hTelem));
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));   // [v2] espera de 1 s que cede la CPU (antes era bloqueante)

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
  rfid.PCD_Init();

  // [v2] Diagnóstico de arranque: cada sensor informa si respondió
  byte versionRC522 = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.printf("[INIT] RC522   : %s (VersionReg = 0x%02X)\n",
                (versionRC522 == 0x00 || versionRC522 == 0xFF) ? "FALLA" : "OK", versionRC522);

  sensorTemp.begin();
  bool dsOk = sensorTemp.getAddress(dirDS18B20, 0);
  if (dsOk) sensorTemp.setResolution(dirDS18B20, 12);
  sensorTemp.setWaitForConversion(false);
  Serial.printf("[INIT] DS18B20 : %s (%u dispositivo/s en el bus)\n",
                dsOk ? "OK" : "FALLA", sensorTemp.getDeviceCount());

  // MPU6050
  bool mpuOk = configurarMPU6050();
  I2Cone.beginTransmission(MPU_ADDR);
  I2Cone.write(MPU_REG_WHO_AM_I);
  I2Cone.endTransmission(false);
  uint8_t whoAmI = 0;
  if (I2Cone.requestFrom((uint8_t)MPU_ADDR, (size_t)1, true) == 1) whoAmI = I2Cone.read();
  Serial.printf("[INIT] MPU6050 : %s (WHO_AM_I = 0x%02X, rango ±8 g, 50 Hz)\n", mpuOk ? "OK" : "FALLA", whoAmI);

  // MAX30102: 400 sps con promedio de 4 -> 100 Hz efectivos (pulso 411 us, 18 bits)
  bool maxOk = sensorOptico.begin(Wire, I2C_SPEED_STANDARD);
  if (maxOk) {
    sensorOptico.setup(0x1F, 4, 2, 400, 411, 4096);
    sensorOptico.setPulseAmplitudeRed(0x1F);
    sensorOptico.setPulseAmplitudeIR(0x1F);
  }
  Serial.printf("[INIT] MAX30102: %s\n", maxOk ? "OK" : "FALLA");

  colaEventosAcceso = xQueueCreate(5, sizeof(EventoLocal));
  colaEventosCaida  = xQueueCreate(8, sizeof(EventoCaida));
  mutexDatos        = xSemaphoreCreateMutex();

  if (colaEventosAcceso == NULL || colaEventosCaida == NULL || mutexDatos == NULL) {
    Serial.println("[ERROR] No hay memoria para colas/mutex. Sistema detenido.");
    return;
  }

  // [v2] Distribución por núcleo:
  //  Core 1 (APP_CPU): adquisición biomédica, aislada de la futura pila Wi-Fi.
  //  Core 0 (PRO_CPU): interfaz local y comunicación (aquí correrá Wi-Fi/JWT).
  //  En el Core 1 el MPU6050 tiene la mayor prioridad porque no usa FIFO:
  //  una muestra que no se lee a tiempo se pierde. El MAX30102 tolera
  //  retrasos gracias a su FIFO de 32 muestras (320 ms a 100 Hz).
  xTaskCreatePinnedToCore(TareaMPU6050,     "Task_MPU",   1024 * 3, NULL, 4, &hMPU,   1);
  xTaskCreatePinnedToCore(TareaMAX30102,    "Task_MAX",   1024 * 4, NULL, 3, &hMAX,   1);
  xTaskCreatePinnedToCore(TareaTemperatura, "Task_Temp",  1024 * 3, NULL, 1, &hTemp,  1);

  xTaskCreatePinnedToCore(TareaActuadores,  "Task_Actua", 1024 * 3, NULL, 3, &hActua, 0);
  xTaskCreatePinnedToCore(TareaRFID,        "Task_RFID",  1024 * 4, NULL, 2, &hRFID,  0);
  xTaskCreatePinnedToCore(TareaTelemetria,  "Task_Telem", 1024 * 4, NULL, 1, &hTelem, 0);
}

void loop() {
  vTaskDelete(NULL);   // la tarea de Arduino no se usa: todo corre en FreeRTOS
}
