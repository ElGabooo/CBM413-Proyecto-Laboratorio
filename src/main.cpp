// ============================================================================
// INCLUSIÓN DE LIBRERÍAS DE CONTROL, COMUNICACIÓN Y SENSORES BIOMÉDICOS
// ============================================================================
#include <Arduino.h>           // Núcleo base de funciones del ecosistema Arduino / ESP-IDF
#include <Wire.h>              // Controlador para el bus de comunicación I2C por hardware
#include <SPI.h>               // Controlador para el bus sincrónico SPI de alta velocidad
#include <MFRC522.h>           // Driver del transceptor RFID-RC522 (frecuencia ISO/IEC 14443A a 13.56 MHz)
#include <OneWire.h>           // Protocolo de comunicación de un solo hilo (OneWire de Dallas/Maxim)
#include <DallasTemperature.h> // Funciones de conversión digital para el sensor DS18B20
#include "MAX30105.h"          // Driver de bajo nivel para pulsioximetría (compatible con MAX30102)
#include "heartRate.h"         // Algoritmo de detección de picos sistólicos (fotopletismografía)

// ============================================================================
// ASIGNACIÓN DE PINES FÍSICOS (GPIOs) DEL MICROCONTROLADOR ESP32
// ============================================================================
// Actuadores locales de respuesta clínica
#define PIN_LED_OK        12  // Salida digital: LED activo (Indica sesión autorizada y activa)
#define PIN_LED_FAIL      14  // Salida digital: LED denegado (Alarma de acceso denegado o evento adverso)
#define PIN_BUZZER        15  // Salida digital: Buzzer piezoeléctrico para confirmaciones acústicas

// Bus OneWire para termometría
#define ONE_WIRE_BUS      4   // Pin de datos DQ para el sensor DS18B20 (requiere pull-up de 4.7k a 3.3V)

// Bus I2C Primario (Wire - Instancia física 0): Dedicado exclusivamente al MAX30102
#define I2C0_SDA          21  // Línea SDA (Datos bidireccionales del I2C #0)
#define I2C0_SCL          22  // Línea SCL (Reloj de sincronismo del I2C #0)

// Bus I2C Secundario (I2Cone - Instancia física 1): Dedicado a la IMU MPU6050
#define I2C1_SDA          32  // Línea SDA independiente para evitar saturación de bus y colisiones
#define I2C1_SCL          33  // Línea SCL independiente para el bus secundario
#define MPU_ADDR          0x68 // Dirección hexadecimal I2C por defecto del MPU6050 (pin AD0 a GND)

// Bus SPI nativo: Dedicado al lector de credenciales RFID
#define RC522_SS_PIN      5   // Chip Select / Slave Select (CS/SS) del bus SPI
#define RC522_RST_PIN     27  // Pin de reset por hardware para reanudar el circuito del RC522

// ============================================================================
// INSTANCIACIÓN DE DISPOSITIVOS Y VARIABLES GLOBALES DE TELEMETRÍA
// ============================================================================
OneWire oneWire(ONE_WIRE_BUS);              // Inicializa el canal físico OneWire sobre el GPIO 4
DallasTemperature sensorTemp(&oneWire);     // Pasa la referencia del canal al decodificador de temperatura
MAX30105 sensorOptico;                      // Objeto de control para el pulsioxímetro MAX30102
MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);  // Objeto de control para el módulo RFID bajo SPI
TwoWire I2Cone = TwoWire(1);                // Crea una segunda interfaz I2C por hardware en el puerto 1 del ESP32

// Primitivas de sincronización inter-tarea de FreeRTOS
QueueHandle_t colaEventosAcceso;             // Cola de mensajes para transferir eventos desde RFID a Actuadores
enum TipoAcceso { ACCESO_VALIDO, ACCESO_INVALIDO }; // Tipos de eventos admitidos por la cola de seguridad
const byte UID_AUTORIZADO[4] = {0x61, 0x2A, 0x88, 0x17}; // UID del tag RFID permitido (tarjeta de personal autorizado)

// Variables de estado global con calificador 'volatile' (evita que el compilador las optimice en registros,
// forzando la lectura directa desde la memoria RAM entre distintos núcleos y tareas)
volatile bool g_sesionActiva = false;       // Bandera de control: true = sistema desbloqueado y midiendo
volatile float g_tempC = 0.0;               // Almacena la última temperatura válida en °C
volatile float g_aTotal = 1.0;              // Módulo vectorial de aceleración instantánea en 'g' (1.0g en reposo)
volatile float g_aPicoMax = 1.0;            // Retiene el pico más alto de aceleración en una ventana de tiempo
volatile bool  g_alertaImpacto = false;     // Bandera de impacto mecánico/caída libre detectada
volatile long  g_irValue = 0;               // Amplitud cruda del canal Infrarrojo (determina contacto con la piel)
volatile float g_bpmInst = 0.0;             // Frecuencia cardíaca instantánea calculada entre latidos
volatile int   g_bpmAvg = 0;                // Frecuencia cardíaca suavizada por filtro de promedio móvil
volatile int   g_spo2 = 0;                  // Estimación de la saturación periférica de oxígeno en porcentaje

// ============================================================================
// TAREA 1: ADQUISICIÓN ÓPTICA MAX30102 (Core 0 - Prioridad Alta: 3 - Periodo: 10 ms)
// ============================================================================
void TareaMAX30102(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount(); // Guarda la marca de tiempo base para determinismo temporal
  const TickType_t xPeriodo = pdMS_TO_TICKS(10);  // Fija el periodo de muestreo en 10 ms (frecuencia = 100 Hz)

  long lastBeat = 0;                              // Almacena el timestamp (ms) del último latido para delta-T
  const byte RATE_SIZE = 4;                       // Ventana de 4 muestras para el promedio de pulsaciones
  byte rates[RATE_SIZE] = {0};                    // Buffer circular del filtro de promedio móvil
  byte rateSpot = 0;                              // Índice circular de inserción de muestras

  // Variables para extraer los componentes AC (pico a pico) y DC (nivel medio) de la onda fotopletismográfica
  long minRed = 999999, maxRed = 0;
  long minIR = 999999, maxIR = 0;
  int muestras = 0;

  for (;;) {
    sensorOptico.check(); // Consulta los registros de interrupción del sensor y traslada datos a su buffer local

    // Procesa todas las muestras acumuladas en la memoria FIFO del MAX30102
    while (sensorOptico.available()) {
      long irVal = sensorOptico.getFIFOIR();   // Extrae la muestra cruda del canal Infrarrojo (940 nm)
      long redVal = sensorOptico.getFIFORed(); // Extrae la muestra cruda del canal Rojo (660 nm)
      sensorOptico.nextSample();               // Avanza el puntero de lectura de la FIFO interna

      g_irValue = irVal; // Publica la señal IR en memoria global para telemetría

      // Umbrales para discriminación de presencia tisular (dedo puesto vs. sensor al aire)
      if (irVal > 50000 && irVal < 250000) {
        // Seguimiento dinámico de crestas y valles para aislar la componente pulsátil
        if (redVal < minRed) minRed = redVal;
        if (redVal > maxRed) maxRed = redVal;
        if (irVal < minIR) minIR = irVal;
        if (irVal > maxIR) maxIR = irVal;
        muestras++;

        // Algoritmo por derivada que detecta el cruce de umbral del pulso sistólico
        if (checkForBeat(irVal)) {
          long delta = millis() - lastBeat; // Calcula el periodo interlatido en milisegundos
          lastBeat = millis();

          float bpmCalculado = 60.0 / (delta / 1000.0); // Conversión matemática de periodo a pulsos por minuto (BPM)

          // Filtro de validez fisiológica: descarta artefactos de movimiento fuera de rangos humanos creíbles
          if (bpmCalculado >= 45 && bpmCalculado <= 200) {
            g_bpmInst = bpmCalculado;
            rates[rateSpot++] = (byte)bpmCalculado; // Inserta el dato en el buffer circular
            rateSpot %= RATE_SIZE;                  // Retorna el índice a 0 cuando alcanza el límite

            // Calcula el promedio móvil de BPM para atenuar variaciones bruscas
            int suma = 0;
            for (byte i = 0; i < RATE_SIZE; i++) suma += rates[i];
            g_bpmAvg = suma / RATE_SIZE;

            // Descomposición fotopletismográfica en componentes alternas (AC) y continuas (DC)
            long acRed = maxRed - minRed;          // Amplitud pulsátil de la luz roja absorbida por desoxihemoglobina
            long dcRed = (maxRed + minRed) / 2;    // Nivel base tisular estático para luz roja
            long acIR = maxIR - minIR;             // Amplitud pulsátil de la luz IR absorbida por oxihemoglobina
            long dcIR = (maxIR + minIR) / 2;       // Nivel base tisular estático para luz infrarroja

            // Estimación de oximetría por principio de doble absorción óptica
            if (dcRed > 0 && dcIR > 0 && acIR > 0) {
              // Razón de modulación óptica (Ratio of Ratios)
              float ratio = ((float)acRed / (float)dcRed) / ((float)acIR / (float)dcIR);
              // Curva de calibración empírica estándar para SpO2
              int calcSpO2 = (int)(110.0 - (25.0 * ratio));
              if (calcSpO2 > 100) calcSpO2 = 99;   // Limita el techo fisiológico al 99%
              if (calcSpO2 >= 85) g_spo2 = calcSpO2; // Filtra valores anómalos o subfisiológicos por desprendimiento
            }

            // Reinicia los acumuladores para evaluar el siguiente ciclo cardíaco
            minRed = 999999; maxRed = 0;
            minIR = 999999;  maxIR = 0;
            muestras = 0;
          }
        }
      } else {
        // En ausencia de dedo, limpia los registros biomédicos a cero inmediatamente
        g_bpmInst = 0.0;
        g_bpmAvg = 0;
        g_spo2 = 0;
        minRed = 999999; maxRed = 0;
        minIR = 999999;  maxIR = 0;
        muestras = 0;
      }

      // Previene desbordamientos si transcurren muchas muestras sin registrar un pulso válido
      if (muestras > 250) {
        minRed = 999999; maxRed = 0;
        minIR = 999999;  maxIR = 0;
        muestras = 0;
      }
    }

    // Suspende la tarea de forma no bloqueante asegurando una periodicidad estricta de 10 ms
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ============================================================================
// TAREA 2: ADQUISICIÓN CINEMÁTICA MPU6050 (Core 0 - Prioridad Media: 2 - Periodo: 20 ms)
// ============================================================================
void TareaMPU6050(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(20); // Muestreo cinemático a 50 Hz

  for (;;) {
    I2Cone.beginTransmission(MPU_ADDR); // Apunta al bus I2C secundario y dirección esclava 0x68
    I2Cone.write(0x3B);                 // Escribe la dirección del primer registro de aceleración (ACCEL_XOUT_H)
    byte err = I2Cone.endTransmission(false); // Envía restart para mantener control exclusivo del bus sin soltar la línea

    if (err == 0) { // Si el sensor respondió con ACK (comunicación exitosa sin fallas en el cableado)
      byte len = I2Cone.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true); // Solicita 6 bytes (X, Y, Z en 16 bits cada uno)
      if (len == 6) {
        // Reconstrucción de enteros con signo de 16 bits combinando High Byte y Low Byte
        int16_t ax = I2Cone.read() << 8 | I2Cone.read();
        int16_t ay = I2Cone.read() << 8 | I2Cone.read();
        int16_t az = I2Cone.read() << 8 | I2Cone.read();

        // Conversión a unidades de gravedad 'g' (Escala ±2g: sensibilidad = 16384 LSB/g)
        // y cálculo de la magnitud resultante del vector tridimensional (Norma Euclidiana)
        float atotal = sqrt(pow(ax / 16384.0, 2) + pow(ay / 16384.0, 2) + pow(az / 16384.0, 2));
        g_aTotal = atotal; // En reposo estático, atotal oscila cerca de 1.0g

        // Actualiza el valor pico máximo observado
        if (atotal > g_aPicoMax) g_aPicoMax = atotal;

        // Detección de impacto severo compatible con caída de un paciente
        if (atotal > 2.30) {
          g_alertaImpacto = true; // Activa bandera crítica para la telemetría
        }
      }
    } else {
      // Rutina de recuperación ante pérdida de bus o desconexión física
      I2Cone.beginTransmission(MPU_ADDR);
      I2Cone.write(0x6B); // Registro PWR_MGMT_1 (Gestión de energía del MPU6050)
      I2Cone.write(0x00); // Escribe 0x00 para despertar al integrado si cayó en modo SLEEP profundo
      I2Cone.endTransmission();
    }

    // Espera no bloqueante hasta el siguiente ciclo de 20 ms
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ============================================================================
// TAREA 3: TERMOMETRÍA DIGITAL DS18B20 (Core 0 - Prioridad Baja: 1 - Periodo: 1000 ms)
// ============================================================================
void TareaTemperatura(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000); // Muestreo térmico a 1 Hz (la temperatura corporal cambia lento)

  for (;;) {
    sensorTemp.requestTemperatures(); // Emite la orden digital de conversión OneWire a los sensores del bus
    float tC = sensorTemp.getTempCByIndex(0); // Lee la temperatura del primer dispositivo hallado en el bus
    
    // Filtro de validación: DEVICE_DISCONNECTED_C (-127°C) indica falla de cableado o desconexión física
    if (tC != DEVICE_DISCONNECTED_C) {
      g_tempC = tC; // Guarda únicamente datos válidos en la variable de telemetría
    }
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo); // Cede la CPU durante 1 segundo
  }
}

// ============================================================================
// TAREA 4: IDENTIFICACIÓN RFID RC522 (Core 1 - Prioridad Media: 2 - Periodo: 150 ms)
// ============================================================================
void TareaRFID(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(150); // Polling del transceptor RFID cada 150 ms

  for (;;) {
    // Comprueba si hay una tarjeta pasiva en el campo de 13.56 MHz y si pudo leer su número de serie
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      bool autorizado = true;
      // Comparación byte a byte del UID detectado contra la clave de acceso clínico
      for (byte i = 0; i < 4; i++) {
        if (rfid.uid.uidByte[i] != UID_AUTORIZADO[i]) autorizado = false;
      }

      // Clasifica el resultado en la enumeración de acceso
      TipoAcceso evento = autorizado ? ACCESO_VALIDO : ACCESO_INVALIDO;
      
      // Envía el evento a la cola de FreeRTOS sin tiempo de bloqueo (tiempo de espera = 0 ticks)
      xQueueSend(colaEventosAcceso, &evento, 0);

      // Comandos de fin de transacción según el estándar ISO/IEC 14443A
      rfid.PICC_HaltA();       // Ordena a la tarjeta entrar en estado de reposo (evita relecturas parásitas)
      rfid.PCD_StopCrypto1();  // Desactiva el cifrado por hardware Crypto1 del transceptor RC522
    }
    vTaskDelayUntil(&xUltimoTiempo, xPeriodo);
  }
}

// ============================================================================
// TAREA 5: ACTUADORES LOCALES Y SESIÓN (Core 1 - Prioridad Baja: 1 - Guiada por Eventos)
// ============================================================================
void TareaActuadores(void *pvParameters) {
  TipoAcceso eventoRecibido;

  for (;;) {
    // Modo de bajo consumo: la tarea se suspende indefinidamente (portMAX_DELAY) hasta que entra un evento a la cola
    if (xQueueReceive(colaEventosAcceso, &eventoRecibido, portMAX_DELAY) == pdTRUE) {
      if (eventoRecibido == ACCESO_VALIDO) {
        g_sesionActiva = !g_sesionActiva; // Conmuta el estado de la sesión (abrir/cerrar monitoreo)

        if (g_sesionActiva) {
          // Sesión iniciada: LED activo encendido fijo y un pitido corto de confirmación
          digitalWrite(PIN_LED_OK, HIGH);
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(150)); // Pausa no bloqueante para conformar el pulso sonoro
          digitalWrite(PIN_BUZZER, LOW);
        } else {
          // Sesión cerrada: LED activo se apaga y emite dos pulsos acústicos breves
          digitalWrite(PIN_LED_OK, LOW);
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(PIN_BUZZER, LOW);
          vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(PIN_BUZZER, HIGH);
          vTaskDelay(pdMS_TO_TICKS(80));
          digitalWrite(PIN_BUZZER, LOW);
        }
      } else {
        // Tag inválido o denegado: cierre forzoso inmediato de sesión por seguridad clínica
        g_sesionActiva = false;
        digitalWrite(PIN_LED_OK, LOW);

        // Dispara ráfaga de alarma visual y sonora (3 destellos rojos sincronizados con el buzzer)
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

// ============================================================================
// TAREA 6: TELEMETRÍA CONDICIONADA (Core 1 - Prioridad Baja: 1 - Periodo: 1000 ms)
// ============================================================================
void TareaTelemetria(void *pvParameters) {
  TickType_t xUltimoTiempo = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000); // Emite telemetría estructurada cada 1 segundo

  for (;;) {
    // Control de privacidad: solo transmite constantes vitales si existe una sesión autorizada activa
    if (g_sesionActiva) {
      char strCaida[25];
      // Evaluación de la bandera cinemática de caída
      if (g_alertaImpacto) {
        snprintf(strCaida, sizeof(strCaida), "ALERTA (IMPACTO)");
        g_alertaImpacto = false; // Restablece la bandera tras reportar el incidente
      } else {
        snprintf(strCaida, sizeof(strCaida), "Estable");
      }

      char strBpm[30];
      if (g_bpmAvg > 0) {
        snprintf(strBpm, sizeof(strBpm), "%.0f BPM (Media: %d)", g_bpmInst, g_bpmAvg);
      } else {
        snprintf(strBpm, sizeof(strBpm), "Sin pulso detectado");
      }

      // Impresión formateada con contexto clínico completo (valor, unidad, diagnóstico y validación)
      Serial.printf("[SESION ACTIVA] Temp: %.2f °C | Aceleracion: %.2f g (Pico: %.2f g) [%s] | Oximetro: [Dedo: %s | Ritmo: %s | Oxigeno: %d%%]\n",
                    g_tempC,
                    g_aTotal,
                    g_aPicoMax,
                    strCaida,
                    (g_irValue > 50000) ? "DETECTADO" : "NO DETECTADO",
                    strBpm,
                    g_spo2);

      g_aPicoMax = g_aTotal; // Restablece el valor pico para el siguiente intervalo de 1 s
    } else {
      // Estado de bloqueo seguro mientras espera autenticación
      Serial.println("[ACCESO BLOQUEADO] Sistema en reposo. Acerque credencial autorizada para iniciar monitoreo clinico...");
    }

    vTaskDelayUntil(&xUltimoTiempo, xPeriodo); // Pausa no bloqueante hasta el próximo segundo
  }
}

// ============================================================================
// FUNCIÓN DE INICIALIZACIÓN GENERAL (SETUP)
// ============================================================================
void setup() {
  Serial.begin(115200); // Configura la interfaz serial a alta velocidad para depuración
  delay(1000);          // Breve retraso de estabilización eléctrica para el puerto serie

  // Configuración de los pines de salida de actuadores
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_FAIL, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_FAIL, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // Inicialización del Bus I2C #0 (Wire) asignado al MAX30102 con timeout preventivo de 50 ms
  Wire.begin(I2C0_SDA, I2C0_SCL);
  Wire.setTimeOut(50);

  // Inicialización del Bus I2C #1 (I2Cone) asignado al MPU6050 con timeout preventivo
  I2Cone.begin(I2C1_SDA, I2C1_SCL);
  I2Cone.setTimeOut(50);

  // Inicialización de buses periféricos SPI y OneWire
  SPI.begin();
  sensorTemp.begin();
  // Configura la librería de temperatura en modo asíncrono para no bloquear la CPU durante la conversión de 750 ms
  sensorTemp.setWaitForConversion(false);
  rfid.PCD_Init(); // Inicializa el transceptor de radiofrecuencia RC522

  // Configuración de registros del sensor cinemático MPU6050
  I2Cone.beginTransmission(MPU_ADDR);
  I2Cone.write(0x6B); // Registro de gestión energética PWR_MGMT_1
  I2Cone.write(0x00); // Pone a 0 el bit de SLEEP para encender los osciladores internos del MPU6050
  I2Cone.endTransmission();

  // Inicialización y configuración del oxímetro MAX30102 en el Bus I2C #0
  if (sensorOptico.begin(Wire, I2C_SPEED_STANDARD)) {
    // Parámetros: Potencia LED (0x1F), Muestra promedio (4), Modo LED Rojo+IR (2), Muestreo (100 Hz), Ancho pulso (411 us), Rango ADC (4096)
    sensorOptico.setup(0x1F, 4, 2, 100, 411, 4096);
    sensorOptico.setPulseAmplitudeRed(0x1F); // Amplitud de corriente para el emisor Rojo
    sensorOptico.setPulseAmplitudeIR(0x1F);  // Amplitud de corriente para el emisor Infrarrojo
  }

  // Creación de la cola de eventos de seguridad (capacidad para 5 elementos tipo TipoAcceso)
  colaEventosAcceso = xQueueCreate(5, sizeof(TipoAcceso));

  if (colaEventosAcceso != NULL) {
    // Tareas fijadas en Core 0: Adquisición de señales analógicas y buses de alta frecuencia
    // Argumentos: Función, Nombre, Memoria Stack (bytes), Parámetros, Prioridad, Puntero de control, Núcleo
    xTaskCreatePinnedToCore(TareaMAX30102,    "Task_MAX",    1024 * 4, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TareaMPU6050,     "Task_MPU",    1024 * 3, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TareaTemperatura, "Task_Temp",   1024 * 4, NULL, 1, NULL, 0);

    // Tareas fijadas en Core 1: Lógica de autenticación, control de actuadores y telemetría de salida
    xTaskCreatePinnedToCore(TareaRFID,        "Task_RFID",   1024 * 4, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TareaActuadores,  "Task_Actua",  1024 * 3, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TareaTelemetria,  "Task_Telem",  1024 * 3, NULL, 1, NULL, 1);
  }
}

// ============================================================================
// BUCLE PRINCIPAL (LOOP)
// ============================================================================
void loop() {
  // En arquitecturas RTOS puro, la tarea nativa 'loop' es innecesaria.
  // vTaskDelete(NULL) elimina la tarea actual del planificador, liberando su memoria de stack asociada.
  vTaskDelete(NULL);
}
