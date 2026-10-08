# Firmware v2 — qué cambió y por qué (CBM413, entrega inicial)

Compilado con Arduino-ESP32 2.0.17 (placa "ESP32 Dev Module"): flash 299 609 B (22,9 %), RAM estática 22 204 B (6,8 %), sin warnings.
Librerías: MFRC522 1.4.x, OneWire 2.3.8, DallasTemperature 4.0.x, SparkFun MAX3010x 1.1.2.

Todas las líneas modificadas llevan la marca `[v2]` en el código.

| # | Cambio | Problema en la versión del avance | Criterio de la rúbrica |
|---|--------|-----------------------------------|------------------------|
| 1 | `delay(1000)` en `setup()` → `vTaskDelay()` | La rúbrica prohíbe cualquier `delay()` | Firmware no bloqueante |
| 2 | Variables compartidas agrupadas en `DatosClinicos g_datos` + `mutexDatos` (espera máx. 2 ms) | `g_aPicoMax` se escribía desde los dos núcleos sin exclusión mutua; `volatile` no hace atómico un leer-modificar-escribir | Firmware no bloqueante / IA (arquitectura FreeRTOS) |
| 3 | Bandera `g_alertaImpacto` → `colaEventosCaida` | Si llegaba un impacto entre que Telemetría leía la bandera y la ponía en `false`, el evento se perdía | Firmware no bloqueante |
| 4 | MPU6050 en ±8 g (ACCEL_CONFIG 0x1C = 0x10), DLPF 21 Hz (0x1A = 0x04), 50 Hz (0x19 = 19) | Con ±2 g por defecto cada eje se satura en 2 g: el umbral de 2,3 g casi nunca se alcanzaba | Adquisición / demostración |
| 5 | Bytes del MPU leídos uno por uno | En `read() << 8 \| read()` el orden de evaluación no está garantizado en C++ | Adquisición |
| 6 | Algoritmo de caída de 3 fases (caída libre → impacto → inmovilidad) | Solo había umbral de impacto | Demostración / IA |
| 7 | MAX30102 a 400 sps con promedio 4 (100 Hz) y latidos medidos contando muestras | Con 100 sps/4 = 25 Hz la resolución era ±3,75 lpm y `millis()` agregaba jitter | Firmware no bloqueante (jitter) |
| 8 | DS18B20: dirección ROM leída una vez, se lee la conversión anterior y se lanza la siguiente; se descartan 85 °C y −127 °C | La primera lectura era 85 °C (valor de encendido) | Adquisición |
| 9 | RFID: lista de UIDs, compara largo del UID; una tarjeta inválida ya **no** cierra la sesión | Cualquier tarjeta ajena podía detener el monitoreo | Autenticación local |
| 10 | Actuadores sin `vTaskDelay` dentro de los patrones (espera de cola con timeout de 10 ms) | Un patrón de 600 ms retrasaba el siguiente evento | Autenticación local y actuadores |
| 11 | Adquisición en Core 1, interfaz/comunicación en Core 0; MPU con prioridad 4 | La pila Wi-Fi corre en Core 0 (`CONFIG_ESP32_WIFI_TASK_PINNED_TO_CORE_0=y`) | Arquitectura (núcleos correspondientes) |
| 12 | Diagnóstico: estado de cada sensor al arrancar y cada 10 s errores I2C, lecturas inválidas, eventos perdidos y stack libre | No había evidencia de "sin pérdida de tramas" | Demostración |

## Antes de entregar
- **Uso regulado de IA:** reescriban los comentarios `[v2]` con sus propias palabras (la rúbrica pide "comentarios exhaustivos propios") y asegúrense de que cada integrante pueda explicar los cambios 2, 3, 6 y 10.
- **GitHub:** suban los cambios en commits separados (uno por fila de la tabla), repartidos entre los integrantes, en una rama y con pull request hacia `main` protegida.
- **Pruebas en banco** para completar las partes en amarillo del informe: copiar 1–2 min del monitor serie con los cuatro sensores, la línea `[DIAG]`, una caída simulada sobre un colchón y la respuesta a tarjeta válida/inválida.
- Ajustar los umbrales (`UMBRAL_*`) con sus propias pruebas y anotar dónde va puesto el MPU6050 en el cuerpo.
