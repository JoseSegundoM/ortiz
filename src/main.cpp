#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Pines de configuración de hardware
#define ONE_WIRE_BUS 19
#define SOLENOID_PIN 12 // Salida digital → relé/MOSFET → electroválvula
#define MIXER_PIN 13    // Salida digital → mezclador (agitador) continuo
// Parámetro de retardo de tiempo (d = t0 / Ts)
// Tiempo muerto físico calculado: 21.94 segundos.
// Período de muestreo (Ts) = 1.0 segundo.
// Sistema: tanque 20L, tubo 20", solenoide 1/4" (C_VL=0.20)
#define BUFFER_SIZE 22

// Ventana de salida time-proporcional (TPO)
// U_output (0.0-1.0) se convierte en duty cycle sobre esta ventana.
// TPO_WINDOW = 33 s << tau = 499.5 s -> el proceso ve el caudal promedio
#define TPO_WINDOW 33

// Parámetros del modelo de planta en tiempo discreto (G(s) = K / (tau*s + 1))
// Ts = 1.0 s, tau = 499.54 s, K = -0.3270
// Discretización ZOH: a = exp(-Ts/tau), b = K*(1-a)
const float MODEL_A = 0.99800f;
const float MODEL_B = -0.00065f;

// Parámetros de sintonización del controlador PI
// Tc = 2.2 s (≈ t0/10) -> Kc = tau/(K*Tc) = -696.4617
// Ti = tau = 499.54 s        -> Ki = Kc/Ti = -1.394201
const float PI_KP = -696.4617f;
const float PI_KI = -1.394201f;
const float TS_SEC = 1.0f;

// Funciones auxiliares de conversión (70.0–200.0 °F mapeadas a 0.0–1.0)
inline float fToNormalized(float tempF) { return (tempF - 70.0f) / 130.0f; }
inline float normalizedToF(float norm) { return (norm * 130.0f) + 70.0f; }

// Variables compartidas entre tareas (protegidas por stateMutex)
// U_output representa el duty cycle equivalente (0.0 = siempre cerrada, 1.0 =
// siempre abierta)
volatile float T_actual = 80.0f;   // Temperatura actual en Fahrenheit
volatile float T_setpoint = 90.0f; // Temperatura objetivo en Fahrenheit (90 °F)
volatile float U_output =
    0.149f; // Duty cycle de la electroválvula (0.0–1.0) (Vp_ss nominal = 0.149)
SemaphoreHandle_t stateMutex = NULL;

DeviceAddress sensorAddress;
bool sensorAddressValid = false;

// Instanciaciones
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Declaraciones de tareas
void TaskSensorRead(void *pvParameters);
void TaskSmithController(void *pvParameters);
void TaskActuatorWrite(void *pvParameters);
void TaskSetpointInput(void *pvParameters);

void setup() {
  Serial.begin(115200);

  // Inicializar sensor DS18B20 con pull-up interno habilitado
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  sensors.begin();
  sensors.setWaitForConversion(
      false); // No bloquear el hilo durante la conversión ADC

  sensorAddressValid =
      sensors.getDeviceCount() > 0 && sensors.getAddress(sensorAddress, 0);
  if (sensorAddressValid) {
    sensors.setResolution(sensorAddress,
                          12); // 12 bits → 0.0625 °C, tiempo conv. ~750 ms
    Serial.println("Sensor DS18B20 detectado e inicializado correctamente.");
  } else {
    Serial.println(
        "Advertencia: No se detectó el sensor DS18B20 en el inicio.");
  }

  // Inicializar salida digital para la electroválvula (a través de relé o
  // MOSFET) Válvula arranca CERRADA — estado seguro (Normally Open -> HIGH para
  // cerrar)
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, HIGH);

  // Inicializar salida digital para el mezclador y mantenerlo encendido (HIGH)
  pinMode(MIXER_PIN, OUTPUT);
  digitalWrite(MIXER_PIN, HIGH);

  // Mutex único para T_actual, T_setpoint y U_output
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    Serial.println("Error: no se pudo crear el mutex de estado");
    while (true) {
      delay(1000);
    }
  }

  // Solicitud inicial para que el primer ciclo de TaskSensorRead encuentre
  // datos listos
  sensors.requestTemperatures();

  // Crear tareas
  if (xTaskCreate(TaskSensorRead, "Sensor", 2048, NULL, 3, NULL) != pdPASS) {
    Serial.println("Error: no se pudo crear tarea Sensor");
    while (true) {
      delay(1000);
    }
  }
  if (xTaskCreate(TaskSmithController, "Smith", 3072, NULL, 2, NULL) !=
      pdPASS) {
    Serial.println("Error: no se pudo crear tarea Smith");
    while (true) {
      delay(1000);
    }
  }
  if (xTaskCreate(TaskActuatorWrite, "Actuator", 1536, NULL, 1, NULL) !=
      pdPASS) {
    Serial.println("Error: no se pudo crear tarea Actuator");
    while (true) {
      delay(1000);
    }
  }
  if (xTaskCreate(TaskSetpointInput, "Setpoint", 2048, NULL, 1, NULL) !=
      pdPASS) {
    Serial.println("Error: no se pudo crear tarea Setpoint");
    while (true) {
      delay(1000);
    }
  }
}

void loop() { vTaskDelay(portMAX_DELAY); }

/*---------------------------------------------------------------------------------*/
/*------------------------------ Implementación de tareas
 * -------------------------*/
/*---------------------------------------------------------------------------------*/

void TaskSensorRead(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  bool lastSensorState =
      false; // Para detectar cambios en el estado de conexión

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    // Intentar re-detectar el sensor si no es válido
    if (!sensorAddressValid) {
      sensors.begin(); // Re-escanear el bus OneWire
      sensorAddressValid =
          sensors.getDeviceCount() > 0 && sensors.getAddress(sensorAddress, 0);
      if (sensorAddressValid) {
        sensors.setResolution(sensorAddress, 12);
        Serial.println("Sensor DS18B20 detectado de forma dinámica.");
      }
    }

    float rawCelsius = DEVICE_DISCONNECTED_C;
    if (sensorAddressValid) {
      rawCelsius = sensors.getTempC(sensorAddress);
    }

    if (rawCelsius != DEVICE_DISCONNECTED_C) {
      if (!lastSensorState) {
        Serial.println("Sensor DS18B20 conectado y leyendo datos.");
        lastSensorState = true;
      }

      xSemaphoreTake(stateMutex, portMAX_DELAY);
      T_actual = (rawCelsius * 1.8f) + 32.0f;
      xSemaphoreGive(stateMutex);
    } else {
      if (lastSensorState || sensorAddressValid) {
        Serial.println(
            "Error: Sensor DS18B20 desconectado o error de lectura.");
        lastSensorState = false;
        sensorAddressValid = false; // Forzar re-detección en el próximo ciclo
      }
    }

    sensors.requestTemperatures(); // Resultado disponible en el próximo ciclo
                                   // (~750 ms)
  }
}

void TaskSmithController(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);

  // Esperar un momento para asegurar que TaskSensorRead haya hecho la primera
  // lectura válida
  vTaskDelay(pdMS_TO_TICKS(1500));

  // Inicializar dinámicamente con la temperatura real inicial
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  float initial_temp_F = T_actual;
  xSemaphoreGive(stateMutex);

  float initial_norm = fToNormalized(initial_temp_F);
  float T_op_norm = fToNormalized(90.0f); // Punto de operación nominal (90 °F)
  float T_predicted_dev =
      initial_norm - T_op_norm; // Variable de desviación del modelo sin retardo
  float T_predicted = initial_norm; // Salida del modelo sin retardo (estado
                                    // interno, normalizado)
  float T_delayed = initial_norm;   // Predicción extraída del buffer hace
                                    // BUFFER_SIZE pasos (normalizado)
  float error_integral =
      0.149f; // Integral inicializada al punto de equilibrio (Vp_ss = 0.149)

  // Buffer circular — implementa el retardo de transporte (d = BUFFER_SIZE
  // muestras)
  static float delay_buffer[BUFFER_SIZE];
  int buffer_idx = 0;

  for (int i = 0; i < BUFFER_SIZE; i++)
    delay_buffer[i] = initial_norm;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    float local_actual_F = T_actual;
    float local_setpoint_F = T_setpoint;
    xSemaphoreGive(stateMutex);

    // Convertir de Fahrenheit a normalizado para el controlador interno (sin
    // clamp para permitir rangos extendidos)
    float local_actual = fToNormalized(local_actual_F);
    float local_setpoint = fToNormalized(local_setpoint_F);

    // PASO 1: Recuperar la predicción de hace BUFFER_SIZE
    T_delayed = delay_buffer[buffer_idx];

    // PASO 2: Señal de retroalimentación del Predictor de Smith
    float error_correction = local_actual - T_delayed;
    float T_feedback = T_predicted + error_correction;
    float control_error = local_setpoint - T_feedback;

    // PASO 3: Ley de control PI (forma posicional)
    float u_proportional = PI_KP * control_error;
    float potential_integral = error_integral + (PI_KI * control_error);
    float u_unclamped = u_proportional + potential_integral;
    float u_final = constrain(u_unclamped, 0.0f, 1.0f);

    // PASO 4b: Anti-windup direccional
    bool saturated_high = (u_unclamped > 1.0f) && (control_error < 0.0f);
    bool saturated_low = (u_unclamped < 0.0f) && (control_error > 0.0f);

    if (!saturated_high && !saturated_low) {
      error_integral = potential_integral;
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    U_output = u_final;
    xSemaphoreGive(stateMutex);

    // PASO 5: Propagar el modelo de desviación un paso adelante y calcular
    // T_predicted absoluta
    T_predicted_dev =
        (MODEL_A * T_predicted_dev) + (MODEL_B * (u_final - 0.149f));
    T_predicted = T_predicted_dev + T_op_norm;

    // PASO 6–7: Guardar predicción en buffer y avanzar índice circular
    delay_buffer[buffer_idx] = T_predicted;
    buffer_idx = (buffer_idx + 1) % BUFFER_SIZE;

    // Telemetría para Serial Plotter (Valores en Fahrenheit para temperaturas,
    // y 0-1 para la válvula)
    float T_predicted_F = normalizedToF(T_predicted);
    Serial.print("Setpoint:");
    Serial.print(local_setpoint_F, 2);
    Serial.print(",");
    Serial.print("Actual:");
    Serial.print(local_actual_F, 2);
    Serial.print(",");
    Serial.print("Predicted:");
    Serial.print(T_predicted_F, 2);
    Serial.print(",");
    Serial.print("Valve_U:");
    Serial.println(u_final, 4);
  }
}

// Prioridad 1 — Salida Time-Proporcional (TPO) para electroválvula on/off.
//
// U_output (0.0–1.0) define el duty cycle sobre una ventana de TPO_WINDOW
// segundos. Ejemplo con U_output = 0.3 y TPO_WINDOW = 10:
//   Muestras 0–2 → válvula ABIERTA (3 de 10)
//   Muestras 3–9 → válvula CERRADA (7 de 10)
//
// El proceso térmico lento (tau = 500 s) integra los pulsos y ve un caudal
// promedio equivalente al de una válvula analógica al 30%.
void TaskActuatorWrite(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);

  int tpo_counter =
      0; // Posición actual dentro de la ventana TPO (0 … TPO_WINDOW-1)

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    float current_u = U_output;
    xSemaphoreGive(stateMutex);

    // Abrir la válvula durante las primeras on_samples muestras de cada ventana
    // (Normally Open -> LOW para abrir)
    digitalWrite(SOLENOID_PIN, tpo_counter < (int)roundf(current_u * TPO_WINDOW)
                                   ? LOW
                                   : HIGH);

    tpo_counter = (tpo_counter + 1) % TPO_WINDOW;
  }
}

// Prioridad 1 — Parsea comandos de setpoint desde el puerto Serial.
// Formato: número decimal 70.0–200.0 seguido de Enter.
void TaskSetpointInput(void *pvParameters) {
  for (;;) {
    if (Serial.available() > 0) {
      float newSetpoint = Serial.parseFloat();
      if (newSetpoint >= 70.0f && newSetpoint <= 200.0f) {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        T_setpoint = newSetpoint;
        xSemaphoreGive(stateMutex);
        Serial.print("Nuevo setpoint (F): ");
        Serial.println(newSetpoint, 2);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}