// ================= Bibliotecas =================
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <EEPROM.h>

// ================= Configurações =================
#define SENSOR_READ_INTERVAL_MS 300000UL  // 5 minutos
#define LOG_SIZE 32

// ================= Estruturas =================

/**
 * @brief Estrutura para armazenar uma leitura de sensores ambientais.
 */
struct SensorEntry {
  unsigned long time; /**< Timestamp da leitura (em milissegundos desde o boot). */
  float temperature;  /**< Temperatura em graus Celsius. */
  float humidity;     /**< Umidade relativa em porcentagem. */
  float pressure;     /**< Pressão atmosférica em hPa. */
};

/**
 * @brief Enumeração dos possíveis códigos de erro do sistema.
 */
enum Status {
  STATUS_OK,
  STATUS_SENSOR_ERROR, /**< Erro de comunicação ou inicialização do sensor. */
  STATUS_I2C_ERROR,    /**< Erro geral no barramento I2C. */
  STATUS_UART_ERROR,   /**< Erro de inicialização da UART. */
  STATUS_UNKNOWN_ERROR /**< Erro desconhecido. */
};

// ================= Variáveis Globais =================
Adafruit_BME680 bme;
SensorEntry sensorLog[LOG_SIZE];
uint8_t logIndex = 0;

// ================= Funções =================

/**
 * @brief Salva uma nova leitura de sensores no buffer de log e na EEPROM.
 * @param temp Temperatura lida.
 * @param hum Umidade relativa lida.
 * @param pres Pressão atmosférica lida.
 */
void saveSensorData(float temp, float hum, float pres) {
  sensorLog[logIndex].time = millis();
  sensorLog[logIndex].temperature = temp;
  sensorLog[logIndex].humidity = hum;
  sensorLog[logIndex].pressure = pres;

  // Grava também na EEPROM
  uint16_t addr = logIndex * sizeof(SensorEntry);
  EEPROM.put(addr, sensorLog[logIndex]);

  logIndex = (logIndex + 1) % LOG_SIZE;
}

/**
 * @brief Trata erros do sistema e reinicia o dispositivo.
 * @param error Código de erro identificado.
 */
void handleError(Status error) {
  Serial.print("Erro detectado: ");
  switch (error) {
    case STATUS_SENSOR_ERROR:
      Serial.println("Sensor");
      break;
    case STATUS_I2C_ERROR:
      Serial.println("I2C");
      break;
    case STATUS_UART_ERROR:
      Serial.println("UART");
      break;
    default:
      Serial.println("Desconhecido");
      break;
  }
  delay(1000);
  NVIC_SystemReset();
}

// ================= Setup =================

/**
 * @brief Função de inicialização do sistema (executada uma única vez).
 */
void setup() {
  Serial.begin(115200);
  if (!Serial) {
    handleError(STATUS_UART_ERROR);
  }

  Wire.begin(D4, D5);
  Wire.setClock(100000);

  delay(3000);

  if (!bme.begin(0x76, &Wire)) {
    handleError(STATUS_SENSOR_ERROR);
  }

  // Configurações padrão do BME680
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150);
}

// ================= Loop =================

/**
 * @brief Função principal de execução contínua.
 */
void loop() {
  if (!bme.performReading()) {
    handleError(STATUS_SENSOR_ERROR);
  }

  Serial.println("Leitura BME680:");
  Serial.print("Temp: ");
  Serial.print(bme.temperature);
  Serial.print(" *C | Hum: ");
  Serial.print(bme.humidity);
  Serial.print(" % | Press: ");
  Serial.print(bme.pressure / 100.0);
  Serial.print(" hPa | Gas: ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  Serial.println();

  saveSensorData(bme.temperature, bme.humidity, bme.pressure / 100.0);

  delay(SENSOR_READ_INTERVAL_MS);
}
