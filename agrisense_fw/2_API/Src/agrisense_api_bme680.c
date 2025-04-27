/**
 ******************************************************************************
 * @file       : agrisense_api_bme680.c
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Source file of API for BME680 Sensor
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "agrisense_api_bme680.h"
#include "agrisense_driver_i2c.h"
#include "stm32l4xx_hal.h"

/* Defines ------------------------------------------------------------------*/

/**
 * @def BME680_I2C_ADDR
 * @brief Endereço I2C padrão do sensor BME680 (0x76 ou 0x77, dependendo do pino SDO)
 */
#define BME680_I2C_ADDR         0x76

/**
 * @def BME680_REG_ID
 * @brief Endereço do registrador de identificação do sensor
 */
#define BME680_REG_ID           0xD0

/**
 * @def BME680_CHIP_ID
 * @brief Valor esperado de ID do sensor BME680 (0x61)
 */
#define BME680_CHIP_ID          0x61

/**
 * @def BME680_REG_SOFT_RESET
 * @brief Endereço do registrador de reset do sensor
 */
#define BME680_REG_SOFT_RESET   0xE0

/**
 * @def BME680_SOFT_RESET_CMD
 * @brief Comando para realizar soft reset do sensor
 */
#define BME680_SOFT_RESET_CMD   0xB6

/**
 * @def BME680_REG_CTRL_HUM
 * @brief Endereço do registrador de controle de oversampling de umidade
 */
#define BME680_REG_CTRL_HUM     0x72

/**
 * @def BME680_REG_CTRL_MEAS
 * @brief Endereço do registrador de controle de oversampling de temperatura e pressão
 */
#define BME680_REG_CTRL_MEAS    0x74

/**
 * @def BME680_REG_CTRL_GAS_1
 * @brief Endereço do registrador de controle do sensor de gás
 */
#define BME680_REG_CTRL_GAS_1   0x71

/**
 * @def BME680_REG_RES_HEAT_0
 * @brief Endereço do registrador de configuração do aquecedor (heater)
 */
#define BME680_REG_RES_HEAT_0   0x5A

/**
 * @def BME680_REG_GAS_WAIT_0
 * @brief Endereço do registrador de configuração de tempo de aquecimento
 */
#define BME680_REG_GAS_WAIT_0   0x64

/**
 * @def BME680_REG_FIELD_0
 * @brief Endereço do registrador onde estão os dados brutos da medição
 */
#define BME680_REG_FIELD_0      0x1D

/**
 * @def BME680_REG_CALIB_DATA
 * @brief Endereço inicial dos registradores de calibração do sensor
 */
#define BME680_REG_CALIB_DATA   0x89


#define BME680_RUN_GAS_ENABLE   0x10
#define BME680_RES_HEAT_DEFAULT 0x5A
#define BME680_GAS_WAIT_DEFAULT 0x59
#define BME680_NEW_DATA_MASK    0x80
#define BME680_READ_DELAY_MS    180



/**
 * @brief Escreve um valor em um registrador do sensor BME680 via I2C
 *
 * @param reg Endereço do registrador
 * @param value Valor a ser escrito
 * @return SYS_SUCCESS em caso de sucesso, erro caso contrário
 */

/* Private Functions --------------------------------------------------------*/
static status_t write_register(uint8_t reg, uint8_t value)
{
	return i2c_write_buffer(BME680_I2C_ADDR, reg, I2C_REGISTER_SIZE_8BIT, &value, 1);
}

/**
 * @brief Lê um byte de um registrador do sensor BME680 via I2C
 *
 * @param reg Endereço do registrador
 * @param value Ponteiro para armazenar o valor lido
 * @return SYS_SUCCESS em caso de sucesso, erro caso contrário
 */
static status_t read_register(uint8_t reg, uint8_t *value)
{
	return i2c_read_buffer(BME680_I2C_ADDR, reg, I2C_REGISTER_SIZE_8BIT, value, 1);
}

/**
 * @brief Lê múltiplos bytes consecutivos de registradores do BME680 via I2C
 *
 * @param reg Endereço inicial do registrador
 * @param buffer Ponteiro para buffer de destino
 * @param len Quantidade de bytes a serem lidos
 * @return SYS_SUCCESS em caso de sucesso, erro caso contrário
 */
static status_t read_bytes(uint8_t reg, uint8_t *buffer, uint16_t len)
{
	return i2c_read_buffer(BME680_I2C_ADDR, reg, I2C_REGISTER_SIZE_8BIT, buffer, len);
}


/**
 * @brief Estrutura que armazena os coeficientes de calibração lidos do sensor
 */
static bme680_calib_data_t calib;

/**
 * @brief Valor auxiliar usado nas equações de compensação de temperatura, pressão e umidade
 */
static float t_fine;

/**
 * @brief Lê os coeficientes de calibração do sensor BME680 e armazena localmente
 */
static void read_calibration_data(void)
{
	uint8_t coeff1[25] = {0};  // 0x89–0xA1 (25 bytes)
	uint8_t coeff2[16] = {0};  // 0xE1–0xF0 (16 bytes)
	uint8_t coeff3[1]  = {0};  // 0x02        (1 byte)

	// Leitura dos registradores conforme datasheet e API Bosch
	read_bytes(0x89, coeff1, 25);  // reg_data[0]  → reg_data[24]
	read_bytes(0xE1, coeff2, 16);  // reg_data[25] → reg_data[40]
	read_bytes(0x02, coeff3, 1);   // range_sw_err

	// Temperatura
	calib.par_t1 = (uint16_t)((coeff1[1]  << 8) | coeff1[0]);
	calib.par_t2 = (int16_t)((coeff1[3]   << 8) | coeff1[2]);
	calib.par_t3 = (int8_t)coeff1[4];

	// Pressão
	calib.par_p1 = (uint16_t)((coeff1[6]  << 8) | coeff1[5]);
	calib.par_p2 = (int16_t)((coeff1[8]   << 8) | coeff1[7]);
	calib.par_p3 = (int8_t)coeff1[9];
	calib.par_p4 = (int16_t)((coeff1[11] << 8) | coeff1[10]);
	calib.par_p5 = (int16_t)((coeff1[13] << 8) | coeff1[12]);
	calib.par_p6 = (int8_t)coeff1[14];
	calib.par_p7 = (int8_t)coeff1[15];
	calib.par_p8 = (int16_t)((coeff1[17] << 8) | coeff1[16]);
	calib.par_p9 = (int16_t)((coeff1[19] << 8) | coeff1[18]);
	calib.par_p10 = coeff1[20];

	// Umidade
	calib.par_h1 = (uint16_t)((coeff2[1] << 4) | (coeff2[0] & 0x0F));
	calib.par_h2 = (uint16_t)((coeff2[2] << 4) | (coeff2[0] >> 4));
	calib.par_h3 = (int8_t)coeff2[3];
	calib.par_h4 = (int8_t)coeff2[4];
	calib.par_h5 = (int8_t)coeff2[5];
	calib.par_h6 = (uint8_t)coeff2[6];
	calib.par_h7 = (int8_t)coeff2[7];

	// Gás
	calib.par_gh1 = (int8_t)coeff2[8];
	calib.par_gh2 = (int16_t)((coeff2[10] << 8) | coeff2[9]);
	calib.par_gh3 = (int8_t)coeff2[11];

	// Outros
	calib.res_heat_range = (coeff2[14] & 0x30) >> 4;
	calib.res_heat_val   = (int8_t)coeff2[15];
	calib.range_sw_err   = (int8_t)((coeff3[0] & 0xF0) >> 4);
}
/**
 * @brief Aplica compensação nos dados brutos de temperatura do BME680
 *
 * @param adc_temp Valor ADC de temperatura lido do sensor
 * @return Temperatura compensada em °C x100
 */
static float compensate_temperature(uint32_t adc_temp)
{
	float var1 = (((float)adc_temp / 16384.0f) - ((float)calib.par_t1 / 1024.0f)) * calib.par_t2;
	float var2 = ((((float)adc_temp / 131072.0f) - ((float)calib.par_t1 / 8192.0f)) *
			(((float)adc_temp / 131072.0f) - ((float)calib.par_t1 / 8192.0f))) * (calib.par_t3 * 16.0f);

	t_fine = var1 + var2;
	return t_fine / 5120.0f;
}

/**
 * @brief Aplica compensação nos dados brutos de pressão do BME680
 *
 * @param adc_pres Valor ADC de pressão lido do sensor
 * @return Pressão compensada em Pa (divida por 100.0 para obter hPa)
 */
static float compensate_pressure(uint32_t adc_pres)
{
	float var1 = (t_fine / 2.0f) - 64000.0f;
	float var2 = var1 * var1 * (calib.par_p6 / 131072.0f);
	var2 = var2 + (var1 * calib.par_p5 * 2.0f);
	var2 = (var2 / 4.0f) + (calib.par_p4 * 65536.0f);
	var1 = (((calib.par_p3 * var1 * var1 / 16384.0f) + (calib.par_p2 * var1)) / 524288.0f);
	var1 = (1.0f + (var1 / 32768.0f)) * calib.par_p1;
	float calc_pres = 1048576.0f - adc_pres;

	if ((int)var1 != 0)
	{
		calc_pres = ((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1;
		var1 = (calib.par_p9 * calc_pres * calc_pres) / 2147483648.0f;
		var2 = calc_pres * (calib.par_p8 / 32768.0f);
		float var3 = (calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * (calib.par_p10 / 131072.0f);
		calc_pres = calc_pres + (var1 + var2 + var3 + (calib.par_p7 * 128.0f)) / 16.0f;
	}
	else
	{
		calc_pres = 0;
	}

	return calc_pres;
}

/**
 * @brief Aplica compensação nos dados brutos de umidade do BME680
 *
 * @param adc_hum Valor ADC de umidade lido do sensor
 * @return Umidade compensada em % x1000
 */
static float compensate_humidity(uint16_t adc_hum)
{
	float temp_comp = t_fine / 5120.0f;
	float var1 = (float)adc_hum - ((calib.par_h1 * 16.0f) + ((calib.par_h3 / 2.0f) * temp_comp));
	float var2 = var1 * (calib.par_h2 / 262144.0f *
			(1.0f + ((calib.par_h4 / 16384.0f) * temp_comp) +
					((calib.par_h5 / 1048576.0f) * temp_comp * temp_comp)));
	float var3 = calib.par_h6 / 16384.0f;
	float var4 = calib.par_h7 / 2097152.0f;
	float calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

	if (calc_hum > 100.0f) calc_hum = 100.0f;
	else if (calc_hum < 0.0f) calc_hum = 0.0f;

	return calc_hum;
}

/**
 * @brief Aplica a compensação no valor bruto da resistência de gás do BME680.
 *
 * @param gas_res_adc Valor ADC bruto da resistência do gás (10 bits) obtido do sensor.
 * @param gas_range Intervalo de medição do gás (0–15), também lido do sensor.
 *
 * @return Valor compensado da resistência do gás em Ohms (Ω).
 */
static float compensate_gas(uint32_t gas_res_adc, uint8_t gas_range)
{
	float var1, var2, var3;
	float gas_res_f = (float)gas_res_adc;
	float gas_range_f = (float)(1U << gas_range);

	const float lookup_k1_range[16] = {
			0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f,
			0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
	};

	const float lookup_k2_range[16] = {
			0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f,
			-0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
	};

	var1 = 1340.0f + (5.0f * (float)calib.range_sw_err);
	var2 = var1 * (1.0f + lookup_k1_range[gas_range] / 100.0f);
	var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);

	return 1.0f / (var3 * 0.000000125f * gas_range_f * (((gas_res_f - 512.0f) / var2) + 1.0f));
}

/* Public Functions ---------------------------------------------------------*/

/**
 * @brief Inicializa o sensor BME680
 * @return SYS_SUCCESS em caso de sucesso, SYS_ERROR caso contrário
 */
status_t bme680_init(void)
{
	uint8_t id = 0;

	if (read_register(BME680_REG_ID, &id) != SYS_SUCCESS || id != BME680_CHIP_ID)
		return SYS_ERROR;

	if (write_register(BME680_REG_SOFT_RESET, BME680_SOFT_RESET_CMD) != SYS_SUCCESS)
		return SYS_ERROR;

	HAL_Delay(10);
	read_calibration_data();
	return SYS_SUCCESS;
}

/**
 * @brief Configura os parâmetros do sensor (oversampling, etc)
 * @param config Ponteiro para estrutura de configuração
 * @return SYS_SUCCESS em caso de sucesso, SYS_ERROR caso contrário
 */
status_t bme680_configure(const bme680_config_t *config)
{
	if (!config) return SYS_INVALID_PARAMETER;

	if (write_register(BME680_REG_CTRL_HUM, config->os_hum) != SYS_SUCCESS)
		return SYS_ERROR;

	uint8_t ctrl_meas = (config->os_temp << 5) | (config->os_pres << 2);
	if (write_register(BME680_REG_CTRL_MEAS, ctrl_meas) != SYS_SUCCESS)
		return SYS_ERROR;

	uint8_t ctrl_gas_1 = 0x10;
	if (write_register(BME680_REG_CTRL_GAS_1, ctrl_gas_1) != SYS_SUCCESS)
		return SYS_ERROR;

	uint8_t res_heat = 0x5A;
	uint8_t gas_wait = 0x59;
	if (write_register(BME680_REG_RES_HEAT_0, res_heat) != SYS_SUCCESS ||
			write_register(BME680_REG_GAS_WAIT_0, gas_wait) != SYS_SUCCESS)
		return SYS_ERROR;

	return SYS_SUCCESS;
}

/**
 * @brief Realiza uma medição e retorna os dados compensados
 * @param data Ponteiro para estrutura que receberá os dados
 * @return SYS_SUCCESS em caso de sucesso, SYS_ERROR caso contrário
 */
status_t bme680_read(bme680_data_t *data)
{
	if (!data) return SYS_INVALID_PARAMETER;

	// 1. Força uma medição (modo "forced mode") SEM sobrescrever oversampling
	uint8_t ctrl_meas;
	if (read_register(BME680_REG_CTRL_MEAS, &ctrl_meas) != SYS_SUCCESS)
		return SYS_ERROR;

	ctrl_meas = (ctrl_meas & 0xFC) | 0x01;  // mantém oversampling, ativa forced mode
	if (write_register(BME680_REG_CTRL_MEAS, ctrl_meas) != SYS_SUCCESS)
		return SYS_ERROR;

	HAL_Delay(BME680_READ_DELAY_MS); // aguarda conversão (~180ms para config típica)

	// 2. Leitura dos dados brutos do registrador FIELD_0
	uint8_t buffer[15];
	if (read_bytes(BME680_REG_FIELD_0, buffer, 15) != SYS_SUCCESS)
		return SYS_ERROR;

	// 3. Verifica se os dados são válidos (bit NEW_DATA_READY)
	if ((buffer[0] & BME680_NEW_DATA_MASK) == 0)
		return SYS_ERROR;  // dados ainda não disponíveis

	// 4. Constrói valores brutos a partir dos bytes
	uint32_t adc_pres = ((uint32_t)(buffer[2]) << 12) | ((uint32_t)(buffer[3]) << 4) | ((uint32_t)(buffer[4]) >> 4);
	uint32_t adc_temp = ((uint32_t)(buffer[5]) << 12) | ((uint32_t)(buffer[6]) << 4) | ((uint32_t)(buffer[7]) >> 4);
	uint16_t adc_hum  = ((uint16_t)(buffer[8]) << 8) | buffer[9];
	uint32_t gas_res_adc = ((uint16_t)buffer[13] << 2) | ((buffer[14] & 0xC0) >> 6);
	uint8_t gas_range = buffer[14] & 0x0F;

	// 5. Compensa e preenche estrutura de saída
	data->temperature     = compensate_temperature(adc_temp);                  // °C
	data->pressure        = compensate_pressure(adc_pres) / 100.0f;           // hPa
	data->humidity        = compensate_humidity(adc_hum);                     // %
	data->gas_resistance  = compensate_gas(gas_res_adc, gas_range) / 1000.0f; // kΩ

	return SYS_SUCCESS;
}
