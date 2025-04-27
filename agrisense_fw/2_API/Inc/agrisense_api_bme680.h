/**
 ******************************************************************************
 * @file       : agrisense_api_bme680.h
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Header of API of BME680 Sensor
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGRISENSE_API_BME680_H__
#define __AGRISENSE_API_BME680_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "agrisense_app_syserror.h"

/* Typedef -------------------------------------------------------------------*/

/**
 * @brief Estrutura com dados ambientais lidos do BME680
 */
typedef struct bme680_data
{
	float temperature;       /**< Temperatura em graus Celsius */
	float pressure;          /**< Pressão em hPa */
	float humidity;          /**< Umidade relativa em % */
	float gas_resistance;    /**< Resistência de gás (não utilizado) */
} bme680_data_t;

/**
 * @brief Estrutura de configuração do sensor BME680
 */
typedef struct bme680_config
{
	uint8_t os_temp;     /**< Oversampling temperatura */
	uint8_t os_pres;     /**< Oversampling pressão */
	uint8_t os_hum;      /**< Oversampling umidade */
	uint16_t heater_temp; /**< Temperatura do aquecedor em °C */
	uint16_t heater_dur;  /**< Duração do aquecimento em ms */
} bme680_config_t;

typedef struct bme680_calib_data
{
	// Temperatura
	uint16_t par_t1;
	int16_t  par_t2;
	int8_t   par_t3;

	// Pressão
	uint16_t par_p1;
	int16_t  par_p2;
	int8_t   par_p3;
	int16_t  par_p4;
	int16_t  par_p5;
	int8_t   par_p6;
	int8_t   par_p7;
	int16_t  par_p8;
	int16_t  par_p9;
	uint8_t  par_p10;

	// Umidade
	uint16_t par_h1;
	uint16_t par_h2;
	int8_t   par_h3;
	int8_t   par_h4;
	int8_t   par_h5;
	uint8_t  par_h6;
	int8_t   par_h7;

	// Gás
	int8_t   par_gh1;
	int16_t  par_gh2;
	int8_t   par_gh3;

	// Correções
	uint8_t  res_heat_range;
	int8_t   res_heat_val;
	int8_t   range_sw_err;

	// Valor auxiliar para compensações
	float    t_fine;

} bme680_calib_data_t;

/* Prototype Functions -------------------------------------------------------------------*/
status_t bme680_init(void);
status_t bme680_configure(const bme680_config_t *config);
status_t bme680_read(bme680_data_t *data);

#endif /* __AGRISENSE_API_BME680_H__ */
