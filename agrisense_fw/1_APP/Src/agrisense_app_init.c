/**
 ******************************************************************************
 * @file       : agrisense_app_init.c
 * @author     : Kayann Soares
 * @version    : V0.1
 * @date       : 28/02/2025
 * @brief      : Inicializa drivers, APIs e aplicação do projeto Agrisense
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "agrisense_app_init.h"
#include "agrisense_app_log.h"
#include "agrisense_app_syserror.h"
#include "agrisense_api_bme680.h"
#include "agrisense_driver_i2c.h"
#include "agrisense_driver_uart.h"
#include "stm32l4xx_hal.h"

/* Funções ------------------------------------------------------------------*/

/**
 * @brief Inicializa os drivers (camada DRIVER) utilizados pelo sistema
 */
void agrisense_driver_init(void)
{
	status_t status = SYS_SUCCESS;

	status = i2c_init();
	if (status != SYS_SUCCESS)
	{
		agrisense_error_handler(I2C_INIT_ERROR);
	}

	status = uart_init();
	if (status != SYS_SUCCESS)
	{
		agrisense_error_handler(UART_INIT_ERROR);
	}

	// TODO: Adicionar Outros Drivers
}

/**
 * @brief Inicializa os sensores e módulos de API (camada API)
 */
void agrisense_api_init(void)
{
	if (bme680_init() != SYS_SUCCESS)
	{
		agrisense_error_handler(SYS_ERROR);
	}

	bme680_config_t config = {
			.os_temp      = 4,    // x8
			.os_pres      = 3,    // x4
			.os_hum       = 2,    // x2
			.heater_temp  = 320,  // °C
			.heater_dur   = 150   // ms
	};

	if (bme680_configure(&config) != SYS_SUCCESS)
	{
		agrisense_error_handler(SYS_ERROR);
	}

	// TODO: Adicionar outras APIs de Sensores
}

/**
 * @brief Inicializa os módulos da aplicação (camada APP)
 */
void agrisense_app_init(void)
{
	agrisense_error_init();
}
