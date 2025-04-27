/**
 ******************************************************************************
 * @file       : agrisense_app_init.h
 * @author     : Kayann Soares
 * @version    : V0.1
 * @date       : 28/02/2025
 * @brief      : Header da inicialização de sistema (DRIVER, API, APP)
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AGRISENSE_APP_INIT_H_
#define AGRISENSE_APP_INIT_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "agrisense_app_init.h"
#include "agrisense_app_log.h"
#include "agrisense_app_syserror.h"
#include "agrisense_api_bme680.h"
#include "agrisense_driver_i2c.h"
#include "agrisense_driver_uart.h"
#include "stm32l4xx_hal.h"

/**
 * @brief Inicializa todos os drivers de hardware (camada DRIVER)
 */
void agrisense_driver_init(void);

/**
 * @brief Inicializa sensores e módulos de API (camada API)
 */
void agrisense_api_init(void);

/**
 * @brief Inicializa os módulos da aplicação (camada APP)
 */
void agrisense_app_init(void);

/**
 * @brief Função principal da aplicação (executada em loop no main)
 */
void agrisense_app_main(void);


#endif /* AGRISENSE_APP_INIT_H_ */
