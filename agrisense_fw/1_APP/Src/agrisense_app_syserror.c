/**
 ******************************************************************************
 * @file       : agrisense_app_syserror.C
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Source file of APP for error handling
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "agrisense_app_syserror.h"
#include "agrisense_app_log.h"
#include "stm32l4xx_hal.h"

/**
 * @brief Instância do sistema de log.
 */
agrisense_log_instance_t g_log_instance;

/**
 * @brief Função de inicialização do log de erros na memória
 *
 */
void agrisense_error_init ()
{
	status_t status = agrisense_log_init(&g_log_instance);

	if (status != SYS_SUCCESS)
	{
		__NVIC_SystemReset();
	}
}

/**
 * @brief Função para tratamento de erros do agrisense
 *
 * @param error Código de erro retornado pelas APIs
 */
void agrisense_error_handler (status_t error)
{

	uint8_t class = DECODE_CLASS(error);

	switch (class)
	{
	case SYS_CLASS:
	case TIM_CLASS:
	case I2C_CLASS:
	case GPIO_CLASS:
	case UART_CLASS:
	case RTC_CLASS:
	case ADC_CLASS:
	case DMA_CLASS:
	default:
		agrisense_log_write(&g_log_instance, error);
		break;
	}

	__NVIC_SystemReset();
}
