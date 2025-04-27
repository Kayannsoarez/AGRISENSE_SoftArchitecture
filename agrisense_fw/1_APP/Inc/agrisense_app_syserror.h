
/**
 ******************************************************************************
 * @file       : agrisense_app_syserror.h
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Header of APP for of errors to a design patterns format
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGRISENSE_APP_SYSERROR_H__
#define __AGRISENSE_APP_SYSERROR_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**
 * @brief Redefinição do valor de NULL como 0U
 *
 */
#undef NULL
#define NULL            0U

/* Typedef -------------------------------------------------------------------*/
typedef uint32_t status_t;

/* Defines -------------------------------------------------------------------*/

/* Definição das classes de erros, periférico que o erro pertence */
#define SYS_CLASS          									   0x0000U  // Sistema geral
#define TIM_CLASS          									   0x0001U  // Timer
#define I2C_CLASS          									   0x0002U  // I2C
#define GPIO_CLASS         									   0x0003U  // Entradas/Saídas
#define UART_CLASS         									   0x0004U  // UART
#define RTC_CLASS          									   0x0005U  // Relógio em tempo real
#define DMA_CLASS          									   0x0006U  // Acesso direto à memória
#define ADC_CLASS          									   0x0007U  // Conversor analógico-digital
#define LOG_CLASS          									   0x0008U  // Logs do sistema
#define CLOCK_CLASS        									   0x0009U  // Gerenciamento de clock


/* Definição de máscaras para operações */
#define CLASS_MASK                                             0x000000FFU
#define HAL_ERROR_MASK                                         0x0000FF00U
#define DRIVER_ERROR_MASK                                      0x00FF0000U
#define API_ERROR_MASK                                   	   0xFF000000U

/* Macros para codificar/decodificar erros */
#define ENCODE_STATUS(driver_error, hal_error, class)          (0x00FFFFFFUL & (driver_error << 16 | hal_error << 8 | class))
#define ADD_API_ERROR(status, api_error)                       (status |(api_error << 24))
#define DECODE_CLASS(status)                                   (status & CLASS_MASK)
#define DECODE_HAL_ERROR(status)                               ((status & HAL_ERROR_MASK) >> 8)
#define DECODE_DRIVER_ERROR(status)                            ((status & DRIVER_ERROR_MASK) >> 16)
#define DECODE_API_ERROR(status)                               ((status & API_ERROR_MASK) >> 24)

/* Definição de erros do sistema */
#define SYS_SUCCESS                                            0
#define SYS_ERROR                                              ENCODE_STATUS(1, 0, SYS_CLASS)
#define SYS_INVALID_PARAMETER                                  ENCODE_STATUS(2, 0, SYS_CLASS)
#define SYS_NOT_READY                                          ENCODE_STATUS(3, 0, SYS_CLASS)
#define SYS_TIMEOUT                                            ENCODE_STATUS(4, 0, SYS_CLASS)
#define SYS_BUFFER_OVERFLOW                                    ENCODE_STATUS(5, 0, SYS_CLASS)
#define SYS_NOT_FOUND                                          ENCODE_STATUS(6, 0, SYS_CLASS)
#define SYS_BUSY                                               ENCODE_STATUS(7, 0, SYS_CLASS)

#define LOG_INVALID                                            ENCODE_STATUS(1, 0, LOG_CLASS)

/* Typedef -------------------------------------------------------------------*/

/*
 * @brief Definição de erros para periféricos do SLW
 */
typedef enum agrisense_driver_error
{
	I2C_INIT_ERROR = 1,
	I2C_MASTER_READ_ERROR,
	I2C_MASTER_WRITE_ERROR,
	I2C_MEM_READ_ERROR,
	I2C_MEM_WRITE_ERROR,
	UART_INIT_ERROR,
	UART_FORMAT_ERROR,
	UART_TRANSMIT_ERROR,
	UART_RECEIVE_ERROR,
	DMA_INIT_ERROR,
	ADC_INIT_ERROR,
	ADC_CH_CONFIG_ERROR,
	ADC_READ_VALUE_ERROR,
	ADC_START_ERROR,
	ADC_POLL_ERROR,
} agrisense_driver_error_t;

/*
 * Definição de erros da API do Agrisense
 */
typedef enum agrisense_api_error
{
	BME680_INVALID_PARAMETER_ERROR = 1,      // Ponteiro nulo ou parâmetro inválido
	BME680_CHIP_ID_MISMATCH_ERROR,           // ID do chip não corresponde ao esperado
	BME680_SOFT_RESET_ERROR,                 // Falha ao executar soft reset
	BME680_CALIBRATION_READ_ERROR,           // Falha na leitura dos coeficientes de calibração
	BME680_CONFIG_HUM_ERROR,                 // Falha ao configurar oversampling de umidade
	BME680_CONFIG_MEAS_ERROR,                // Falha ao configurar oversampling de temp/press
	BME680_CONFIG_GAS_ERROR,                 // Falha ao ativar sensor de gás
	BME680_CONFIG_HEATER_ERROR,              // Falha na configuração do aquecedor de gás
	BME680_TRIGGER_MEASUREMENT_ERROR,        // Falha ao acionar medição (forced mode)
	BME680_DATA_NOT_READY_ERROR,             // Bit NEW_DATA_MASK não setado
	BME680_DATA_READ_ERROR,                  // Falha na leitura dos dados brutos
	BME680_COMPENSATION_ERROR                // Falha durante a compensação (overflow, invalid math)
} agrisense_api_error_t;


/* Prototype Functions -------------------------------------------------------------------*/
void agrisense_error_init ();
void agrisense_error_handler (status_t error);

#endif /* __AGRISENSE_APP_SYSERROR_H__ */
