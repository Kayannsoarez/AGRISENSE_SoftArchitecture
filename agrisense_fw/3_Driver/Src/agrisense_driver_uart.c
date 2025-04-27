/**
 ******************************************************************************
 * @file       : agrisense_driver_uart.C
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Source file of driver that controls UART peripheral
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "agrisense_driver_uart.h"
#include "stm32l4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
/* Definições do periférico UART */

/**
 * @def AGRISENSE_UART
 * @brief Instância da UART utilizada para comunicação serial.
 *
 */
#define AGRISENSE_UART                     USART1

/**
 * @def AGRISENSE_UART_BAUDRATE
 * @brief Taxa de transmissão (baudrate) configurada para a UART.
 *
 */
#define AGRISENSE_UART_BAUDRATE           115200

/**
 * @def AGRISENSE_UART_WORD_LENGTH
 * @brief Tamanho da palavra de dados na UART (8 bits).
 *
 */
#define AGRISENSE_UART_WORD_LENGTH        UART_WORDLENGTH_8B

/**
 * @def AGRISENSE_UART_MODE
 * @brief Modo de operação da UART (transmissão e recepção).
 *
 */
#define AGRISENSE_UART_MODE               UART_MODE_TX_RX

/**
 * @def AGRISENSE_UART_CLK_ENABLE
 * @brief Macro que habilita o clock do periférico UART.
 *
 */
#define AGRISENSE_UART_CLK_ENABLE()         __HAL_RCC_USART1_CLK_ENABLE()

/**
 * @def AGRISENSE_UART_TX_GPIO_CLK_ENABLE
 * @brief Macro que habilita o clock do GPIO utilizado para TX.
 *
 */
#define AGRISENSE_UART_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @def AGRISENSE_UART_RX_GPIO_CLK_ENABLE
 * @brief Macro que habilita o clock do GPIO utilizado para RX.
 *
 */
#define AGRISENSE_UART_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @def AGRISENSE_UART_TX_PIN
 * @brief Definição do pino utilizado para transmissão (TX) da UART.
 *
 */
#define AGRISENSE_UART_TX_PIN             GPIO_PIN_9

/**
 * @def AGRISENSE_UART_RX_PIN
 * @brief Definição do pino utilizado para recepção (RX) da UART.
 *
 */
#define AGRISENSE_UART_RX_PIN             GPIO_PIN_10

/**
 * @def AGRISENSE_UART_GPIO_PORT
 * @brief Porta GPIO utilizada para os pinos TX e RX da UART.
 *
 */
#define AGRISENSE_UART_GPIO_PORT          GPIOA

/**
 * @def AGRISENSE_UART_AF
 * @brief Função alternativa (AF) dos pinos TX e RX para a UART.
 *
 */
#define AGRISENSE_UART_AF                 GPIO_AF7_USART1

/**
 * @def AGRISENSE_UART_TIMEOUT
 * @brief Tempo máximo de espera nas operações de transmissão e recepção.
 *
 */
#define AGRISENSE_UART_TIMEOUT            0x1000

/* Handlers ------------------------------------------------------------------*/

/**
 * @brief Handler da interface UART.
 */
UART_HandleTypeDef g_uart1_handler;

/* Private Functions --------------------------------------------------------*/

/**
 * @brief Inicialização do MSP (MCU Support Package) UART.
 * @return void
 */
static void uart_msp_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	AGRISENSE_UART_TX_GPIO_CLK_ENABLE();
	AGRISENSE_UART_RX_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin = AGRISENSE_UART_TX_PIN | AGRISENSE_UART_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = AGRISENSE_UART_AF;
	HAL_GPIO_Init(AGRISENSE_UART_GPIO_PORT, &GPIO_InitStruct);

	AGRISENSE_UART_CLK_ENABLE();
}


/* Public Functions ---------------------------------------------------------*/

/**
 * @brief Inicializa e configura a interface UART.
 * @return Status do Sistema.
 */
status_t uart_init(void)
{
	HAL_UART_StateTypeDef uart_state = HAL_UART_GetState(&g_uart1_handler);

	if (uart_state == HAL_UART_STATE_RESET)
	{
		g_uart1_handler.Instance = AGRISENSE_UART;
		g_uart1_handler.Init.BaudRate = AGRISENSE_UART_BAUDRATE;
		g_uart1_handler.Init.WordLength = AGRISENSE_UART_WORD_LENGTH;
		g_uart1_handler.Init.StopBits = UART_STOPBITS_1;
		g_uart1_handler.Init.Parity = UART_PARITY_NONE;
		g_uart1_handler.Init.Mode = AGRISENSE_UART_MODE;
		g_uart1_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		g_uart1_handler.Init.OverSampling = UART_OVERSAMPLING_16;
		g_uart1_handler.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		g_uart1_handler.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

		uart_msp_init();

		if (HAL_UART_Init(&g_uart1_handler) != HAL_OK)
		{
			return ENCODE_STATUS(UART_INIT_ERROR, g_uart1_handler.ErrorCode, UART_CLASS);
		}

		return SYS_SUCCESS;
	}
	else if (uart_state == HAL_UART_STATE_READY)
	{
		return SYS_SUCCESS;
	}

	return SYS_NOT_READY;
}

/**
 * @brief Transmite dados pela UART de forma bloqueante.
 * @param p_data Ponteiro para o buffer de dados a ser transmitido.
 * @param size Quantidade de bytes a transmitir.
 * @return Status do Sistema.
 */
status_t uart_transmit(uint8_t *p_data, uint16_t size)
{
	if (p_data == NULL)
		return SYS_INVALID_PARAMETER;

	HAL_StatusTypeDef status = HAL_UART_Transmit(&g_uart1_handler, p_data, size, AGRISENSE_UART_TIMEOUT);

	if (status != HAL_OK)
		return ENCODE_STATUS(UART_TRANSMIT_ERROR, g_uart1_handler.ErrorCode, UART_CLASS);

	return SYS_SUCCESS;
}

/**
 * @brief Recebe dados pela UART de forma bloqueante.
 * @param p_data Ponteiro para o buffer onde os dados recebidos serão armazenados.
 * @param size Quantidade de bytes a receber.
 * @return Status do sistema.
 */
status_t uart_receive(uint8_t *p_data, uint16_t size)
{
	if (p_data == NULL)
		return SYS_INVALID_PARAMETER;

	HAL_StatusTypeDef status = HAL_UART_Receive(&g_uart1_handler, p_data, size, AGRISENSE_UART_TIMEOUT);

	if (status != HAL_OK)
		return ENCODE_STATUS(UART_RECEIVE_ERROR, g_uart1_handler.ErrorCode, UART_CLASS);

	return SYS_SUCCESS;
}

/**
 * @brief Envia uma string formatada pela UART.
 * @param format Cadeia de formato (ex: "%d %s").
 * @param ... Argumentos variáveis a serem formatados.
 * @return Status do sistema.
 */
status_t uart_print(const char *format, ...)
{
	char buffer[128];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	if (len < 0 || len >= sizeof(buffer))
		return ENCODE_STATUS(UART_FORMAT_ERROR, 0, UART_CLASS);

	return uart_transmit((uint8_t *)buffer, len);
}
