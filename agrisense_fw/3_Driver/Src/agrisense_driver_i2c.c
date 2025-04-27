/**
 ******************************************************************************
 * @file       : agrisense_driver_i2c.c
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Source file of driver that controls I2C peripheral
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "agrisense_driver_i2c.h"
#include "stm32l4xx_hal.h"

/* Defines -------------------------------------------------------------------*/

/**
 * @def AGRISENSE_I2C
 * @brief Instância do periférico I2C utilizada no projeto.
 */
#define AGRISENSE_I2C                               I2C1

/**
 * @def AGRISENSE_I2C_TIMMING
 * @brief Valor do registrador TIMINGR do I2C configurado para 100 kHz com clock HSI de 16 MHz.
 */
#define AGRISENSE_I2C_TIMMING                       0x0020098E

/**
 * @def AGRISENSE_I2C_CLK_ENABLE
 * @brief Habilita o clock do periférico I2C.
 */
#define AGRISENSE_I2C_CLK_ENABLE()                  __HAL_RCC_I2C1_CLK_ENABLE()

/**
 * @def AGRISENSE_I2C_SCL_SDA_GPIO_CLK_ENABLE
 * @brief Habilita o clock dos GPIOs utilizados para os pinos SCL e SDA.
 */
#define AGRISENSE_I2C_SCL_SDA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()

/**
 * @def AGRISENSE_I2C_SCL_SDA_AF
 * @brief Função alternativa (Alternate Function) configurada nos pinos SCL e SDA.
 */
#define AGRISENSE_I2C_SCL_SDA_AF                    GPIO_AF4_I2C1

/**
 * @def AGRISENSE_I2C_SCL_SDA_GPIO_PORT
 * @brief Porta GPIO onde os pinos SCL e SDA estão conectados.
 */
#define AGRISENSE_I2C_SCL_SDA_GPIO_PORT             GPIOB

/**
 * @def AGRISENSE_I2C_SCL_PIN
 * @brief Pino utilizado para o sinal SCL (Serial Clock).
 */
#define AGRISENSE_I2C_SCL_PIN                       GPIO_PIN_6

/**
 * @def AGRISENSE_I2C_SDA_PIN
 * @brief Pino utilizado para o sinal SDA (Serial Data).
 */
#define AGRISENSE_I2C_SDA_PIN                       GPIO_PIN_7

/**
 * @def AGRISENSE_I2C_FORCE_RESET
 * @brief Força o reset do periférico I2C.
 */
#define AGRISENSE_I2C_FORCE_RESET()                 __HAL_RCC_I2C1_FORCE_RESET()

/**
 * @def AGRISENSE_I2C_RELEASE_RESET
 * @brief Libera o reset do periférico I2C.
 */
#define AGRISENSE_I2C_RELEASE_RESET()               __HAL_RCC_I2C1_RELEASE_RESET()

/**
 * @def AGRISENSE_I2C_TIMEOUT
 * @brief Tempo máximo de espera em loops de operação no barramento I2C (em ticks).
 */
#define AGRISENSE_I2C_TIMEOUT                       0x1000

/* Handlers ------------------------------------------------------------------*/

/**
 * @var g_i2c_handler
 * @brief Handler da interface I2C utilizada pelo driver.
 */
static I2C_HandleTypeDef g_i2c_handler;

/* Private Functions --------------------------------------------------------*/

/**
 * @brief Inicialização do MSP (MCU Support Package) I2C.
 * @return void
 */
static void i2c_msp_init (void)
{
	GPIO_InitTypeDef gpio_init_struct = { 0 };

	/* Habilita os clocks dos GPIOs utilizados pela I2C */
	AGRISENSE_I2C_SCL_SDA_GPIO_CLK_ENABLE();

	/* Configuração dos pinos SCL e SDA do SLW ---------------------------*/
	gpio_init_struct.Pin = AGRISENSE_I2C_SCL_PIN | AGRISENSE_I2C_SDA_PIN;
	gpio_init_struct.Mode = GPIO_MODE_AF_OD;
	gpio_init_struct.Speed = GPIO_SPEED_FAST;
	gpio_init_struct.Pull = GPIO_PULLUP;
	gpio_init_struct.Alternate = AGRISENSE_I2C_SCL_SDA_AF;
	HAL_GPIO_Init(AGRISENSE_I2C_SCL_SDA_GPIO_PORT, &gpio_init_struct);

	/* Habilita o clock do periférico I2C */
	AGRISENSE_I2C_CLK_ENABLE();

	/* Força o reset do clock do periférico I2C */
	AGRISENSE_I2C_FORCE_RESET();

	/* Libera o reset do clock do periférico I2C */
	AGRISENSE_I2C_RELEASE_RESET();

}

/* Public Functions ---------------------------------------------------------*/

/**
 * @brief Inicializa e Configura a interface I2C.
 * @return Status do sistema.
 */
status_t i2c_init (void)
{
	if (HAL_I2C_GetState(&g_i2c_handler) == HAL_I2C_STATE_RESET)
	{
		g_i2c_handler.Instance = AGRISENSE_I2C;
		g_i2c_handler.Init.Timing = AGRISENSE_I2C_TIMMING;
		g_i2c_handler.Init.OwnAddress1 = 0;
		g_i2c_handler.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		g_i2c_handler.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		g_i2c_handler.Init.OwnAddress2 = 0;
		g_i2c_handler.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		g_i2c_handler.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		g_i2c_handler.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

		/* Inicia o I2C */
		i2c_msp_init();

		HAL_StatusTypeDef status = HAL_I2C_Init(&g_i2c_handler);

		if (status != HAL_OK)
		{
			return ENCODE_STATUS(I2C_INIT_ERROR, g_i2c_handler.ErrorCode, I2C_CLASS);
		}

		return SYS_SUCCESS;
	}

	return SYS_NOT_READY;
}

/**
 * @brief Realiza leitura simples via I2C.
 * @param address Endereço do dispositivo no barramento.
 * @param p_buffer Ponteiro para o buffer onde os dados lidos serão armazenados.
 * @param size Quantidade de bytes a serem lidos.
 * @return Status do sistema.
 */
status_t i2c_master_read (uint8_t address, uint8_t *p_buffer, uint16_t size)
{
	address <<= 1;

	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&g_i2c_handler, address, p_buffer, size, AGRISENSE_I2C_TIMEOUT);

	if (status != HAL_OK)
	{
		return ENCODE_STATUS(I2C_MASTER_READ_ERROR, g_i2c_handler.ErrorCode, I2C_CLASS);
	}

	return SYS_SUCCESS;
}

/**
 * @brief Realiza escrita simples via I2C.
 * @param address Endereço do dispositivo no barramento.
 * @param p_buffer Ponteiro para o buffer contendo os dados a serem escritos.
 * @param size Quantidade de bytes a serem enviados.
 * @return Status do sistema.
 */
status_t i2c_master_write (uint8_t address, uint8_t *p_buffer, uint16_t size)
{
	address <<= 1;

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&g_i2c_handler, address, p_buffer, size, AGRISENSE_I2C_TIMEOUT);

	if (status != HAL_OK)
	{
		return ENCODE_STATUS(I2C_MASTER_WRITE_ERROR, g_i2c_handler.ErrorCode, I2C_CLASS);
	}

	return SYS_SUCCESS;
}

/**
 * @brief Escreve um buffer em um registrador específico via I2C.
 * @param address Endereço do dispositivo no barramento.
 * @param reg Endereço do registrador a ser escrito.
 * @param reg_address_size Tamanho do endereço do registrador (8 ou 16 bits).
 *        Este parâmetro pode ser:
 *          - I2C_REGISTER_SIZE_8BIT
 *          - I2C_REGISTER_SIZE_16BIT
 * @param p_buffer Ponteiro para o buffer com os dados a serem escritos.
 * @param size Quantidade de bytes a serem escritos.
 * @return Status do sistema.
 */
status_t i2c_write_buffer (uint8_t address, uint16_t reg, i2c_reg_address_size_t reg_address_size, uint8_t *p_buffer, uint16_t size)
{
	address <<= 1;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&g_i2c_handler, address, reg, reg_address_size, p_buffer, size, AGRISENSE_I2C_TIMEOUT);

	if (status != HAL_OK)
	{
		return ENCODE_STATUS(I2C_MEM_WRITE_ERROR, g_i2c_handler.ErrorCode, I2C_CLASS);
	}

	return SYS_SUCCESS;
}

/**
 * @brief Lê um buffer de um registrador específico via I2C.
 * @param address Endereço do dispositivo no barramento.
 * @param reg Endereço do registrador a ser lido.
 * @param reg_address_size Tamanho do endereço do registrador (8 ou 16 bits).
 *        Este parâmetro pode ser:
 *          - I2C_REGISTER_SIZE_8BIT
 *          - I2C_REGISTER_SIZE_16BIT
 * @param p_buffer Ponteiro para o buffer onde os dados lidos serão armazenados.
 * @param size Quantidade de bytes a serem lidos.
 * @return Status do sistema.
 */
status_t i2c_read_buffer (uint8_t address, uint16_t reg, i2c_reg_address_size_t reg_address_size, uint8_t *p_buffer, uint16_t size)
{
	address <<= 1;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&g_i2c_handler, address, reg, reg_address_size, p_buffer, size,
			AGRISENSE_I2C_TIMEOUT);
	if (status != HAL_OK)
	{
		return ENCODE_STATUS(I2C_MEM_READ_ERROR, g_i2c_handler.ErrorCode, I2C_CLASS);
	}

	return SYS_SUCCESS;
}
