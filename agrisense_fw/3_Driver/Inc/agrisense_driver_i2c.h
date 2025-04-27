/**
 ******************************************************************************
 * @file       : agrisense_driver_i2c.h
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Header of I2C Driver
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGRISENSE_DRIVER_I2C_H__
#define __AGRISENSE_DRIVER_I2C_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "agrisense_app_syserror.h"

/* Typedef -------------------------------------------------------------------*/

/**
 * @brief Tamanho do registrador I2C (endereçamento).
 */
typedef enum i2c_reg_address_size
{
	I2C_REGISTER_SIZE_8BIT = 1,
	I2C_REGISTER_SIZE_16BIT
}i2c_reg_address_size_t;

/* Prototype Functions -------------------------------------------------------------------*/

/**
 * @brief Inicializa e configura a interface I2C.
 * @return Status do sistema.
 */
status_t i2c_init(void);

/**
 * @brief Realiza leitura simples via I2C.
 * @param address Endereço do dispositivo no barramento.
 * @param p_buffer Ponteiro para o buffer onde os dados lidos serão armazenados.
 * @param size Quantidade de bytes a serem lidos.
 * @return Status do sistema.
 */
status_t i2c_master_read(uint8_t address, uint8_t *p_buffer, uint16_t size);

/**
 * @brief Realiza escrita simples via I2C.
 * @param address Endereço do dispositivo no barramento.
 * @param p_buffer Ponteiro para o buffer contendo os dados a serem escritos.
 * @param size Quantidade de bytes a serem enviados.
 * @return Status do sistema.
 */
status_t i2c_master_write(uint8_t address, uint8_t *p_buffer, uint16_t size);

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
status_t i2c_write_buffer(uint8_t address, uint16_t reg, i2c_reg_address_size_t reg_address_size, uint8_t *p_buffer, uint16_t size);

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
status_t i2c_read_buffer(uint8_t address, uint16_t reg, i2c_reg_address_size_t reg_address_size, uint8_t *p_buffer, uint16_t size);

#endif /* __AGRISENSE_DRIVER_I2C_H__ */
