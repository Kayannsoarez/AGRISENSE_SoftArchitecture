/**
 ******************************************************************************
 * @file       : agrisense_driver_uart.h
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Header of UART Driver
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGRISENSE_DRIVER_UART_H__
#define __AGRISENSE_DRIVER_UART_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "agrisense_app_syserror.h"

/* Prototype Functions -------------------------------------------------------------------*/

/**
 * @brief Inicializa e configura a interface UART.
 * @return Status do Sistema.
 */
status_t uart_init(void);

/**
 * @brief Transmite dados pela UART de forma bloqueante.
 * @param p_data Ponteiro para o buffer de dados a ser transmitido.
 * @param size Quantidade de bytes a transmitir.
 * @return Status do sistema.
 */
status_t uart_transmit(uint8_t *p_data, uint16_t size);

/**
 * @brief Recebe dados pela UART de forma bloqueante.
 * @param p_data Ponteiro para o buffer onde os dados recebidos serão armazenados.
 * @param size Quantidade de bytes a receber.
 * @return Status do sistema.
 */
status_t uart_receive(uint8_t *p_data, uint16_t size);

/**
 * @brief Envia uma string formatada pela UART.
 * @param format Cadeia de formato (ex: "%d %s").
 * @param ... Argumentos variáveis a serem formatados.
 * @return Status do sistema.
 */
status_t uart_print(const char *format, ...);

#endif /* __AGRISENSE_DRIVER_UART_H__ */
