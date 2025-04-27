/**
 ******************************************************************************
 * @file       : agrisense_app_log.h
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Header of APP for Log Erros
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AGRISENSE_APP_LOG_H__
#define __AGRISENSE_APP_LOG_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "agrisense_app_syserror.h"

/* Define -------------------------------------------------------------------*/

/**
 * @brief Definição de quantas entradas de log serão salvas
 *
 */
#define AGRISENSE_LOG_N_ENTRIES               32
#define AGRISENSE_LOG_PAYLOAD_MAX_SIZE        32

/**
 * @brief Definição dos valores das flags de controle
 *
 */
#define AGRISENSE_FLAG_LOG_VALID              0xAC
#define AGRISENSE_FLAG_ERROR_HANDLED          0xCA

/**
 * @brief Definição dos valores dos Endereços da Memória
 *
 */
#define AGRISENSE_FLASH_PAGE_SIZE             0x800  // 2KB
#define AGRISENSE_LOG_START_ADDR              ((uint32_t)0x0803F800)


/**
 * @brief Tipos de entradas que podem ser armazenadas no log
 */
typedef enum agrisense_log_type{
	AGRISENSE_LOG_TYPE_ERROR = 0x00,
	AGRISENSE_LOG_TYPE_SENSOR_GENERIC = 0x10,
} agrisense_log_type_t;

/**
 * @brief Estrutura para uma entrada de log na memória
 *
 */
typedef struct __attribute__((packed)) agrisense_log_entry {
	uint64_t time;
	uint32_t sequencial;
	agrisense_log_type_t type;
	uint8_t sensor_id;
	uint8_t var_count;
	float variables[6];
} agrisense_log_entry_t;

/**
 * @brief Estrutura para controle do log armazenado na memória
 *
 */
typedef struct __attribute__((packed)) agrisense_log_control {
	uint8_t is_log_valid;
	uint8_t last_error_handled;
	uint32_t write_ptr;
} agrisense_log_control_t;

/**
 * @brief Estrutura para instância de controle do log armazenado na memória
 *
 */
typedef struct __attribute__((packed)) agrisense_log_instance {
	agrisense_log_entry_t last_entry;
	agrisense_log_control_t ctrl_flags;
} agrisense_log_instance_t;

/* Prototype Functions -------------------------------------------------------------------*/
status_t agrisense_log_init (agrisense_log_instance_t *p_log_instance);
status_t agrisense_log_read (agrisense_log_instance_t *p_log_instance, agrisense_log_entry_t *p_entry_buffer, uint32_t n_entries);
status_t agrisense_log_write (agrisense_log_instance_t *p_log_instance, status_t error_code);
status_t agrisense_log_write_sensors(uint8_t sensor_id, uint8_t var_count, const float *p_vars);
void agrisense_log_print_last_sensor_entries(uint32_t n);

// Funções internas para flash
status_t agrisense_flash_read(uint32_t address, uint8_t *buffer, uint32_t size);
status_t agrisense_flash_write(uint32_t address, const uint8_t *data, uint32_t size);
status_t agrisense_flash_erase_sector(uint32_t address);

#endif /* __AGRISENSE_APP_LOG_H__ */
