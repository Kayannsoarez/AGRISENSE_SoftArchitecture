/**
 ******************************************************************************
 * @file       : agrisense_app_log.C
 * @author     : Kayann Soares
 * @version	: V0.1
 * @date       : 28/02/2025
 * @brief      : Source file of APP for device critical error logs
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "agrisense_app_log.h"
#include "agrisense_driver_uart.h"
#include "stm32l4xx_hal.h"

/* Handlers ------------------------------------------------------------------*/

/**
 * @brief Handler da interface I2C.
 */
static agrisense_log_instance_t g_log_instance;

/* Static Functions ----------------------------------------------------------*/

/**
 * @brief Atualiza a estrutura de controle do log na flash
 *
 * @param p_config Ponteiro para a estrutura de controle
 * @return Status do sistema
 */
static status_t agrisense_update_log_config_flash(agrisense_log_control_t *p_config)
{
	status_t status = agrisense_flash_erase_sector(AGRISENSE_LOG_START_ADDR);
	if (status != SYS_SUCCESS)
		return status;

	return agrisense_flash_write(AGRISENSE_LOG_START_ADDR, (uint8_t*)p_config, sizeof(*p_config));
}

/**
 * @brief Grava configurações do log na flash
 *
 * @param p_config Ponteiro para estrutura de controle
 * @return Status do sistema
 */
static status_t agrisense_write_log_config(agrisense_log_control_t *p_config)
{
	return agrisense_update_log_config_flash(p_config);
}

/**
 * @brief Lê configurações do log da flash
 *
 * @param p_config Ponteiro para estrutura de controle
 * @return Status do sistema
 */
static status_t agrisense_read_log_config(agrisense_log_control_t *p_config)
{
	if (p_config == NULL)
		return SYS_INVALID_PARAMETER;

	return agrisense_flash_read(AGRISENSE_LOG_START_ADDR, (uint8_t*)p_config, sizeof(*p_config));
}

/**
 * @brief Calcula o próximo endereço de gravação no log
 *
 * @param start_addr Endereço atual
 * @param p_log_instance Ponteiro para instância do log
 * @return Próximo endereço válido
 */
static uint32_t agrisense_get_next_addr(uint32_t start_addr, agrisense_log_instance_t *p_log_instance)
{
	uint32_t first_entry_addr = AGRISENSE_LOG_START_ADDR + sizeof(p_log_instance->ctrl_flags);
	uint32_t last_log_addr = first_entry_addr + (AGRISENSE_LOG_N_ENTRIES - 1) * sizeof(p_log_instance->last_entry);
	uint32_t next_addr = start_addr + sizeof(p_log_instance->last_entry);

	if ((next_addr > last_log_addr) || (next_addr < first_entry_addr))
		next_addr = first_entry_addr;

	return next_addr;
}

/**
 * @brief Calcula o endereço anterior de leitura no log
 *
 * @param start_addr Endereço atual
 * @param p_log_instance Ponteiro para instância do log
 * @return Endereço anterior válido
 */
static uint32_t agrisense_get_previous_addr(uint32_t start_addr, agrisense_log_instance_t *p_log_instance)
{
	uint32_t first_entry_addr = AGRISENSE_LOG_START_ADDR + sizeof(p_log_instance->ctrl_flags);
	uint32_t last_log_addr = first_entry_addr + (AGRISENSE_LOG_N_ENTRIES - 1) * sizeof(p_log_instance->last_entry);
	uint32_t previous_addr = start_addr - sizeof(p_log_instance->last_entry);

	if ((previous_addr < first_entry_addr) || (previous_addr > last_log_addr))
		previous_addr = last_log_addr;

	return previous_addr;
}


/**
 * @brief Inicializa o sistema de log
 *
 * @param p_log_instance Instância do log
 * @return Status do sistema
 */
status_t agrisense_log_init(agrisense_log_instance_t *p_log_instance)
{
	status_t status = agrisense_read_log_config(&p_log_instance->ctrl_flags);
	if (status != SYS_SUCCESS)
		return status;

	if (p_log_instance->ctrl_flags.is_log_valid != AGRISENSE_FLAG_LOG_VALID)
	{
		p_log_instance->ctrl_flags.is_log_valid = AGRISENSE_FLAG_LOG_VALID;
		p_log_instance->ctrl_flags.last_error_handled = AGRISENSE_FLAG_ERROR_HANDLED;
		p_log_instance->ctrl_flags.write_ptr = sizeof(p_log_instance->ctrl_flags);
		memset(&p_log_instance->last_entry, 0, sizeof(p_log_instance->last_entry));

		return agrisense_write_log_config(&p_log_instance->ctrl_flags);
	}
	else
	{
		agrisense_log_entry_t entry_temp;
		status = agrisense_log_read(p_log_instance, &entry_temp, 1);
		if (status == SYS_SUCCESS)
			memcpy(&p_log_instance->last_entry, &entry_temp, sizeof(agrisense_log_entry_t));
		return status;
	}
}

/**
 * @brief Lê dados diretamente da flash
 *
 * @param address Endereço na flash
 * @param buffer Buffer de destino
 * @param size Quantidade de bytes
 * @return Status do sistema
 */
status_t agrisense_flash_read(uint32_t address, uint8_t *buffer, uint32_t size)
{
	if (address < AGRISENSE_LOG_START_ADDR || (address + size) > (AGRISENSE_LOG_START_ADDR + AGRISENSE_LOG_N_ENTRIES * sizeof(agrisense_log_entry_t)))
		return SYS_INVALID_PARAMETER;

	memcpy(buffer, (const void*)address, size);
	return SYS_SUCCESS;
}

/**
 * @brief Grava dados na flash usando escrita por doubleword
 *
 * @param address Endereço de destino
 * @param data Dados a serem escritos
 * @param size Tamanho dos dados
 * @return Status do sistema
 */
status_t agrisense_flash_write(uint32_t address, const uint8_t *data, uint32_t size)
{
	HAL_FLASH_Unlock();
	for (uint32_t i = 0; i < size; i += 8)
	{
		uint64_t data64 = 0xFFFFFFFFFFFFFFFF;
		memcpy(&data64, data + i, (size - i >= 8) ? 8 : (size - i));
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data64) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return SYS_ERROR;
		}
	}
	HAL_FLASH_Lock();
	return SYS_SUCCESS;
}

/**
 * @brief Apaga uma página da flash interna
 *
 * @param address Endereço dentro da página
 * @return Status do sistema
 */
status_t agrisense_flash_erase_sector(uint32_t address)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t pageError;

	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Page = (address - 0x08000000) / AGRISENSE_FLASH_PAGE_SIZE;
	eraseInit.NbPages = 1;

	if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return SYS_ERROR;
	}

	HAL_FLASH_Lock();
	return SYS_SUCCESS;
}

/**
 * @brief Lê entradas do log da flash
 *
 * @param p_log_instance Instância do log
 * @param p_entry_buffer Buffer de destino
 * @param n_entries Número de entradas a ler
 * @return Status do sistema
 */
status_t agrisense_log_read(agrisense_log_instance_t *p_log_instance, agrisense_log_entry_t *p_entry_buffer, uint32_t n_entries)
{
	status_t status = agrisense_read_log_config(&p_log_instance->ctrl_flags);
	if (status != SYS_SUCCESS)
		return status;

	if (p_log_instance->ctrl_flags.is_log_valid == AGRISENSE_FLAG_LOG_VALID)
	{
		uint32_t entry_addr = agrisense_get_previous_addr(p_log_instance->ctrl_flags.write_ptr, p_log_instance);

		for (int i = n_entries - 1; i >= 0; i--)
		{
			agrisense_log_entry_t temp_entry;
			status = agrisense_flash_read(entry_addr, (uint8_t*)&temp_entry, sizeof(temp_entry));
			if (status != SYS_SUCCESS)
				return status;
			memcpy(&p_entry_buffer[i], &temp_entry, sizeof(temp_entry));
			entry_addr = agrisense_get_previous_addr(entry_addr, p_log_instance);
		}
	}
	else
	{
		return LOG_INVALID;
	}

	return SYS_SUCCESS;
}

/**
 * @brief Escreve uma nova entrada no log
 *
 * @param p_log_instance Instância do log
 * @param error_code Código do erro
 * @return Status do sistema
 */
status_t agrisense_log_write(agrisense_log_instance_t *p_log_instance, status_t error_code)
{
	uint32_t error_write_addr = p_log_instance->ctrl_flags.write_ptr;

	p_log_instance->last_entry.type = AGRISENSE_LOG_TYPE_ERROR;
	p_log_instance->last_entry.sequencial++;
	p_log_instance->last_entry.time = HAL_GetTick();
	p_log_instance->ctrl_flags.write_ptr = agrisense_get_next_addr(error_write_addr, p_log_instance);
	p_log_instance->ctrl_flags.last_error_handled = AGRISENSE_FLAG_ERROR_HANDLED;

	status_t status = agrisense_write_log_config(&p_log_instance->ctrl_flags);
	if (status != SYS_SUCCESS)
		return status;

	return agrisense_flash_write(error_write_addr, (uint8_t*)&p_log_instance->last_entry, sizeof(p_log_instance->last_entry));
}

/**
 * @brief Escreve uma nova entrada de leitura de sensores no log.
 *
 * @param sensor_id ID do sensor.
 * @param var_count Número de variáveis registradas.
 * @param p_vars Ponteiro para array contendo os valores das variáveis.
 * @return Status do sistema
 */
status_t agrisense_log_write_sensors(uint8_t sensor_id, uint8_t var_count, const float *p_vars)
{
	if (!p_vars || var_count == 0 || var_count > 6) return SYS_INVALID_PARAMETER;

	agrisense_log_entry_t entry;
	entry.time = HAL_GetTick();
	entry.sequencial = g_log_instance.last_entry.sequencial + 1;
	entry.type = AGRISENSE_LOG_TYPE_SENSOR_GENERIC;
	entry.sensor_id = sensor_id;
	entry.var_count = var_count;
	memcpy(entry.variables, p_vars, var_count * sizeof(float));

	uint32_t addr = g_log_instance.ctrl_flags.write_ptr;
	g_log_instance.ctrl_flags.write_ptr = agrisense_get_next_addr(addr, &g_log_instance);

	status_t status = agrisense_write_log_config(&g_log_instance.ctrl_flags);
	if (status != SYS_SUCCESS) return status;

	return agrisense_flash_write(addr, (uint8_t*)&entry, sizeof(entry));
}

/**
 * @brief Imprime as últimas entradas de sensores gravadas no log.
 *
 * @param n Número de entradas a serem impressas.
 */
void agrisense_log_print_last_sensor_entries(uint32_t n)
{
	agrisense_log_entry_t buffer[n];
	if (agrisense_log_read(&g_log_instance, buffer, n) != SYS_SUCCESS) return;

	for (int i = n - 1; i >= 0; i--)
	{
		if (buffer[i].type != AGRISENSE_LOG_TYPE_SENSOR_GENERIC) continue;
		uart_print("[Sensor %02X] @ %llums | ", buffer[i].sensor_id, buffer[i].time);
		for (int j = 0; j < buffer[i].var_count; j++)
		{
			uart_print("%.2f ", buffer[i].variables[j]);
		}
		uart_print("\r\n");
	}
}

