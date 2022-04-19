/** @file   briter_encoder_can
 *  @brief  Source file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include <briter_encoder_can.h>
#include <stdlib.h>
#include <string.h>

/** @defgroup briter_encoder_rs485 Private Functions
 * @{
 */
static HAL_StatusTypeDef CAN_Tx(Briter_CAN_Handler_t* handler,Briter_CAN_Command_e cmd, uint8_t data_length, uint16_t selection);
/**
 * @}
 */

HAL_StatusTypeDef BRITER_CAN_Init(Briter_CAN_Handler_t* handler,uint8_t address, CAN_HandleTypeDef* hcan){
	memset(handler, 0, sizeof(Briter_CAN_Handler_t));
	if(handler == NULL || hcan == NULL)
		return HAL_ERROR;
	handler->hcan = hcan;
	handler->address = address;
	return HAL_OK;
}

HAL_StatusTypeDef BRITER_CAN_ReadValue(Briter_CAN_Handler_t* handler){
	return CAN_Tx(handler, BRITER_CAN_GET_VALUE, 4, 0);
}

uint32_t BRITER_CAN_GetEncoderValue_Callback(Briter_CAN_Handler_t* handler, uint8_t *pData){
	if(handler->address != pData[1] || pData[0] != 0x07 || pData[2] != BRITER_CAN_GET_VALUE)
		return BRITER_CAN_ERROR;
	handler->position = pData[3] << (0 * 8) | pData[4] << (1 * 8) | pData[5] << (2 * 8)
					| pData[6] << (3 * 8);
	return handler->position;
}

HAL_StatusTypeDef BRITER_CAN_SetBaudrate(Briter_CAN_Handler_t* handler, Briter_CAN_Baudrate_e baudrate){
	return CAN_Tx(handler, BRITER_CAN_SET_BAUDRATE, 4, baudrate);
}

HAL_StatusTypeDef BRITER_CAN_SetAddress(Briter_CAN_Handler_t* handler, uint8_t to_address){
	return CAN_Tx(handler, BRITER_CAN_SET_ID, 4, to_address);
}

HAL_StatusTypeDef BRITER_CAN_SetDataMode(Briter_CAN_Handler_t* handler, Briter_CAN_Mode_e mode){
	return CAN_Tx(handler, BRITER_CAN_SET_MODE, 4, mode);
}

HAL_StatusTypeDef BRITER_CAN_SetReturnTime(Briter_CAN_Handler_t* handler, uint16_t time){
	return CAN_Tx(handler, BRITER_CAN_SET_RETURN_TIME, 4, time);
}

HAL_StatusTypeDef BRITER_CAN_SetZero(Briter_CAN_Handler_t* handler){
	return CAN_Tx(handler, BRITER_CAN_SET_ZERO, 4, 0);
}

/**
 * @brief  Send encoder message via CAN bus.
 * @param  handler pointer encoder handler
 * @param  cmd action to be taken by encoder
 * @param  data_length size of data to be sent
 * @param  selection depends on the action user want to take
 * @retval none
 */
static HAL_StatusTypeDef CAN_Tx(Briter_CAN_Handler_t* handler,Briter_CAN_Command_e cmd, uint8_t data_length, uint16_t selection){
	assert_param(data_length <= 8);
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t txMailbox;
	canTxHeader.DLC = data_length;
	canTxHeader.IDE = CAN_ID_STD;
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.StdId = handler->address;
	canTxHeader.TransmitGlobalTime = DISABLE;
	canTxHeader.ExtId = 0;

	uint8_t i = 0;
	uint8_t* tx_buf = (uint8_t*)malloc(data_length * sizeof(uint8_t));
	tx_buf[i++] = canTxHeader.DLC;
	tx_buf[i++] = canTxHeader.StdId;
	tx_buf[i++] = (uint8_t)cmd;
	if(data_length == 4){
		tx_buf[i++] = (uint8_t)selection;
	}
	else if(data_length == 5){
		tx_buf[i++] = (uint8_t)((selection >> 0) & 0xFF);
		tx_buf[i++] = (uint8_t)((selection >> 8) & 0xFF);
	}
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(handler->hcan, &canTxHeader, tx_buf, &txMailbox);
	free(tx_buf);
	return status;
}
