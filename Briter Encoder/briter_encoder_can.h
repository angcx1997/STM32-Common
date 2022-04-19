/**
 ******************************************************************************
 * @file    briter_encoder_can.h
 * @author  Ang Chin Xian
 * @brief   Briter encoder CAN Driver.
 *
 * ## Main functionality
 * 	+ Initialization functions
 * 	+ Configuration functions
 * 	+ Read and get function
 *
 *## How to use this driver
 *-# Create handler to hold Briter_CAN_Handler_t and initialize using BRITER_CAN_Init()
 *-# Make sure baudrate and address is correct
 * 	 - Default encoder address is 1 and baudrate is 9500kbps if no configure
 *-# For reading encoder value,
 *	 - Encoder value is depends on the hardware itself
 *	 - Call BRITER_CAN_ReadValue()
 *	 - In HAL_CAN_RxFifo0MsgPendingCallback()
 *	 	- Call HAL_CAN_GetRxMessage()
 *	 	- If HAL_OK, call BRITER_CAN_GetEncoderValue_Callback() to read the encoder position
 *
 */

#ifndef BRITER_ENCODER_CAN_H_
#define BRITER_ENCODER_CAN_H_

#include "stm32f4xx_hal.h"

/** Used to indicate error when incorrect reception occur*/
#define BRITER_CAN_ERROR	0xFFFFFFFF

/** @name Encoder Characteristic
 */
/**@{*/
#define BRITER_CAN_PPR			4096		//Pulse per revolution
#define BRITER_CAN_NO_OF_TURN	24			//Multi turn number
#define BRITER_CAN_MAX_VALUE	(BRITER_CAN_PPR * BRITER_CAN_NO_OF_TURN)
/**@}*/

/** Briter CAN handler*/
typedef struct
{
  CAN_HandleTypeDef*    hcan;
  uint8_t address;
  uint32_t position;			/*!<Preprocessed encoder position, (24turn * 4096ppr)*/
 }Briter_CAN_Handler_t;

/** Briter CAN Command Selection */
typedef enum {
    BRITER_CAN_GET_VALUE = 0x01,
    BRITER_CAN_SET_ID,
    BRITER_CAN_SET_BAUDRATE,
    BRITER_CAN_SET_MODE,
    BRITER_CAN_SET_RETURN_TIME,
    BRITER_CAN_SET_ZERO,
} Briter_CAN_Command_e;

/** Briter CAN Baudrate Selection */
typedef enum {
    BRITER_CAN_BAUDRATE_500K = 0x00,
    BRITER_CAN_BAUDRATE_1000K,
    BRITER_CAN_BAUDRATE_250K,
    BRITER_CAN_BAUDRATE_125K,
    BRITER_CAN_BAUDRATE_100K,
} Briter_CAN_Baudrate_e;


/** Briter CAN Mode Selection */
typedef enum {
    RS485_ENC_MODE_QUERY = 0x00,
    RS485_ENC_MODE_BACKHAUL,
} Briter_CAN_Mode_e;

/**
* @brief  Initialize encoder handler.
* @param  handler: encoder handler
* @param  address: encoder address
* @param  huart: uart handler
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_CAN_Init(Briter_CAN_Handler_t* handler,uint8_t address, CAN_HandleTypeDef* hcan);

/**
* @brief  Read encoder value.
* @param  handler: encoder handler to give address and store encoder return value
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_CAN_ReadValue(Briter_CAN_Handler_t* handler);

/**
* @brief  Read encoder value.
* @param  handler: encoder handler to give address and store encoder return value
* @param pData pointer to receive data buffer
* @retval HAL status
*/
uint32_t BRITER_CAN_ReadValue_Callback(Briter_CAN_Handler_t* handler, uint8_t *pData);

/**
* @brief  BRITER_CAN_Callback.
* @param  handler: encoder handler
* @retval HAL status
* @note   Use HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
*/
HAL_StatusTypeDef BRITER_CAN_Callback(Briter_CAN_Handler_t* handler, uint8_t *pData);

/**
* @brief Set encoder baudrate.
* @param  handler: encoder handler
* @param  baudrate: refer to ::Briter_CAN_Baudrate_e
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_CAN_SetBaudrate(Briter_CAN_Handler_t* handler, Briter_CAN_Baudrate_e baudrate);

/**
* @brief Change encoder address.
* @param  handler: encoder handler
* @param  to_address: the address you want to change to
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_CAN_SetAddress(Briter_CAN_Handler_t* handler, uint8_t to_address);

/**
* @brief Set data mode to query or backhaul
* @param  handler: encoder handler
* @param  mode : refer to ::Briter_CAN_Mode_e
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_CAN_SetDataMode(Briter_CAN_Handler_t* handler, Briter_CAN_Mode_e mode);

/**
* @brief Set current position as zero
* @param  handler: encoder handler
* @retval HAL status
*/
HAL_StatusTypeDef BRITER_CAN_SetZero(Briter_CAN_Handler_t* handler);

/**
* @brief Set encoder return value
* @param  handler: encoder handler
* @param  time : range from 0-65535 (ms)
* @retval HAL status
* @note Default 50ms.  once the automatic return time is set to less than
* 	20 milliseconds, the encoder will not be
* 	able to set other parameters, use with caution!
*/
HAL_StatusTypeDef BRITER_CAN_SetReturnTime(Briter_CAN_Handler_t* handler, uint16_t time);

//Paste this under Rx interrupt function to sort incoming messages 
/*
HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, incoming); // change FIFO accordingly
switch(incoming[1]){
	case 0x04:
		ENCODER_Sort_Incoming(incoming, &hEncoderLeftPull);
		break;
	case 0x02:
		ENCODER_Sort_Incoming(incoming, &hEncoderLeftTurn);
		break;
	case 0x03:
		ENCODER_Sort_Incoming(incoming, &hEncoderRightPull);
		break;
	case 0x01:
		ENCODER_Sort_Incoming(incoming, &hEncoderRightTurn);
		break;
}
*/

#endif
