/**
 * @file disp.h
 * 
 */

#ifndef DISP_H
#define DISP_H

/*********************
 *      INCLUDES
 *********************/
//
#include <stm32f4xx.h>
#include <stm32f469xx.h>
#include <stdint.h>
//#include "lvgl/lvgl.h"
#include "lvgl/src/lv_misc/lv_color.h"
#include "lvgl/src/lv_misc/lv_area.h"
#include "Utilities/STM32F469I-Discovery/stm32469i_discovery.h"
#include "Utilities/STM32F469I-Discovery/stm32469i_discovery_lcd.h"
#include "Utilities/STM32F469I-Discovery/stm32469i_discovery_sdram.h"

HAL_StatusTypeDef HAL_LTDCEx_StructInitFromVideoConfig(LTDC_HandleTypeDef* hltdc, DSI_VidCfgTypeDef *VidCfg);
/*********************
 *      DEFINES
 *********************/
#define TFT_HOR_RES 800
#define TFT_VER_RES 480
//#define HAL_LTDC_MODULE_ENABLED
#define TFT_EXT_FB		1		/*Frame buffer is located into an external SDRAM*/
#define TFT_USE_GPU		1		/*Enable hardware accelerator*/
//#define STM32F469xx
//#define DSI
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void tft_init(void);

/**********************
 *      MACROS
 **********************/

#endif
