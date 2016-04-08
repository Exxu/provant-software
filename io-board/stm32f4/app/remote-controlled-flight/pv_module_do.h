/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_do.h
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo de transmissao de dados para fora do ARM.
  ******************************************************************************/

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_DO_H
#define PV_MODULE_DO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_common_i2c.h"
#include "c_common_uart.h"
#include "c_common_utils.h"
#include "pv_typedefs.h"
#include "c_io_imu.h"

#include "c_datapr_MultWii.h"

/*Control includes*/
#include "../../base/modules/rc/c_rc_LQR2_control.h"
#include "../../base/modules/rc/c_rc_LQR_servos.h"


/* Exported types ------------------------------------------------------------*/
struct pv_interface_do 
{
  xQueueHandle iInputData;  
  xQueueHandle oControlData;
  xQueueHandle iGpsData;  
} pv_interface_do;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_do_init();
void module_do_run();

#ifdef __cplusplus
}
#endif

#endif 
