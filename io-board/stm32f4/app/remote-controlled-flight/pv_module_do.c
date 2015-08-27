/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_do.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo de transmissao de dados para fora do ARM.
  ******************************************************************************/

      /* Includes ------------------------------------------------------------------*/
#include "pv_module_do.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_do
  * \brief Módulo responsavel por transmitir dados.
  *
  * Definição do módulo de transmissão de dados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	    10//ms
#define USART_BAUDRATE     460800
#define NONHIL
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
unsigned int heartBeat=0;
pv_msg_input iInputData;
pv_msg_gps iGpsData;
pv_msg_controlOutput iControlOutputData;
float data1[2];
float data2[2];
float data3[2];
GPIOPin LED3;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_do_init() 
{
  /* Inicia a usart2 */
  c_common_usart2_init(USART_BAUDRATE);

  /* Reserva o local de memoria compartilhado */
  pv_interface_do.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
 // pv_interface_do.iGpsData           = xQueueCreate(1, sizeof(pv_msg_gps));
 // pv_interface_do.iControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));

  /* Pin for debug */
  LED3 = c_common_gpio_init(GPIOD, GPIO_Pin_13, GPIO_Mode_OUT); //LED3

}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_do_run()
{
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;

		xQueueReceive(pv_interface_do.iInputData, &iInputData, 0);
		//xQueueReceive(pv_interface_do.iGpsData, &iGpsData, 0);
		//xQueueReceive(pv_interface_do.iControlOutputData, &iControlOutputData, 0);

		arm_scale_f32(iInputData.imuOutput.accRaw,RAD_TO_DEG,iInputData.imuOutput.accRaw,3);
		arm_scale_f32(iInputData.imuOutput.gyrRaw,RAD_TO_DEG,iInputData.imuOutput.gyrRaw,3);
		int channel[]={iInputData.receiverOutput.joystick[0],iInputData.receiverOutput.joystick[1],iInputData.receiverOutput.joystick[2],iInputData.receiverOutput.joystick[3],iInputData.receiverOutput.aButton,iInputData.receiverOutput.bButton,iInputData.receiverOutput.vrPot};
		# ifdef NONHIL
		//c_common_datapr_multwii_raw_imu(iInputData.imuOutput.accRaw,iInputData.imuOutput.gyrRaw,iInputData.imuOutput.magRaw);
		//c_common_datapr_multwii_attitude(iInputData.attitude.roll*RAD_TO_DEG*10,iInputData.attitude.pitch*RAD_TO_DEG*10,iInputData.attitude.yaw*RAD_TO_DEG*10);
		//c_common_datapr_multwii_attitude(iGpsData.heartBeat,iGpsData.gpsOutput.lat,iGpsData.gpsOutput.lon);
		//c_common_datapr_multwii2_rcNormalize(channel);
		//c_common_datapr_multwii_altitude(iInputData.position.z,iInputData.position_refrence.z*100);
		c_common_datapr_multwii_debug(iInputData.servoLeft.angle*180/PI,iInputData.servoLeft.angularSpeed*180/PI,iInputData.servoRight.angle*180/PI,iInputData.servoRight.angularSpeed*180/PI);
		//c_common_datapr_multwii_debug(iInputData.servoLeft.torque,iInputData.servoLeft.torque,0,0);
		c_common_datapr_multwii_sendstack(USART2);

//        data1[0]=iControlOutputData.actuation.servoLeft*RAD_TO_DEG;
//        data1[1]=iControlOutputData.actuation.servoRight*RAD_TO_DEG;
//        //data1[0]=iGpsData.gpsOutput.lat;
//        //data1[1]=iGpsData.gpsOutput.lon;
//        data2[0]=iControlOutputData.actuation.escLeftSpeed;
//        data2[1]=iControlOutputData.actuation.escRightSpeed;
//        data3[0]=iInputData.attitude_reference.roll*RAD_TO_DEG;
//        data3[1]=iInputData.attitude_reference.pitch*RAD_TO_DEG;


		//c_common_datapr_multwii2_sendControldatain(iControlOutputData.actuation.servoLeftvantBehavior.rpy, iControlOutputData.vantBehavior.drpy, iControlOutputData.vantBehavior.xyz, iControlOutputData.vantBehavior.dxyz);
		//c_common_datapr_multwii2_sendControldataout(data1,data3,data2);
		//c_common_datapr_multwii_sendstack(USART2);
		#else
		c_common_datapr_multwii_attitude(5,2,3);
		c_common_datapr_multwii_altitude(-4,-5);
		c_common_datapr_multwii_sendstack(USART2);

		data1[0]=6.6;
		data1[1]=7.7;
		data2[0]=8.8;
		data2[1]=9.9;
		data3[0]=10.10;
		data3[1]=11.11;

		c_common_datapr_multwii2_sendControldataout(data1,data3,data2);
		c_common_datapr_multwii_sendstack(USART2);
		#endif
		/* toggle pin for debug */
		c_common_gpio_toggle(LED3);

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
