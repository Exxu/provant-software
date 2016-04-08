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
#define MODULE_PERIOD	    12//ms
#define USART_BAUDRATE     460800  //<-Bluethood
//#define USART_BAUDRATE     921600 //<-Beaglebone

//#define NONHIL
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType pv_module_do_lastWakeTime;
unsigned int pv_module_do_heartBeat=0;
unsigned int pv_module_do_cicleTime=0;
/* Input Message */
pv_msg_input pv_module_do_InputData;
pv_msg_gps iGpsData;
/* Output Message */
pv_msg_controlOutput pv_module_do_ControlData;
pv_type_actuation pv_module_do_actuation;
pv_type_actuation pv_module_do_auxactuation;
GPIOPin pv_module_do_LED3;

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
  /*Data consumed by the thread*/
  pv_interface_do.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  // pv_interface_do.iGpsData           = xQueueCreate(1, sizeof(pv_msg_gps));
  /*Data produced by the thread*/
  pv_interface_do.oControlData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));

  /*Inicializa o controlador*/
  c_rc_LQR2_control_init();
  c_rc_LQR_servo_control_init();

  /* Pin for debug */
  pv_module_do_LED3 = c_common_gpio_init(GPIOD, GPIO_Pin_13, GPIO_Mode_OUT); //LED3

}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_do_run()
{
	/*Dados usados para comunicaçao*/
	float pv_module_do_com_rpy[3];
	float pv_module_do_com_drpy[3];
	float pv_module_do_com_position[3];
	float pv_module_do_com_velocity[3];
	float pv_module_do_com_alpha[2];
	float pv_module_do_com_dalpha[2];
	float pv_module_do_com_data1[2],pv_module_do_com_data2[2],pv_module_do_com_data3[2];
	int pv_module_do_com_aux[2];
	float pv_module_do_com_aux2[3];
	float pv_module_do_com_servoTorque[2];
	float pv_module_do_com_escForce[2];
	int pv_module_do_com_channel[7];
	int flag=0;

	/*Inicializa dados da comunicaçao*/
	pv_module_do_ControlData.actuation.servoRight = 0;
	pv_module_do_ControlData.actuation.servoLeft  = 0;
	pv_module_do_ControlData.actuation.escRightSpeed = 0;
	pv_module_do_ControlData.actuation.escLeftSpeed  = 0;
	pv_module_do_ControlData.actuation.escRightNewtons = 0;
	pv_module_do_ControlData.actuation.escLeftNewtons  = 0;

	while(1)
	{
		pv_module_do_lastWakeTime = xTaskGetTickCount();
		pv_module_do_heartBeat++;

		if (uxQueueMessagesWaiting(pv_interface_do.iInputData)!=0){
			xQueueReceive(pv_interface_do.iInputData, &pv_module_do_InputData, 0);
		}

		//xQueueReceive(pv_interface_do.iGpsData, &iGpsData, 0);

		pv_module_do_com_aux[0]=pv_module_do_InputData.securityStop;
		pv_module_do_com_aux[1]=0;

		pv_module_do_com_aux2[0]=(float)pv_module_do_InputData.attitude_reference.roll*100.0;
		pv_module_do_com_aux2[1]=(float)pv_module_do_InputData.attitude_reference.pitch*100.0;
		pv_module_do_com_aux2[2]=(float)pv_module_do_InputData.attitude_reference.yaw*100.0;

		pv_module_do_com_rpy[0]=pv_module_do_InputData.attitude.roll;
		pv_module_do_com_rpy[1]=pv_module_do_InputData.attitude.pitch;
		pv_module_do_com_rpy[2]=pv_module_do_InputData.attitude.yaw;

		pv_module_do_com_drpy[0]=pv_module_do_InputData.attitude.dotRoll;
		pv_module_do_com_drpy[1]=pv_module_do_InputData.attitude.dotPitch;
		pv_module_do_com_drpy[2]=pv_module_do_InputData.attitude.dotYaw;

		pv_module_do_com_position[0]=pv_module_do_InputData.position.x;
		pv_module_do_com_position[1]=pv_module_do_InputData.position.y;
		pv_module_do_com_position[2]=pv_module_do_InputData.position.z;

		pv_module_do_com_velocity[0]=pv_module_do_InputData.position.dotX;
		pv_module_do_com_velocity[1]=pv_module_do_InputData.position.dotY;
		pv_module_do_com_velocity[2]=pv_module_do_InputData.position.dotZ;

		pv_module_do_com_alpha[0]=pv_module_do_InputData.servosOutput.servo.alphal;
		pv_module_do_com_alpha[1]=pv_module_do_InputData.servosOutput.servo.alphar;

		pv_module_do_com_dalpha[0]=pv_module_do_InputData.servosOutput.servo.dotAlphal;
		pv_module_do_com_dalpha[1]=pv_module_do_InputData.servosOutput.servo.dotAlphar;

		pv_module_do_com_channel[0]=pv_module_do_InputData.receiverOutput.joystick[0];
		pv_module_do_com_channel[1]=pv_module_do_InputData.receiverOutput.joystick[1];
		pv_module_do_com_channel[2]=pv_module_do_InputData.receiverOutput.joystick[2];
		pv_module_do_com_channel[3]=pv_module_do_InputData.receiverOutput.joystick[3];
		pv_module_do_com_channel[4]=pv_module_do_InputData.receiverOutput.aButton;
		pv_module_do_com_channel[5]=pv_module_do_InputData.receiverOutput.bButton;
		pv_module_do_com_channel[6]=0;

		pv_module_do_com_data1[0]= pv_module_do_ControlData.actuation.servoLeft;
		pv_module_do_com_data1[1]= pv_module_do_ControlData.actuation.servoRight;
		pv_module_do_com_data3[0]= pv_module_do_ControlData.actuation.escLeftNewtons;
		pv_module_do_com_data3[1]= pv_module_do_ControlData.actuation.escLeftSpeed;
		pv_module_do_com_data2[0]= pv_module_do_ControlData.actuation.escRightNewtons;
		pv_module_do_com_data2[1]= pv_module_do_ControlData.actuation.escRightSpeed;


		c_common_datapr_multwii_attitude(pv_module_do_InputData.attitude.roll*RAD_TO_DEG,pv_module_do_InputData.attitude.pitch*RAD_TO_DEG,pv_module_do_InputData.attitude.yaw*RAD_TO_DEG);
		c_common_datapr_multwii2_sendControldatain(pv_module_do_com_rpy,pv_module_do_com_drpy,pv_module_do_com_position,pv_module_do_com_velocity);
		c_common_datapr_multwii2_sendEscdata(pv_module_do_com_aux,pv_module_do_com_alpha,pv_module_do_com_dalpha);
		c_common_datapr_multwii2_sendControldataout(pv_module_do_com_data1,pv_module_do_com_data3,pv_module_do_com_data2);
		c_common_datapr_multwii_debug(pv_module_do_InputData.cicleTime,pv_module_do_InputData.securityStop,0,0);
		//	c_common_datapr_multwii_debug((float)pv_module_do_com_aux2[0],(float)pv_module_do_com_aux2[1],(float)pv_module_do_com_aux2[2],0);

		c_common_datapr_multwii_sendstack(USART2);

		#ifdef	CONTROL_BEAGLE

		/*Receives control input data from the beaglebone*/
		c_common_datapr_multiwii_receivestack(USART2);
		pv_module_do_actuation=c_common_datapr_multwii_getactuation();
		if(pv_module_do_actuation.escLeftNewtons!=0 || pv_module_do_actuation.escRightNewtons!=0 || pv_module_do_actuation.servoLeft!=0 || pv_module_do_actuation.servoRight!=0){
			pv_module_do_auxactuation=pv_module_do_actuation;
		}

		if(pv_module_do_auxactuation.servoLeft<=2 && pv_module_do_auxactuation.servoLeft>=-2)
			pv_module_do_ControlData.actuation.servoLeft=pv_module_do_auxactuation.servoLeft;
		if(pv_module_do_auxactuation.servoRight<=2 && pv_module_do_auxactuation.servoRight>=-2)
			pv_module_do_ControlData.actuation.servoRight=pv_module_do_auxactuation.servoRight;

		pv_module_do_ControlData.actuation.escRightNewtons=pv_module_do_auxactuation.escRightNewtons;
		pv_module_do_ControlData.actuation.escLeftNewtons=pv_module_do_auxactuation.escLeftNewtons;

		if (pv_module_do_InputData.securityStop){
			pv_module_do_ControlData.actuation.servoLeft=0;
			pv_module_do_ControlData.actuation.servoRight=0;
			pv_module_do_ControlData.actuation.escRightNewtons=0;
			pv_module_do_ControlData.actuation.escLeftNewtons=0;
			pv_module_do_auxactuation=pv_module_do_ControlData.actuation;
			pv_module_do_actuation=pv_module_do_ControlData.actuation;
		}
#else
//		pv_module_do_ControlData.actuation=c_rc_LQR2_controller(pv_module_all_InputData.attitude,pv_module_all_InputData.attitude_reference,
//															 pv_module_all_InputData.position,pv_module_all_InputData.position_refrence,
//															 pv_module_all_InputData.servosOutput.servo, pv_module_all_InputData.servosOutput.servo_refrence,
//															 0.012,pv_module_all_InputData.securityStop);
		pv_type_datapr_servos servo_ref;

		servo_ref.alphar=-0.35;
		servo_ref.alphal=0.35;
		servo_ref.dotAlphar=0;
		servo_ref.dotAlphal=0;

		pv_module_do_ControlData.actuation=c_rc_LQR_servo( pv_module_do_InputData.servosOutput.servo, servo_ref,
														0.012,pv_module_do_InputData.securityStop);
		#endif

		if(pv_interface_do.oControlData != 0)
			xQueueOverwrite(pv_interface_do.oControlData, &pv_module_do_ControlData);

		unsigned int timeNow=xTaskGetTickCount();
		pv_module_do_cicleTime = timeNow - pv_module_do_lastWakeTime;

		/* toggle pin for debug */
		c_common_gpio_toggle(pv_module_do_LED3);

		vTaskDelayUntil( &pv_module_do_lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
