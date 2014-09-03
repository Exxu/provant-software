/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_co.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_RC
  * \brief Módulo com as principais funcionalidades para operação em modo rádio controlado.
  *
  * Definição do módulo de controle e comunicação via rádio manual.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   10//ms
#define ESC_ON           1
#define SERVO_ON         1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
pv_msg_input iInputData;
pv_msg_controlOutput oControlOutputData; 
/* Inboxes buffers */

/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de RC.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * o controle remoto e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_co_init() 
{

  /* Inicializar os escs*/
  c_common_i2c_init(I2C3);
  c_io_blctrl_init_i2c(I2C3);

  /* Inicializar os servos */
  c_io_rx24f_init(1000000);
  c_common_utils_delayms(2);
  c_io_rx24f_setSpeed(1, 20);
  c_io_rx24f_setSpeed(2, 20);
  c_common_utils_delayms(2);
  /* CCW Compliance Margin e CCW Compliance margin */
  c_io_rx24f_write(1, 0x1A,0x03);
  c_io_rx24f_write(1, 0x1B,0x03);
  c_io_rx24f_write(2, 0x1A,0x03);
  c_io_rx24f_write(2, 0x1B,0x03);
  c_common_utils_delayms(2);
  c_io_rx24f_move(1, 130);
  c_io_rx24f_move(2, 150);
  c_common_utils_delayms(100);

  pv_interface_co.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  pv_interface_co.oControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve 
  * ser adaptado.
  *
  */
void module_co_run() 
{
  unsigned int heartBeat=0;

  while(1) 
  {
    /* Variavel para debug */
    heartBeat+=1;

    /* Passa os valores davariavel compartilha para a variavel iInputData */
    xQueueReceive(pv_interface_co.iInputData, &iInputData, 0);

    /* Leitura do numero de ciclos atuais */
		lastWakeTime = xTaskGetTickCount();

    /* Escrita dos servos */
    #if SERVO_ON
        c_io_rx24f_move(2, 150+10);
        c_io_rx24f_move(1, 130+10);
    #endif

    /* Escrita dos escs */
    #if ESC_ON
      if (heartBeat < 500)
      {
        c_io_blctrl_setSpeed(0, 10 );
        c_common_utils_delayus(10);
        c_io_blctrl_setSpeed(1, 10 );
      }
      else
      {
        c_io_blctrl_setSpeed(0, 15 );
        c_common_utils_delayus(10);
        c_io_blctrl_setSpeed(1, 15 );      
      }
    #endif

    oControlOutputData.actuation.servoPosition[0] = 13.1;
    oControlOutputData.actuation.servoPosition[1] = 13.2;
    oControlOutputData.actuation.escNewtons[0]    = 13.3;
    oControlOutputData.actuation.escNewtons[1]    = 13.4;
    oControlOutputData.actuation.escRpm[0]        = 13.5;
    oControlOutputData.actuation.escRpm[1]        = 13.6;
    oControlOutputData.vantBehavior.rpy[0]        = 7.1;
    oControlOutputData.vantBehavior.rpy[1]        = 7.2;
    oControlOutputData.vantBehavior.rpy[2]        = 7.3;
    oControlOutputData.vantBehavior.drpy[0]       = 8.1;
    oControlOutputData.vantBehavior.drpy[1]       = 8.2;
    oControlOutputData.vantBehavior.drpy[2]       = 8.3;
    oControlOutputData.vantBehavior.xyz[0]        = 3.1;
    oControlOutputData.vantBehavior.xyz[1]        = 3.2;
    oControlOutputData.vantBehavior.xyz[2]        = 3.3;
    oControlOutputData.vantBehavior.dxyz[0]       = 4.1;
    oControlOutputData.vantBehavior.dxyz[1]       = 4.2;
    oControlOutputData.vantBehavior.dxyz[2]       = 4.3;
    oControlOutputData.heartBeat                  = heartBeat;

    if(pv_interface_co.oControlOutputData != 0)
      xQueueOverwrite(pv_interface_co.oControlOutputData, &oControlOutputData);

    /* A thread dorme ate o tempo final ser atingido */
    vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
