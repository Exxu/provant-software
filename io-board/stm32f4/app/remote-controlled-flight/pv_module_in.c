/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_in.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_in
  * \brief Componentes para o sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de input do VANT.
  * Leituras de todos os sensores. O processamento destes
  * dados brutos é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   5//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
char str[256];
GPIOPin LED_builtin_io;
//GPIOPin debugPin;
float attitude_quaternion[4]={1,0,0,0};
int securityStop=0; //Promove uma parada de seguranca - desliga os atuadores
int init=1; //Se 1 entao o UAV está em fase de inicializacao

/* Output Message */
pv_msg_input oInputData;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_in_init() 
{
	/* Inicialização do hardware do módulo */
	LED_builtin_io = c_common_gpio_init(GPIOD, GPIO_Pin_15, GPIO_Mode_OUT);

	/* Inicialização da imu */
	c_common_i2c_init(I2C1); 
	c_io_imu_init(I2C1); 

  /* Inicializador do sonar */
  c_io_sonar_init();

  /* Inicializador do receiver */
	c_rc_receiver_init();

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);

  /* Resevar o espaco para a variavel compartilhada */
	pv_interface_in.oInputData  = xQueueCreate(1, sizeof(pv_msg_input));
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores como necessário.
  *
  */
void module_in_run() 
{
  unsigned int heartBeat=0;
  /////////////////
  bool lock_increment_roll=false, lock_increment_pitch=false, lock_increment_yaw=false, enable_integration=false, lock_increment_z=false;
  	float rpy[6] = {0}, attitude_yaw_initial=0.0f, last_valid_sonar_raw=0.35f, position_reference_initial=0.0f;
  	int iterations=1, channel_flight_mode=0, sample=0;
  	float sonar_raw=0.0f, sonar_raw_real=0.0f, sonar_raw_filter=0.0f, sonar_corrected_debug=0.0f, sonar_corrected=0.0f, sonar_filtered=0.0f, dotZ=0.0f, dotZ_filtered=0.0f;
   	int n_valid_samples=0;
  	long sample_time_gyro_us[1] ={0};
  ////////////////
  	/*Dados usados no sonar*/
  	float k1_1o_10Hz=0.7265, k2_1o_10Hz=0.1367, k3_1o_10Hz=0.1367;
  	float k1_2o_10Hz=1.56102, k2_2o_10Hz=-0.64135, k3_2o_10Hz=0.02008, k4_2o_10Hz=0.04017, k5_2o_10Hz=0.02008;
  	float sonar_raw_k_minus_1=0.0f, sonar_raw_k_minus_2=0.0f, sonar_filtered_k_minus_1=0.0f, sonar_filtered_k_minus_2=0.0f;
  	float dotZ_filtered_k_minus_1=0.0f, dotZ_k_minus_1=0.0f;
  	float last_reference_z=0;
	int valid_sonar_measurements=0;
  	/* Inicializa os dados da attitude*/
  	oInputData.attitude.roll  = 0;
  	oInputData.attitude.pitch = 0;
  	oInputData.attitude.yaw   = 0;
  	oInputData.attitude.dotRoll  = 0;
  	oInputData.attitude.dotPitch = 0;
  	oInputData.attitude.dotYaw   = 0;

  	/* Inicializa os dados da posiçao*/
  	oInputData.position.x = 0;
  	oInputData.position.y = 0;
  	oInputData.position.z = 0;
  	oInputData.position.dotX = 0;
  	oInputData.position.dotY = 0;
  	oInputData.position.dotZ = 0;

  	/*Inicializa as referencias*/
  	oInputData.reference.refx = 0;
  	oInputData.reference.refy = 0;
  	oInputData.reference.refz = 0;
  	oInputData.reference.refdotx = 0;
  	oInputData.reference.refdoty = 0;
  	oInputData.reference.refdotz = 0;

  	oInputData.reference.refroll  = 0;
  	oInputData.reference.refpitch = 0;
  	oInputData.reference.refyaw   = 0;
  	oInputData.reference.refdotRoll  = 0;
  	oInputData.reference.refdotPitch = 0;
  	oInputData.reference.refdotYaw   = 0;

  	while(1)
	{
    oInputData.heartBeat=heartBeat+=1;

    /* toggle pin for debug */
    c_common_gpio_toggle(LED_builtin_io);

    /* Leitura do numero de ciclos atuais */
	lastWakeTime = xTaskGetTickCount();

	/*----------------------Tratamento da IMU---------------------*/
    /* Pega e trata os valores da imu */
	c_io_imu_getRaw(oInputData.imuOutput.accRaw, oInputData.imuOutput.gyrRaw, oInputData.imuOutput.magRaw,sample_time_gyro_us);
	c_datapr_MahonyAHRSupdate(attitude_quaternion,oInputData.imuOutput.gyrRaw[0],oInputData.imuOutput.gyrRaw[1],oInputData.imuOutput.gyrRaw[2],oInputData.imuOutput.accRaw[0],oInputData.imuOutput.accRaw[1],oInputData.imuOutput.accRaw[2],oInputData.imuOutput.magRaw[0],oInputData.imuOutput.magRaw[1],oInputData.imuOutput.magRaw[2],sample_time_gyro_us[0]);
	c_io_imu_Quaternion2Euler(attitude_quaternion, rpy);
	c_io_imu_EulerMatrix(rpy,oInputData.imuOutput.gyrRaw);
	oInputData.imuOutput.sampleTime =xTaskGetTickCount() -lastWakeTime;

    /* Saida dos dados de posição limitada a uma variaçao minima */
    if (abs2(rpy[PV_IMU_ROLL]-oInputData.attitude.roll)>ATTITUDE_MINIMUM_STEP)
    	oInputData.attitude.roll= rpy[PV_IMU_ROLL];
    if (abs2(rpy[PV_IMU_PITCH]-oInputData.attitude.pitch)>ATTITUDE_MINIMUM_STEP)
    	oInputData.attitude.pitch= rpy[PV_IMU_PITCH];
    if (abs2(rpy[PV_IMU_YAW]-oInputData.attitude.yaw)>ATTITUDE_MINIMUM_STEP)
    	oInputData.attitude.yaw= rpy[PV_IMU_YAW];

    /* Saida dos dados da velocidade angular*/
    oInputData.attitude.dotRoll  = rpy[PV_IMU_DROLL];
    oInputData.attitude.dotPitch = rpy[PV_IMU_DPITCH];
    oInputData.attitude.dotYaw   = rpy[PV_IMU_DYAW ];

    /*----------------------Tratamento da Referencia---------------------*/

    /* Realiza a leitura dos canais do radio-controle */
	oInputData.receiverOutput.joystick[0]=c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);
	oInputData.receiverOutput.joystick[1]=c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
	oInputData.receiverOutput.joystick[2]=c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL);
	oInputData.receiverOutput.joystick[3]=c_rc_receiver_getChannel(C_RC_CHANNEL_YAW);
	oInputData.receiverOutput.vrPot		 =c_rc_receiver_getChannel(C_RC_CHANNEL_A);
	oInputData.receiverOutput.bButton    =c_rc_receiver_getChannel(C_RC_CHANNEL_B);
	oInputData.receiverOutput.sampleTime =xTaskGetTickCount();

	/*Referencia de attitude*/
	oInputData.reference.refroll  = ((float)oInputData.receiverOutput.joystick[2]/100)*REF_ROLL_MAX+REF_ROLL_BIAS;
	oInputData.reference.refpitch = ((float)oInputData.receiverOutput.joystick[1]/100)*REF_PITCH_MAX+REF_PITCH_BIAS;
	oInputData.reference.refyaw   = attitude_yaw_initial;// + REF_YAW_MAX*channel_YAW/100;

	/*Referencia de altitude*/
	//Se o canal 3 esta ligado ele muda a referencia de altura se nao esta ligado fica na referencia pasada
	// Trothel varia de -100 a 100 -> adiciono 100 para ficar 0-200 e divido para 200 para ficar 0->1
	if (oInputData.receiverOutput.joystick[3]){
		oInputData.reference.refz=(((float)oInputData.receiverOutput.joystick[0]+100)/200)*HEIGHT_REFERENCE_MAX;
		last_reference_z = oInputData.reference.refz;
	}
	else
		oInputData.reference.refz = last_reference_z;

	/*----------------------Tratamento do Sonar---------------------*/
	/* Executa a leitura do sonar */
	oInputData.sonarOutput.altitude  =c_io_sonar_read();
	sonar_raw= sonar_raw_real/100;
    oInputData.sonarOutput.sampleTime=xTaskGetTickCount() - lastWakeTime;
    oInputData.cicleTime             =xTaskGetTickCount() - lastWakeTime;

	#ifdef LIMIT_SONAR_VAR
		if ( ( (oInputData.reference.refz-SONAR_MAX_VAR)<sonar_raw && (oInputData.reference.refz+SONAR_MAX_VAR)>sonar_raw ) || init){
			sonar_corrected = (sonar_raw)*cos(oInputData.attitude.roll)*cos(oInputData.attitude.pitch);//the altitude must be in meters
		}
	#else
		sonar_corrected = (sonar_raw)*cos(oInputData.attitude.roll)*cos(oInputData.attitude.pitch);;
	#endif
	/*Filtrajem das amostras do sonar*/
	#ifdef SONAR_FILTER_1_ORDER_10HZ
		//1st order filter with fc=10Hz
		sonar_filtered = k1_1o_10Hz*sonar_filtered_k_minus_1 + k2_1o_10Hz*sonar_corrected + k3_1o_10Hz*sonar_raw_k_minus_1;
		// Filter memory
		sonar_raw_k_minus_1 = sonar_corrected;
		sonar_filtered_k_minus_1 = sonar_filtered;
	#elif defined SONAR_FILTER_2_ORDER_10HZ
		//1st order filter with fc=10Hz
		sonar_filtered = k1_2o_10Hz*sonar_filtered_k_minus_1 + k2_2o_10Hz*sonar_filtered_k_minus_2 + k3_2o_10Hz*sonar_corrected + k4_2o_10Hz*sonar_raw_k_minus_1 + k5_2o_10Hz*sonar_raw_k_minus_2;
		// Filter memory
		sonar_raw_k_minus_2 = sonar_raw_k_minus_1;
		sonar_raw_k_minus_1 = sonar_corrected;
		sonar_filtered_k_minus_2 = sonar_filtered_k_minus_1;
		sonar_filtered_k_minus_1 = sonar_filtered;
	#else //If no filter is active, the result is the measurement
		sonar_filtered = sonar_corrected;
	#endif

	// Derivada = (dado_atual-dado_anterior )/(tempo entre medicoes) - fiz a derivada do sinal filtrado, REVER
	dotZ = (sonar_filtered - oInputData.position.z)/0.005;
	// 1st order filter with fc=10Hz
	dotZ_filtered = k1_1o_10Hz*dotZ_filtered_k_minus_1 + k2_1o_10Hz*dotZ + k3_1o_10Hz*dotZ_k_minus_1;
	// Filter memory
	dotZ_filtered_k_minus_1 = dotZ_filtered;
	dotZ_k_minus_1 = dotZ;

	//Filtered measurements
	oInputData.position.z = sonar_filtered;
	oInputData.position.dotZ = dotZ_filtered;
	/*----------------------Init-------------------------------------*/
    //Falta resolver

    /* toggle pin for debug */
    //c_common_gpio_toggle(LED_builtin_io);

    /* Realiza o trabalho de mutex */
	if(pv_interface_in.oInputData != 0)
		xQueueOverwrite(pv_interface_in.oInputData, &oInputData);

    /* A thread dorme ate o tempo final ser atingido */
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



