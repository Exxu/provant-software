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
#define MODULE_PERIOD	   12//ms

//
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	portTickType pv_module_in_WakeTime, pv_module_in_lastWakeTime;
	float pv_module_in_attitude_quaternion[4]={1,0,0,0};
	char str[256];
	GPIOPin pv_module_in_LED4;
	pv_type_actuation pv_module_in_actuation;
	pv_type_actuation pv_module_in_actuation_servos;
	pv_type_actuation pv_module_in_auxactuation;

	/* Output Message */
	pv_msg_input pv_module_in_InputData;

	/* Input Message */
	pv_msg_controlOutput pv_module_in_ControlData;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
	unsigned char pv_module_in_setPointESC_Forca(float forca);
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
	pv_module_in_LED4 = c_common_gpio_init(GPIOD, GPIO_Pin_12, GPIO_Mode_OUT); //LED4

	/* Inicialização da imu */
	c_common_i2c_init(I2C1);
	c_io_imu_init(I2C1);

	/* Inicializador do sonar */
	c_io_sonar_init();

	/* Inicializador do receiver */
	c_rc_receiver_init();

	/* Inicializar os escs*/
	c_common_i2c_init(I2C3);
	c_io_blctrl_init_i2c(I2C3);

	/* Inicializador do servos */
	c_io_servos_init();

	/* Reserva o local de memoria compartilhado */
	/*Data consumed by the thread*/
	pv_interface_in.iControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
	/*Data produced by the thread*/
	pv_interface_in.oInputData  = xQueueCreate(1, sizeof(pv_msg_input));
	/* Pin for debug */
	//debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);



	pv_module_in_InputData.init=1;
	pv_module_in_InputData.securityStop=0;
	pv_module_in_InputData.flightmode=0;
	pv_module_in_lastWakeTime=0;
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
	unsigned int pv_module_in_heartBeat=0;
	unsigned int aux_time;
	/*Dados usados para IMU*/
	float pv_module_in_rpy[6] = {0};
	float pv_module_in_barometer[2]={0};
	float pv_module_in_temperature=0;
	long  pv_module_in_pressure=0;

	/*Dadosa usados para Calibrar IMU*/
	float  attitude_yaw_initial=0.0f, attitude_pitch_initial=0.0f,attitude_roll_initial=0.0f;
	float  yaw_aux, pitch_aux, roll_aux;

	/*Dados gerais*/
	int iterations=1, channel_flight_mode=0;
	long sample_time_gyro_us[1] ={0};
	bool pv_module_in_aButton_ant=0;
	int pv_module_cont=0;

	/*Dados usados no sonar*/
	float last_valid_sonar_raw=0.35f;
	float k1_1o_10Hz=0.7265, k2_1o_10Hz=0.1367, k3_1o_10Hz=0.1367;
	float k1_2o_10Hz=1.56102, k2_2o_10Hz=-0.64135, k3_2o_10Hz=0.02008, k4_2o_10Hz=0.04017, k5_2o_10Hz=0.02008;
	float sonar_raw_k_minus_1=0.0f, sonar_raw_k_minus_2=0.0f, sonar_filtered_k_minus_1=0.0f, sonar_filtered_k_minus_2=0.0f;
	float dotZ_filtered_k_minus_1=0.0f, dotZ_k_minus_1=0.0f;
	float last_reference_z=0;
	int valid_sonar_measurements=0, sample=0;
	float sonar_raw=0.0f, sonar_raw_real=0.0f, sonar_raw_filter=0.0f, sonar_corrected_debug=0.0f, sonar_corrected=0.0f, sonar_filtered=0.0f, dotZ=0.0f, dotZ_filtered=0.0f;
	float servo_filter_right=0, servo_filter_left=0;


	/*Dados usados nos servos*/
	pv_type_datapr_servos pv_module_in_servo_real;
	pv_type_datapr_servos pv_module_in_servo_actual;
	pv_type_datapr_servos pv_module_in_servo_last;
	pv_type_datapr_servos pv_module_in_servo_filtered;
	float servo_r_raw_k_minus_1=0.0f, servo_r_raw_k_minus_2=0.0f, servo_r_filtered_k_minus_1=0.0f, servo_r_filtered_k_minus_2=0.0f;
	float servo_l_raw_k_minus_1=0.0f, servo_l_raw_k_minus_2=0.0f, servo_l_filtered_k_minus_1=0.0f, servo_l_filtered_k_minus_2=0.0f;

	/*Dados usados para posiçao*/
	pv_type_datapr_position pv_module_in_position;
	pv_type_datapr_position pv_module_in_position_reference;

	/*Dados usados para comunicaçao*/
	float pv_module_in_com_rpy[3];
	float pv_module_in_com_drpy[3];
	float pv_module_in_com_position[3];
	float pv_module_in_com_velocity[3];
	float pv_module_in_com_alpha[2];
	float pv_module_in_com_dalpha[2];
	float pv_module_in_com_data1[2],pv_module_in_com_data2[2],pv_module_in_com_data3[2];
	int pv_module_in_com_aux[2];
	float pv_module_in_com_aux2[3];
	float pv_module_in_com_servoTorque[2];
	float pv_module_in_com_escForce[2];
	int pv_module_in_com_channel[7];
	int flag=0;

	/* Inicializa os dados dos servos*/
	pv_module_in_servo_real.alphal=0;
	pv_module_in_servo_real.alphar=0;
	pv_module_in_servo_real.dotAlphal=0;
	pv_module_in_servo_real.dotAlphar=0;

	pv_module_in_servo_filtered.alphal=0;
	pv_module_in_servo_filtered.alphar=0;
	pv_module_in_servo_filtered.dotAlphal=0;
	pv_module_in_servo_filtered.dotAlphar=0;

	pv_module_in_servo_last.alphal=0;
	pv_module_in_servo_last.alphar=0;
	pv_module_in_servo_last.dotAlphal=0;
	pv_module_in_servo_last.dotAlphar=0;

	/* Inicializa os dados da attitude*/
	pv_module_in_InputData.attitude.roll  = 0;
	pv_module_in_InputData.attitude.pitch = 0;
	pv_module_in_InputData.attitude.yaw   = 0;
	pv_module_in_InputData.attitude.dotRoll  = 0;
	pv_module_in_InputData.attitude.dotPitch = 0;
	pv_module_in_InputData.attitude.dotYaw   = 0;

	/* Inicializa os dados da posiçao*/
	pv_module_in_InputData.position.x = 0;
	pv_module_in_InputData.position.y = 0;
	pv_module_in_InputData.position.z = 0;
	pv_module_in_InputData.position.dotX = 0;
	pv_module_in_InputData.position.dotY = 0;
	pv_module_in_InputData.position.dotZ = 0;

	/*Inicializa as referencias*/
	pv_module_in_InputData.position_refrence.x = 0;
	pv_module_in_InputData.position_refrence.y = 0;
	pv_module_in_InputData.position_refrence.z = 0;
	pv_module_in_InputData.position_refrence.dotX = 0;
	pv_module_in_InputData.position_refrence.dotY = 0;
	pv_module_in_InputData.position_refrence.dotZ = 0;

	pv_module_in_InputData.attitude_reference.roll  = 0;
	pv_module_in_InputData.attitude_reference.pitch = 0;
	pv_module_in_InputData.attitude_reference.yaw   = 0;
	pv_module_in_InputData.attitude_reference.dotRoll  = 0;
	pv_module_in_InputData.attitude_reference.dotPitch = 0;
	pv_module_in_InputData.attitude_reference.dotYaw   = 0;

	while(1)
	{
		if (uxQueueMessagesWaiting(pv_interface_in.iControlOutputData)!=0){
			xQueueReceive(pv_interface_in.iControlOutputData, &pv_module_in_ControlData, 0);
		}

  		c_common_gpio_toggle(pv_module_in_LED4);
		/* Leitura do numero de ciclos atuais */
		pv_module_in_WakeTime = xTaskGetTickCount();

		pv_module_in_InputData.heartBeat=pv_module_in_heartBeat+=1;

		/* Verifica init*/
		if (iterations > INIT_ITERATIONS)
				pv_module_in_InputData.init = 0; //Sai da fase de inicializacao

		/* toggle pin for debug */
		//c_common_gpio_toggle(LED_builtin_io);


		#ifdef ENABLE_IMU
			/*----------------------Tratamento da IMU---------------------*/
			/* Pega e trata os valores da imu */

			USART2->CR1 &= 0xFFFFFFDF; //disable interrupt of serial communication
			c_io_imu_getRaw(pv_module_in_InputData.imuOutput.accRaw, pv_module_in_InputData.imuOutput.gyrRaw, pv_module_in_InputData.imuOutput.magRaw,sample_time_gyro_us);
			USART2->CR1 |= 0x00000020; //enable interrupt of serial communication
			c_datapr_MahonyAHRSupdate(pv_module_in_attitude_quaternion,pv_module_in_InputData.imuOutput.gyrRaw[0],pv_module_in_InputData.imuOutput.gyrRaw[1],pv_module_in_InputData.imuOutput.gyrRaw[2],pv_module_in_InputData.imuOutput.accRaw[0],pv_module_in_InputData.imuOutput.accRaw[1],pv_module_in_InputData.imuOutput.accRaw[2],pv_module_in_InputData.imuOutput.magRaw[0],pv_module_in_InputData.imuOutput.magRaw[1],pv_module_in_InputData.imuOutput.magRaw[2],sample_time_gyro_us[0]);
			c_io_imu_Quaternion2Euler(pv_module_in_attitude_quaternion, pv_module_in_rpy);
//			c_datapr_filter_complementary(pv_module_in_rpy,pv_module_in_InputData.imuOutput.accRaw, pv_module_in_InputData.imuOutput.gyrRaw, pv_module_in_InputData.imuOutput.magRaw,sample_time_gyro_us[0]);
			c_io_imu_EulerMatrix(pv_module_in_rpy,pv_module_in_InputData.imuOutput.gyrRaw);

			pv_module_in_InputData.imuOutput.sampleTime =sample_time_gyro_us[0];

			/* Saida dos dados de posição limitada a uma variaçao minima */
			if (abs2(pv_module_in_rpy[PV_IMU_ROLL]-pv_module_in_InputData.attitude.roll)>ATTITUDE_MINIMUM_STEP)
				roll_aux= pv_module_in_rpy[PV_IMU_ROLL];
			if (abs2(pv_module_in_rpy[PV_IMU_PITCH]-pv_module_in_InputData.attitude.pitch)>ATTITUDE_MINIMUM_STEP)
				pitch_aux= pv_module_in_rpy[PV_IMU_PITCH];
			if (abs2(pv_module_in_rpy[PV_IMU_YAW]-yaw_aux)>ATTITUDE_MINIMUM_STEP){
				yaw_aux= pv_module_in_rpy[PV_IMU_YAW];
			}

			pv_module_in_InputData.attitude.roll=roll_aux;
			pv_module_in_InputData.attitude.pitch=pitch_aux;
			pv_module_in_InputData.attitude.yaw=yaw_aux;

			/* Saida dos dados da velocidade angular*/
			pv_module_in_InputData.attitude.dotRoll  = pv_module_in_rpy[PV_IMU_DROLL];
			pv_module_in_InputData.attitude.dotPitch = pv_module_in_rpy[PV_IMU_DPITCH];
			pv_module_in_InputData.attitude.dotYaw   = pv_module_in_rpy[PV_IMU_DYAW ];

			// A referencia é a orientacao que o UAV é iniciado
			if (pv_module_in_InputData.init){
				attitude_roll_initial = pv_module_in_rpy[PV_IMU_ROLL];
				attitude_pitch_initial = pv_module_in_rpy[PV_IMU_PITCH];
				attitude_yaw_initial = pv_module_in_rpy[PV_IMU_YAW];
			}
		#endif
		/*----------------------Tratamento do Radio---------------------*/
		/* Realiza a leitura dos canais do radio-controle */
		//	c_common_gpio_toggle(pv_module_in_LED5);
		pv_module_in_InputData.receiverOutput.joystick[0]=c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);//+100;
		pv_module_in_InputData.receiverOutput.joystick[1]=c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
		pv_module_in_InputData.receiverOutput.joystick[2]=c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL);
		pv_module_in_InputData.receiverOutput.joystick[3]=c_rc_receiver_getChannel(C_RC_CHANNEL_YAW);
		pv_module_in_InputData.receiverOutput.aButton = (int)c_rc_receiver_getChannel(C_RC_CHANNEL_A);
		pv_module_in_InputData.receiverOutput.bButton = (int)c_rc_receiver_getChannel(C_RC_CHANNEL_B);

		/*Como o canal B da valores 1 ou 100 */
		if (pv_module_in_InputData.receiverOutput.bButton>50)
			pv_module_in_InputData.enableintegration = true;
		else
			pv_module_in_InputData.enableintegration = false;

		/*----------------------Seguranças-------------------------------------*/
		/*Para evita falso positivo foi implementada esta funçao*/
		if (!pv_module_in_InputData.receiverOutput.aButton){
			if(pv_module_in_aButton_ant==pv_module_in_InputData.receiverOutput.aButton){
				pv_module_cont++;
			}
			if(pv_module_cont>=10){
				pv_module_in_InputData.securityStop = 1;
				pv_module_cont=0;
			}
			pv_module_in_aButton_ant=pv_module_in_InputData.receiverOutput.aButton;
		}
		else{
			if (pv_module_in_InputData.receiverOutput.aButton){
				pv_module_in_InputData.securityStop = 0;
				pv_module_cont=0;
				pv_module_in_aButton_ant=pv_module_in_InputData.receiverOutput.aButton;
			}
		}

		// Se o yaw está perto da zona de perigo a emergencia é acionada e o birotor é desligado
//		if ( (pv_module_in_rpy[PV_IMU_YAW]*RAD_TO_DEG < -140.0) || (pv_module_in_rpy[PV_IMU_YAW]*RAD_TO_DEG > 140.0))
//			pv_module_in_InputData.securityStop=1;

		if ( (pv_module_in_rpy[PV_IMU_ROLL]*RAD_TO_DEG < -90.0) || (pv_module_in_rpy[PV_IMU_ROLL]*RAD_TO_DEG > 90.0))
			pv_module_in_InputData.securityStop=1;

		if ( (pv_module_in_rpy[PV_IMU_PITCH]*RAD_TO_DEG < -90.0) || (pv_module_in_rpy[PV_IMU_PITCH]*RAD_TO_DEG > 90.0))
			pv_module_in_InputData.securityStop=1;



		/*-----------------------Referencia do sistema---------------------------*/
		pv_module_in_InputData.position_refrence.x  = 0;
		pv_module_in_InputData.position_refrence.y  = 0;
		pv_module_in_InputData.position_refrence.z  = 1;
		pv_module_in_InputData.position_refrence.dotX  = 0;
		pv_module_in_InputData.position_refrence.dotY  = 0;
		pv_module_in_InputData.position_refrence.dotZ  = 0;
		pv_module_in_InputData.attitude_reference.roll  = 0.0000890176;//+(REF_ROLL_MAX*pv_module_in_InputData.receiverOutput.joystick[2]/100);
		pv_module_in_InputData.attitude_reference.pitch = 0.0154833;//+(REF_PITCH_MAX*pv_module_in_InputData.receiverOutput.joystick[1]/100);
		pv_module_in_InputData.attitude_reference.yaw   = 0;
		pv_module_in_InputData.attitude_reference.dotRoll = 0;
		pv_module_in_InputData.attitude_reference.dotPitch= 0;
		pv_module_in_InputData.attitude_reference.dotYaw = 0;
		pv_module_in_InputData.servosOutput.servo_refrence.alphar= -0.0154821;
		pv_module_in_InputData.servosOutput.servo_refrence.alphal= -0.0153665;
		pv_module_in_InputData.servosOutput.servo_refrence.dotAlphar =0;
		pv_module_in_InputData.servosOutput.servo_refrence.dotAlphal =0;


		#ifdef ENABLE_ALTURA
			/*----------------------Tratamento do Sonar---------------------*/
			/* Executa a leitura do sonar */
//		    c_common_gpio_toggle(pv_module_in_LED5);
			sonar_raw_real  =c_io_sonar_read();
			sonar_raw= sonar_raw_real/100;
//			c_common_gpio_toggle(pv_module_in_LED5);

			#ifdef LIMIT_SONAR_VAR
			// corrects the measurement of sonar
			if ( ( (last_valid_sonar_raw-SONAR_MAX_VAR)<sonar_raw && (last_valid_sonar_raw+SONAR_MAX_VAR)>sonar_raw ))
				last_valid_sonar_raw = sonar_raw;
			else
				sonar_raw = last_valid_sonar_raw;
			#endif

			#ifdef FILTER_SONAR_100ms
			//Measurement of sonar with the average of 20 samples
			if (sample<=20){
				sonar_raw_filter=sonar_raw_filter+sonar_raw;
				sample++;
			}else{
				sonar_corrected_debug= sonar_raw_filter/20;
				sonar_corrected = (sonar_corrected_debug)*cos(pv_module_in_InputData.attitude.roll)*cos(pv_module_in_InputData.attitude.pitch);//the altitude must be in meters
				sample=0;
				sonar_raw_filter=0;
			}
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
			dotZ = (sonar_filtered - pv_module_in_InputData.position.z)/0.012;
			// 1st order filter with fc=10Hz
			dotZ_filtered = k1_1o_10Hz*dotZ_filtered_k_minus_1 + k2_1o_10Hz*dotZ + k3_1o_10Hz*dotZ_k_minus_1;
			// Filter memory
			dotZ_filtered_k_minus_1 = dotZ_filtered;
			dotZ_k_minus_1 = dotZ;

			//Filtered measurements
			pv_module_in_InputData.position.x = 0;
			pv_module_in_InputData.position.y = 0;
			pv_module_in_InputData.position.z = sonar_filtered;
			pv_module_in_InputData.position.dotX = 0;
			pv_module_in_InputData.position.dotY = 0;
			pv_module_in_InputData.position.dotZ =dotZ;
		#endif

		#ifdef ENABLE_SERVO_READ
			/*----------------------Tratamento dos servos---------------------*/
			//Leitura da posicao e velocidade atual dos servo motores
			if (!pv_module_in_InputData.init){
				pv_module_in_servo_real=c_io_servos_read();

				/*Left servo filter*/
				//1st order filter with fc=10Hz
				pv_module_in_servo_filtered.dotAlphal = k1_1o_10Hz*servo_l_filtered_k_minus_1 + k2_1o_10Hz*pv_module_in_servo_real.dotAlphal + k3_1o_10Hz*servo_l_raw_k_minus_1;
				// Filter memory
				servo_l_raw_k_minus_1 = pv_module_in_servo_real.dotAlphal;
				servo_l_filtered_k_minus_1 = pv_module_in_servo_filtered.dotAlphal;

				/*Right servo filter*/
				//1st order filter with fc=10Hz
				pv_module_in_servo_filtered.dotAlphar = k1_1o_10Hz*servo_r_filtered_k_minus_1 + k2_1o_10Hz*pv_module_in_servo_real.dotAlphar + k3_1o_10Hz*servo_r_raw_k_minus_1;
				// Filter memory
				servo_r_raw_k_minus_1 = pv_module_in_servo_real.dotAlphar;
				servo_r_filtered_k_minus_1 = pv_module_in_servo_filtered.dotAlphar;
			}
			pv_module_in_InputData.servosOutput.servo.alphal=pv_module_in_servo_real.alphal;
			pv_module_in_InputData.servosOutput.servo.alphar=pv_module_in_servo_real.alphar;
			pv_module_in_InputData.servosOutput.servo.dotAlphal=pv_module_in_servo_filtered.dotAlphal;
			pv_module_in_InputData.servosOutput.servo.dotAlphar=pv_module_in_servo_filtered.dotAlphar;

		#endif


		#ifdef ENABLE_SERVO_WRITE
			/* Escrita dos servos */
			if (pv_module_in_InputData.securityStop){
				//c_io_servos_writePosition(0,0);
				c_io_servos_writeTorque(0,0);
			}
			else{
				// inicializacao
				if (pv_module_in_InputData.init){
					c_io_servos_writePosition(0,0);
				}
				else{
					c_io_servos_writeTorque(pv_module_in_ControlData.actuation.servoRight,pv_module_in_ControlData.actuation.servoLeft);
				}
			}
		#endif

		#ifdef ENABLE_ESC
			/* Escrita dos esc */
			unsigned char sp_right;
			unsigned char sp_left;

			sp_right = pv_module_in_setPointESC_Forca(pv_module_in_ControlData.actuation.escRightNewtons);
			sp_left = pv_module_in_setPointESC_Forca(pv_module_in_ControlData.actuation.escLeftNewtons);

			if (pv_module_in_InputData.securityStop){
				c_io_blctrl_setSpeed(1, 0 );//sp_right
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(0, 0 );//sp_left
			}
			else{
				if (pv_module_in_InputData.receiverOutput.joystick[0]>50){
					//inicializacao
					if (pv_module_in_InputData.init){
						c_io_blctrl_setSpeed(1, ESC_MINIMUM_VELOCITY);
						c_common_utils_delayus(10);
						c_io_blctrl_setSpeed(0, ESC_MINIMUM_VELOCITY);
					}
					else{
		//				c_common_gpio_toggle(pv_module_in_LED5);
						c_io_blctrl_setSpeed(1, sp_right );//sp_right
						c_common_utils_delayus(10);
						c_io_blctrl_setSpeed(0, sp_left );//sp_left
		//				c_common_gpio_toggle(pv_module_in_LED5);
					}
				}
			}
		#endif

		if (pv_module_in_InputData.init)
			iterations++;

		unsigned int timeNow=xTaskGetTickCount();
		pv_module_in_InputData.cicleTime = timeNow - aux_time;
		aux_time=timeNow;
		/* toggle pin for debug */
		c_common_gpio_toggle(pv_module_in_LED4);

		pv_module_in_lastWakeTime=pv_module_in_WakeTime;

		xQueueOverwrite(pv_interface_in.oInputData, &pv_module_in_InputData);

		vTaskDelayUntil( &pv_module_in_lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}

/**\ brief Calcula o set point do ESC a partir da forca passada por argumento
 * Curva retirada dos ensaios com os motores brushless no INEP
 */
unsigned char pv_module_in_setPointESC_Forca(float forca){
	//	Coefficients:
#ifdef AXI2814
	float p1 = 0.00088809, p2 = -0.039541, p3 = 0.67084, p4 = -5.2113, p5 = 16.33, p6 = 10.854, p7 = 3.0802, set_point=0;

	if (forca <= 0)
		return (unsigned char) ESC_MINIMUM_VELOCITY;
	else{
		set_point = (p1*pow(forca,6) + p2*pow(forca,5) + p3*pow(forca,4) + p4*pow(forca,3)
								+ p5*pow(forca,2) + p6*forca + p7);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
#endif

#ifdef AXI2826
	float p1 = 0.000546, p2 = -0.026944, p3 = 0.508397, p4 = -4.822076, p5 = 35.314598, p6 = 3.636088, set_point=0;

	if (forca <= 0)
		return (unsigned char) ESC_MINIMUM_VELOCITY;
	else{
		set_point = (p1*pow(forca,5) + p2*pow(forca,4) + p3*pow(forca,3) + p4*pow(forca,2)
								+ p5*forca + p6);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
#endif
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */



