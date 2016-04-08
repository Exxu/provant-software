/**
  ******************************************************************************
  * @file    modules/rc/c_rc_SFC.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_LQR_servos.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_State_Feedback_Control
  * \brief Controle de estabilização para o modo de operação RC por realimentacao de estados.
  *
   * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// Matrizes de ganho


float32_t c_rc_LQR_servo_Ke_f32[1][2]={-2465.61778, -144.95599};

float32_t c_rc_LQR_servo_equilibrium_control_f32[1]={0};

float32_t c_rc_LQR_servo_state_vector_f32[2]={0};
float32_t c_rc_LQR_servo_state_vector_reference_f32[2]={0};
float32_t c_rc_LQR_servo_error_state_vector_f32[2]={0};

float32_t c_rc_LQR_servo_control_output_f32[1]={0};

arm_matrix_instance_f32 c_rc_LQR_servo_equilibrium_control;
arm_matrix_instance_f32 c_rc_LQR_servo_Ke;

//Vector integration of error(Trapezoidal method)

/* Private function prototypes -----------------------------------------------*/
arm_matrix_instance_f32 c_rc_LQR_servo_errorStateVector(pv_type_datapr_servos servo,
														pv_type_datapr_servos servo_reference,
														float sample_time, bool stop);

/* Private functions ---------------------------------------------------------*/

float32_t c_rc_LQR_saturation(float32_t value, float32_t lower_limit, float32_t upper_limit){
	if (value <= lower_limit)
		return lower_limit;
	else if (value >= upper_limit)
		return upper_limit;
	else
		return value;
}


arm_matrix_instance_f32 c_rc_LQR_servo_errorStateVector(pv_type_datapr_servos servo,
												   pv_type_datapr_servos servo_reference,
												   float sample_time, bool stop){

	arm_matrix_instance_f32 c_rc_LQR_servo_error_state_vector, c_rc_LQR_servo_state_vector,  c_rc_LQR_servo_reference_state_vector;

	//State Vector
	c_rc_LQR_servo_state_vector_f32[0]=servo.alphal;
    c_rc_LQR_servo_state_vector_f32[1]=servo.dotAlphal;
	//Updates the height equilibrium point according to the reference
	c_rc_LQR_servo_state_vector_reference_f32[0]=servo_reference.alphal;
	c_rc_LQR_servo_state_vector_reference_f32[1]=servo_reference.dotAlphal;

	//Initializes the matrices
	arm_mat_init_f32(&c_rc_LQR_servo_state_vector, 2, 1, (float32_t *)c_rc_LQR_servo_state_vector_f32);
	arm_mat_init_f32(&c_rc_LQR_servo_reference_state_vector, 2, 1, (float32_t *)c_rc_LQR_servo_state_vector_reference_f32);
	arm_mat_init_f32(&c_rc_LQR_servo_error_state_vector, 2, 1, (float32_t *)c_rc_LQR_servo_error_state_vector_f32);
	//e(t)=x(k)- xr(k)
	arm_mat_sub_f32(&c_rc_LQR_servo_state_vector, &c_rc_LQR_servo_reference_state_vector, &c_rc_LQR_servo_error_state_vector);

	return c_rc_LQR_servo_error_state_vector;
}

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utiliza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_rc_LQR_servo_control_init() {

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&c_rc_LQR_servo_equilibrium_control, 1, 1, (float32_t *)c_rc_LQR_servo_equilibrium_control_f32);
	arm_mat_init_f32(&c_rc_LQR_servo_Ke, 1, 2, (float32_t *)c_rc_LQR_servo_Ke_f32);

}

/* \brief LQR Controller.
 *
 * Implemented based on the article "Back-stepping Control Strategy for Stabilization of a Tilt-rotor UAV" by Chowdhury, A. B., with some modifications.
 * It implements an estabilization controller and an altitude controller. It is meant to be used with the radio controller.
 * The struct pv_msg_io_attitude includes the angular velocity.
 */

pv_type_actuation c_rc_LQR_servo(pv_type_datapr_servos servo,
				  pv_type_datapr_servos servo_reference,
				  float sample_time, bool stop){

	pv_type_actuation actuation_signals;

	arm_matrix_instance_f32 c_rc_LQR_servo_error_state_vector, c_rc_LQR_servo_control_output;

	//Initialize result matrices
	arm_mat_init_f32(&c_rc_LQR_servo_control_output, 1, 1, (float32_t *)c_rc_LQR_servo_control_output_f32);

	//e(k)=xs_a(k)-xr_a(k)
	c_rc_LQR_servo_error_state_vector = c_rc_LQR_servo_errorStateVector(servo, servo_reference,sample_time, stop);
	//u=Ke*e(t)
	arm_mat_mult_f32(&c_rc_LQR_servo_Ke, &c_rc_LQR_servo_error_state_vector, &c_rc_LQR_servo_control_output);

	arm_mat_add_f32(&c_rc_LQR_servo_equilibrium_control, &c_rc_LQR_servo_control_output, &c_rc_LQR_servo_control_output);


	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightNewtons=(float)0;
	actuation_signals.escLeftNewtons=(float)0;
	actuation_signals.servoRight=c_rc_LQR_saturation(c_rc_LQR_servo_control_output.pData[0]/1000,-2,2);
	actuation_signals.servoLeft=c_rc_LQR_saturation(c_rc_LQR_servo_control_output.pData[0]/1000,-2,2);
	actuation_signals.escRightSpeed=0;
	actuation_signals.escLeftSpeed=0;
	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

