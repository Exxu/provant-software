/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommLowLevelataManager.cpp
 *  Implementação do gerenciador e instanciador de submodulos.
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 *
 */

#include "CommLowLevelManager.h"
#include "proVantTypes.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>

using namespace Eigen;
using namespace std;

CommLowLevelManager::CommLowLevelManager(std::string name) :
    interface(new CommLowLevelInterface("CommLowLevel:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(12),
    name_(name)
{
	//mpc=new MPC::MpcControler();
	//mpcload=new MPCLOAD::MpcLoad();
	mpcbirotor=new MPCBirotor::MpcBirotor();
	//mpc=new MPC::MpcControler();
	//lqr=new LQR::LQRControler();
	//test= new TEST::TESTActuator();
}

CommLowLevelManager::~CommLowLevelManager()
{

}

void CommLowLevelManager::Init()
{
    //921600     460800
    PROVANT.init("/dev/ttyO1", 921600);
    DEBUG(LEVEL_INFO, "Connected 1");

    // DEBUG(DEBUG_INFO, "Initializing: ") << __func__ ;

    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...

    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void CommLowLevelManager::Run()
{
    Init();
    // Algumas variaveis... 
    proVant::atitude atitude, atitude_aux;
    proVant::position position, position_aux;
    proVant::servos_state servos, servos_aux;
    proVant::debug debug, debug_aux;
    proVant::rcNormalize rcNormalize, rcNormalize_aux;
    proVant::controlOutput actuation;
    proVant::controlOutput actuation2, actuation2_aux;
    proVant::status status;

    atitude.dotPitch=0;
    atitude.dotRoll=0;
    atitude.dotYaw=0;
    atitude.pitch=0;
    atitude.roll=0;
    atitude.yaw=0;

    position.x=0;
    position.y=0;
    position.z=0;
    position.dotX=0;
    position.dotY=0;
    position.dotZ=0;

    servos.alphal=0;
    servos.alphar=0;
    servos.dotAlphal=0;
    servos.dotAlphar=0;

    atitude_aux=atitude;
    position_aux=position;
    servos_aux=servos;
    float data1[2]={};
    float data2[2]={};
    float data3[2]={};
    int i = 0, flag=0;

    /*Initialization of actuation*/
    actuation.servoLeft=0;
    actuation.servoRight=0;
    actuation.escLeftNewtons=0;
    actuation.escRightNewtons=0;
    actuation.escLeftSpeed=0;
    actuation.escRightSpeed=0;
    actuation2=actuation;
    actuation2_aux=actuation;

    /* Matrix class*/
    MatrixXf xs(16,1);
    MatrixXf channels(4,1);
    MatrixXf u(4,1);

    for (int j=0;j<7;j++)
    	rcNormalize.normChannels[j]=0;

    while(1) {
    	auto start = std::chrono::steady_clock::now();
    	/*Recive states from Discovery*/
    	flag=PROVANT.updateData();

    	atitude_aux = PROVANT.getVantData().getAtitude();
    	if (atitude_aux.dotPitch!=0 || atitude_aux.dotRoll!=0 || atitude_aux.dotYaw!=0 || atitude_aux.pitch!=0 || atitude_aux.roll!=0 || atitude_aux.yaw!=0){
    		atitude=atitude_aux;
    	}
    	position_aux= PROVANT.getVantData().getPosition();
    	if(position_aux.dotX!=0 || position_aux.dotY!=0 || position_aux.dotZ!=0 || position_aux.x!=0 || position_aux.y!=0 || position_aux.z!=0){
    		position=position_aux;
    	}
    	actuation2_aux=PROVANT.getVantData().getActuation();
    	if (actuation2_aux.escLeftNewtons!=0 || actuation2_aux.escRightNewtons!=0 || actuation2_aux.servoLeft!=0 || actuation2_aux.servoRight!=0){
    		actuation2=actuation2_aux;
    	}
    	servos_aux= PROVANT.getVantData().getServoState();    //Function made to save current as alpha and voltage as dotalpha
    	if(servos_aux.alphal!=0 || servos_aux.alphar!=0 || servos_aux.dotAlphal!=0 || servos_aux.dotAlphar!=0){
    		servos=servos_aux;
    	}


    	//rcNormalize= PROVANT.getVantData().getNormChannels();
    	status=PROVANT.getVantData().getStatus();
    	debug=PROVANT.getVantData().getDebug();

    	/*Control calculation*/
    	channels.setZero();
    	channels<<rcNormalize.normChannels[0],rcNormalize.normChannels[1],rcNormalize.normChannels[2],rcNormalize.normChannels[3];
    	xs.setZero();
    	xs<<position.x,position.y,position.z,atitude.roll,atitude.pitch,atitude.yaw,servos.alphar,servos.alphal
    			,position.dotX,position.dotY,position.dotZ,atitude.dotRoll,atitude.dotPitch,atitude.dotYaw,servos.dotAlphar,servos.dotAlphal;

    	//u=mpc->Controler(xs);
    	//u=lqr->Controler(xs,status.stop);
    	//u=mpcload->Controler(xs);
    	u=mpcbirotor->Controler(xs);
    	//u=test->Controler(channels);

    	actuation.escRightNewtons=u(0,0);
    	actuation.escLeftNewtons=u(1,0);
    	actuation.servoRight=u(2,0);
    	actuation.servoLeft=u(3,0);
    	actuation.escLeftSpeed=0;
    	actuation.escRightSpeed=0;

    	/*Send Control to Discovery*/

    	data1[0]= actuation.servoLeft;
   		data1[1]= actuation.servoRight;
   		data3[0]= actuation.escLeftNewtons;
   		data3[1]= actuation.escLeftSpeed;
   		data2[0]= actuation.escRightNewtons;
   		data2[1]= actuation.escRightSpeed;

   		PROVANT.multwii2_sendControldataout(data1,data3,data2);
   		PROVANT.multwii_sendstack();

   		auto end = std::chrono::steady_clock::now();
   		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
   		std::cout << "It took me " << (float)(elapsed.count()/1000) << " miliseconds." << std::endl;

    	/*Elapsed time code*/


    	interface->push(position, interface->q_position2_out_);
    	interface->push(atitude, interface->q_atitude2_out_);
    	interface->push(servos, interface->q_servos2_out_);
    	interface->push(debug, interface->q_debug2_out_);
    	interface->push(rcNormalize, interface->q_rc2_out_);
    	interface->push(actuation2, interface->q_actuation2_out_);
    	interface->push(status,interface->q_status2_out_);

    	i++;
    	boost::this_thread::sleep(boost::posix_time::milliseconds(ms_sample_time));
    }
}

void CommLowLevelManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

