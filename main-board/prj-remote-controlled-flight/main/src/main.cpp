/*
 *-----------------------------------------------------------------------------
 *  Filename:    main.cpp
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 */

#include <iostream>

#include "DataProcessingManager.h"
#include "ContinuousControlManager.h"
#include "CommLowLevelManager.h"
//#include "ModuleManager.h"

// STL
#include <iostream>

// Boost
#include <boost/thread/thread.hpp>
class mainManager {
public:
    mainManager() {}
    ~mainManager() {}

    //ModuleManager* mm;
    //DataProcessingManager* dm;
};

int main(int argc, char ** argv) {
    DataProcessingManager DataProcessing("DataProcessing:Manager");
    ContinuousControlManager   ContinuousControl("ContinuousControl:Manager");  //AQUI
    CommLowLevelManager        CommLowLevel("CommLowLevel:Manager");  //AQUI


    CommLowLevel.interface->q_atitude_out_ = &ContinuousControl.interface->q_atitude_in; //Aqui
    CommLowLevel.interface->q_position_out_ = &ContinuousControl.interface->q_position_in; //Aqui
    CommLowLevel.interface->q_servos_out_ = &ContinuousControl.interface->q_servos_in; //Aqui
    CommLowLevel.interface->q_debug_out_= &ContinuousControl.interface->q_debug_in; //Aqui
    CommLowLevel.interface->q_rc_out_= &ContinuousControl.interface->q_rc_in; //Aqui
    ContinuousControl.interface->q_actuation_out_ = &CommLowLevel.interface->q_actuation_in; //Aqui

    CommLowLevel.interface->q_atitude2_out_ = &DataProcessing.interface->q_atitude_in; //Aqui
    CommLowLevel.interface->q_position2_out_ = &DataProcessing.interface->q_position_in; //Aqui
    CommLowLevel.interface->q_servos2_out_ = &DataProcessing.interface->q_servos_in; //Aqui
    CommLowLevel.interface->q_debug2_out_= &DataProcessing.interface->q_debug_in; //Aqui
    CommLowLevel.interface->q_rc2_out_= &DataProcessing.interface->q_rc_in; //Aqui
    ContinuousControl.interface->q_actuation2_out_ = &DataProcessing.interface->q_actuation_in; //Aqui

    boost::thread th1( boost::bind( &CommLowLevelManager::Run, CommLowLevel) ); //AQUI
    boost::thread th2( boost::bind( &ContinuousControlManager::Run, ContinuousControl) ); //AQUI
    boost::thread th3( boost::bind( &DataProcessingManager::Run, DataProcessing) ); //AQUI

    th1.join();
    th2.join();
    th3.join();
}
