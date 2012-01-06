/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */


#ifndef ORU_MODULE_MPC_WALK_H
#define ORU_MODULE_MPC_WALK_H


//----------------------------------------
// INCLUDES
//----------------------------------------
#include <alcore/alptr.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alcore/alerror.h>

// Use DCM proxy
#include <alproxies/dcmproxy.h>

// Used to read values of ALMemory directly in RAM
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <boost/bind.hpp>


#include <alcore/alptr.h> // ALPtr
#include <alcommon/almodule.h> // ALModule, ProcessSignalConnection (almodulesynchronization.h)
#include <alcommon/albroker.h> // ALBroker

#include <almemoryfastaccess/almemoryfastaccess.h> // ALMemoryFastAccess

#include <alproxies/dcmproxy.h> // DCMProxy

#include <alvalue/alvalue.h> // ALValue


#include <string> // string


#include "WMG.h" // footsteps & parameters
#include "smpc_solver.h" // solver
#include "joints_sensors_id.h"
#include "nao_igm.h"

//----------------------------------------
// DEFINITIONS
//----------------------------------------


using namespace AL;
using namespace std;



/**
 * @brief The main walking module class.
 */
class oru_walk : public ALModule
{
public:
    // constructors / destructors
    oru_walk(ALPtr<ALBroker> broker, const string& name);
    ~oru_walk ();


    // is called automatically when a library is loaded 
    void init();


// These methods will be advertised to other modules.
    void stopWalking();
    void initPosition();
    void setStiffness(const float &);
    void walk();


private:
    // initialization
    void initFastRead ();
    void initFastWrite ();
    void preparePositionActuatorCommand();


    // walking
    void readSensorsToNaoModel ();
    void initWMG (const int);
    void initNaoModel ();
    void solveMPCProblem ();
    void logJointValues();

    // Callback called by the DCM every 10ms
    void callbackEveryCycle_walk();


// private variables

    // Used for postprocess sync with the DCM
    ProcessSignalConnection fDCMPostProcessConnection;

    // Used for fast memory access
    ALPtr<ALMemoryFastAccess> accessSensorValues;
    ALPtr<ALMemoryFastAccess> accessActuatorValues;

    // Store sensor values.
    vector<float> sensorValues;
    ALPtr<DCMProxy> dcmProxy;

    // Used to store command to send
    ALValue walkCommands;



    WMG *wmg;
    smpc_solver *solver;
    nao_igm nao;

    int next_preview_len_ms;
    int control_sampling_time_ms;
    int preview_sampling_time_ms;
};

#endif  // ORU_MODULE_MPC_WALK_H
