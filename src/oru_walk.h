/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */


#ifndef ORU_WALK_H
#define ORU_WALK_H


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
#include "avg_filter.h"

//----------------------------------------
// DEFINITIONS
//----------------------------------------


using namespace AL;
using namespace std;


/**
 * @brief A container for constant parameters.
 */
class walk_parameters
{
    public:
        walk_parameters();

        void set(const double,
            const double,
            const double,
            const double,
            const double,
            const double);


        double feedback_gain;
        double feedback_threshold;

        double mpc_alpha;
        double mpc_beta;
        double mpc_gamma;
        double mpc_regularization;
        double mpc_tolerance;

        double step_height;

        int control_sampling_time_ms;
        int preview_sampling_time_ms;
        int preview_window_size;

        unsigned int filter_window_length;
};


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
    void setWalkParameters (
            const float&,
            const float&,
            const float&,
            const float&,
            const float&,
            const float&);


private:
    // initialization
    void initFastRead ();
    void initFastWrite ();
    void initWalkCommands ();
    void initJointAngles ();


    // walking
    void initSteps_NaoModel ();
    void updateModelJoints();
    bool solveMPCProblem ();
    void correctStateAndModel ();

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
    smpc::solver *solver;
    avgFilter *com_filter;
    nao_igm nao;

    smpc::state_orig old_state;

    walk_parameters wp;
    int next_preview_len_ms;

    double init_joint_angles[JOINTS_NUM];
};

#endif  // ORU_WALK_H
