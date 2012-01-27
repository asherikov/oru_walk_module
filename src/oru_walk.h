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

// standard headers
#include <string> // string


// NAO headers
#include <qi/log.hpp>

#include <alcore/alptr.h>
#include <alcore/alerror.h>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alvalue/alvalue.h> // ALValue

#include <alproxies/dcmproxy.h>

#include <almemoryfastaccess/almemoryfastaccess.h>


// other libraries
#include <boost/bind.hpp> // callback hook


// our headers
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

#define ORUW_HALT(message) halt(message, __FUNCTION__)


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
        int loop_time_limit_ms;
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
    void stopWalking(const string& message = "Stopped by request.");
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
    void initWMG_NaoModel ();
    void readSensors (modelState&);
    bool solveMPCProblem ();
    void feedbackError ();
    void halt(const string&, const char *);

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

    walk_parameters wp;
    int next_preview_len_ms;

    double init_joint_angles[JOINTS_NUM];
};

#endif  // ORU_WALK_H
