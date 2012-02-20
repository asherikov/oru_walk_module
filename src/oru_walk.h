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
#include "walk_parameters.h"



//----------------------------------------
// DEFINITIONS
//----------------------------------------


using namespace AL;
using namespace std;

#define ORUW_THROW(message) throw ALERROR(getName(), __FUNCTION__, message)
#define ORUW_THROW_ERROR(message,error) throw ALERROR(getName(), __FUNCTION__, message + string (error.what()))



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
    void stopWalkingRemote();
    void initPosition();
    void setStiffness(const float &);
    void walk();


private:
    // initialization
    void initFastRead (const vector<string>&);
    void initFastWrite (const vector<string>&);
    void initWalkCommands ();
    void initJointAngles ();


    // walking
    void initWMG_NaoModel ();
    void readSensors (jointState&);
    bool solveMPCProblem ();
    void solveIKsendCommands (const int, const int, nao_igm &);

    void feedbackError ();
    void halt(const string&, const char *);
    void stopWalking(const string& message);

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
    smpc_parameters *mpc;
    smpc::solver *solver;
    avgFilter *com_filter;

    nao_igm nao;
    nao_igm nao_next;

    walkParameters wp;
    int next_preview_len_ms;

    double init_joint_angles[JOINTS_NUM];
};

#endif  // ORU_WALK_H
