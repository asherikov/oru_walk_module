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
#include <alproxies/almemoryproxy.h>

#include <almemoryfastaccess/almemoryfastaccess.h>

#include <althread/almutex.h>


// other libraries
#include <boost/bind.hpp> // callback hook
#include <boost/thread.hpp> 

#include <althread/alcriticalsection.h>


// our headers
#include "WMG.h" // footsteps & parameters
#include "smpc_solver.h" // solver
#include "joints_sensors_id.h"
#include "nao_igm.h"
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

//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
    // initialization
    void initFastRead (const vector<string>&);
    void initFastWrite (const vector<string>&);
    void initWalkCommands ();
    void initJointAngles (ALValue &);

    void initWalkPattern_Straight();
    void initWalkPattern_Diagonal();


    // walking
    void initWMG_NaoModel ();
    void initWMG_NaoModel1 ();
    void readSensors (jointState&);
    bool solveMPCProblem ();
    void solveIKsendCommands (const int, const int, nao_igm &);

    void feedbackError ();
    void halt(const char*, const char *);
    void stopWalking(const char*);

    void walkControl();
    // periodically called callback function
    void dcmCallback();


// private variables
    ProcessSignalConnection dcm_callback_connection;

    // Used for fast memory access
    ALPtr<ALMemoryFastAccess> access_sensor_values;
    int* last_dcm_time_ms_ptr;

    // Used to store command to send
    ALValue joint_commands;


    WMG *wmg;
    smpc_parameters *mpc;
    smpc::solver *solver;

    nao_igm nao;
    nao_igm nao_next;

    walkParameters wp;

    int dcm_loop_counter;
    int last_dcm_time_ms;

    ALPtr<DCMProxy> dcm_proxy;
    ALPtr<ALMemoryProxy> memory_proxy;

    boost::condition_variable walk_control_condition;
    boost::mutex walk_control_mutex;
};

#endif  // ORU_WALK_H
