/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include <qi/os.hpp>

#include "oru_walk.h"


/**
 * Constructor for oru_walk object
 * @param broker The parent broker
 * @param name The name of the module
 */
oru_walk::oru_walk(ALPtr<ALBroker> broker, const string& name) : 
    ALModule(broker, name),
    accessSensorValues (ALPtr<ALMemoryFastAccess>(new ALMemoryFastAccess())),
    accessActuatorValues (ALPtr<ALMemoryFastAccess>(new ALMemoryFastAccess())),
    wp (broker)
{
    setModuleDescription("Orebro University: NAO walking module");


    // advertise functions
    functionName( "setStiffness" , getName(), "change stiffness of all joint");
    addParam( "value", "new stiffness value from 0.0 to 1.0");
    BIND_METHOD( oru_walk::setStiffness );

    functionName( "initPosition", getName() , "initialize robot position");
    BIND_METHOD( oru_walk::initPosition );

    functionName( "walk", getName() , "walk");
    BIND_METHOD( oru_walk::walk );

    functionName( "stopWalking", getName() , "stopWalking");
    BIND_METHOD( oru_walk::stopWalking );


    wmg = NULL;
    solver = NULL;
    com_filter = NULL;
}



/**
 * Destructor for oru_walk object
 */
oru_walk::~oru_walk()
{
    setStiffness(0.0f);
    // Remove the postProcess call back connection
    fDCMPostProcessConnection.disconnect();

    if (wmg != NULL)
    {
        delete wmg;
        wmg = NULL;
    }
    if (solver != NULL)
    {
        delete solver;
        solver = NULL;
    }
    if (com_filter != NULL)
    {
        delete com_filter;
        com_filter = NULL;
    }
}



// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
/**
 * @brief 
 */
void oru_walk::init()
{
    bool isDCMRunning;


    // Is the DCM running ?
    try
    {
        isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
    }
    catch (ALError& e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error when connecting to DCM : " + e.toString());
    }

    if (!isDCMRunning)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error no DCM running ");
    }

    try
    {
        // Get the DCM proxy
        dcmProxy = getParentBroker()->getDcmProxy();
    }
    catch (ALError& e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Impossible to create DCM Proxy : " + e.toString());
    }

// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
    initFastRead();
    initFastWrite();
    initWalkCommands();
    initJointAngles();
}


/**
 * @brief Set stiffness of joints.
 *
 * @param[in] stiffnessValue value of stiffness [0;1]
 */
void oru_walk::setStiffness(const float &stiffnessValue)
{
    ALValue stiffnessCommands;


    if ((stiffnessValue < 0) || (stiffnessValue > 1))
    {
        throw ALERROR(getName(), __FUNCTION__, "Wrong parameters");
    }


    // Prepare one dcm command:
    // it will linearly "Merge" all joint stiffness
    // from last value to "stiffnessValue" in 1 seconde
    stiffnessCommands.arraySetSize(3);
    stiffnessCommands[0] = std::string("jointStiffness");
    stiffnessCommands[1] = std::string("Merge");
    stiffnessCommands[2].arraySetSize(1);
    stiffnessCommands[2][0].arraySetSize(2);
    stiffnessCommands[2][0][0] = stiffnessValue;


    /// @attention Hardcoded parameter!
    unsigned int stiffness_change_time = 1000;
    try
    {
        stiffnessCommands[2][0][1] = dcmProxy->getTime(stiffness_change_time);
    }
    catch (const ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error on DCM getTime : " + e.toString());
    }


    try
    {
        dcmProxy->set(stiffnessCommands);
    }
    catch (const ALError &e)
    {
        throw ALERROR (getName(), __FUNCTION__, "Error when sending stiffness to DCM : " + e.toString());
    }

    qi::os::msleep(stiffness_change_time);
    qiLogInfo ("module.oru_walk", "Execution of setStiffness() is finished.");
}



/**
 * @brief 
 */
void oru_walk::initPosition()
{
    /// @attention Hardcoded parameter!
    unsigned int init_time = 1200;


    ALValue initPositionCommands;

    initPositionCommands.arraySetSize(6);
    initPositionCommands[0] = string("jointActuator");
    initPositionCommands[1] = string("ClearAll");
    initPositionCommands[2] = string("time-separate");
    initPositionCommands[3] = 0;
    initPositionCommands[4].arraySetSize(1);
    initPositionCommands[5].arraySetSize(JOINTS_NUM);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        initPositionCommands[5][i].arraySetSize(1);
        initPositionCommands[5][i][0]  =  init_joint_angles[i];
    }

    // set time
    try
    {
        initPositionCommands[4][0] = dcmProxy->getTime(init_time);
    }
    catch (const ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error on DCM getTime : " + e.toString());
    }


    // send commands
    try
    {
        dcmProxy->setAlias(initPositionCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error with DCM setAlias : " + e.toString());
    }

    qi::os::msleep(init_time);
    qiLogInfo ("module.oru_walk", "Execution of initPosition() is finished.");
}
