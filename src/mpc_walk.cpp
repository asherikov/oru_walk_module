/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "mpc_walk.h"


/**
 * Constructor for mpc_walk object
 * @param broker The parent broker
 * @param name The name of the module
 */
mpc_walk::mpc_walk(ALPtr<ALBroker> broker, const string& name) : 
    ALModule(broker, name),
    fMemoryFastAccess (ALPtr<ALMemoryFastAccess>(new ALMemoryFastAccess()))
{
    setModuleDescription("Orebro University: NAO walking module");


    // advertise functions
    functionName( "setStiffness" , getName(), "change stiffness of all joint");
    addParam( "value", "new stiffness value from 0.0 to 1.0");
    BIND_METHOD( mpc_walk::setStiffness );

    functionName( "initPosition", getName() , "initialize robot position");
    BIND_METHOD( mpc_walk::initPosition );

    functionName( "walk", getName() , "walk");
    BIND_METHOD( mpc_walk::walk );

    functionName( "stopWalking", getName() , "stopWalking");
    BIND_METHOD( mpc_walk::stopWalking );


    wmg = NULL;
    solver = NULL;
}



/**
 * Destructor for mpc_walk object
 */
mpc_walk::~mpc_walk()
{
    //setStiffness(0.0f);
    // Remove the postProcess call back connection
    fDCMPostProcessConnection.disconnect();

    // BE CAREFULL:
    // STIFFNESS IS NOT SET = 0 !!!
    // because when we reach initial position, we want to stay there
    // this function is called (among other plances) at the end of 
    // initPosition() when the desired angles have been reached.
    
    if (wmg != NULL)
    {
        delete wmg;
    }
    if (solver != NULL)
    {
        delete solver;
    }
}



// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
/**
 * @brief 
 */
void mpc_walk::init()
{
    bool isDCMRunning;


    // Is the DCM running ?
    try
    {
        isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
    }
    catch (ALError& e)
    {
        throw ALERROR(getName(), "startLoop()", "Error when connecting to DCM : " + e.toString());
    }

    if (!isDCMRunning)
    {
        throw ALERROR(getName(), "startLoop()", "Error no DCM running ");
    }

    try
    {
        // Get the DCM proxy
        dcmProxy = getParentBroker()->getDcmProxy();
    }
    catch (ALError& e)
    {
        throw ALERROR(getName(), "startLoop()", "Impossible to create DCM Proxy : " + e.toString());
    }

// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
    initFastRead();
    initFastWrite();

    preparePositionActuatorCommand();
}



/**
 * @brief Initializes variables, that are necessary for fast reading of data from memory.
 * @todo getDataPtr might be faster
 */
void mpc_walk::initFastRead()
{
    // Sensors names
    vector<string> fSensorKeys;

    fSensorKeys.clear();

    //  Here as an example /*inertial*/ + joints + /*FSR*/ are read
    fSensorKeys.resize(/*7 +*/ JOINTS_NUM /*+ 6*/);

    // Joints Sensor list
    fSensorKeys[HEAD_PITCH]       = std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    fSensorKeys[HEAD_YAW]         = std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");

    fSensorKeys[L_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    fSensorKeys[L_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    fSensorKeys[L_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    fSensorKeys[L_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    fSensorKeys[L_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    fSensorKeys[L_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    fSensorKeys[L_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    fSensorKeys[L_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    fSensorKeys[L_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    fSensorKeys[L_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    fSensorKeys[L_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");

    fSensorKeys[R_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    fSensorKeys[R_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    fSensorKeys[R_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    fSensorKeys[R_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    fSensorKeys[R_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    fSensorKeys[R_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    /// @todo note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    fSensorKeys[R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    fSensorKeys[R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    fSensorKeys[R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    fSensorKeys[R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    fSensorKeys[R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

    // Create the fast memory access
    fMemoryFastAccess->ConnectToVariables(getParentBroker(), fSensorKeys, false);
}



/**
 * @brief Initializes variables, that are necessary for fast sending of parameters to DCM.
 */
void mpc_walk::initFastWrite()
{
    ALValue jointAliases;

    jointAliases.arraySetSize(2);
    jointAliases[1].arraySetSize(JOINTS_NUM);

    // Joints actuator list
    jointAliases[1][HEAD_PITCH]       = std::string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    jointAliases[1][HEAD_YAW]         = std::string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    jointAliases[1][L_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    jointAliases[1][L_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    jointAliases[1][L_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    jointAliases[1][L_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    jointAliases[1][L_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    jointAliases[1][L_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    jointAliases[1][L_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    jointAliases[1][L_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    jointAliases[1][L_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    jointAliases[1][L_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    jointAliases[1][L_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");

    jointAliases[1][R_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    jointAliases[1][R_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    jointAliases[1][R_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    jointAliases[1][R_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    jointAliases[1][R_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    jointAliases[1][R_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    /// @todo note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    jointAliases[1][R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    jointAliases[1][R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    jointAliases[1][R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    jointAliases[1][R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    jointAliases[1][R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");


    // positions of actuators.
    jointAliases[0] = std::string("jointActuator"); // Alias for all joint actuators
    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliases);
    }
    catch (const ALError &e)
    {
        throw ALERROR(getName(), "createPositionActuatorAlias()", "Error when creating Alias : " + e.toString());
    }


    //  stiffness of actuators.
    jointAliases[0] = std::string("jointStiffness"); // Alias for all actuators
    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliases);
    }
    catch (const ALError &e)
    {
        throw ALERROR(getName(), "createStiffnessActuatorAlias()", "Error when creating Alias : " + e.toString());
    }
}


/**
 * @brief Initialize commands, that will be sent to DCM.
 */
void mpc_walk::preparePositionActuatorCommand()
{
    // create the structure of the commands
    walkCommands.arraySetSize(6);
    walkCommands[0] = string("jointActuator");
    walkCommands[1] = string("ClearAll");
    walkCommands[2] = string("time-separate");
    walkCommands[3] = 0;

    walkCommands[4].arraySetSize(1);

    walkCommands[5].arraySetSize(JOINTS_NUM); // For all joints
    for (int i=0; i < JOINTS_NUM; i++)
    {
        walkCommands[5][i].arraySetSize(1);
    }
}
