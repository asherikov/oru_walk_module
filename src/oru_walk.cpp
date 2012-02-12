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
    BIND_METHOD( oru_walk::stopWalkingRemote );


    wmg = NULL;
    mpc = NULL;
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
        ORUW_THROW_ERROR("Error when connecting to DCM: ", e);
    }

    if (!isDCMRunning)
    {
        ORUW_THROW("Error no DCM running");
    }

    try
    {
        // Get the DCM proxy
        dcmProxy = getParentBroker()->getDcmProxy();
    }
    catch (ALError& e)
    {
        ORUW_THROW_ERROR("Impossible to create DCM Proxy: ", e);
    }

// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...

    vector <string> joint_names(JOINTS_NUM, "Device/SubDeviceList/");
    joint_names[HEAD_PITCH]       += "HeadPitch";
    joint_names[HEAD_YAW]         += "HeadYaw";

    joint_names[L_ANKLE_PITCH]    += "LAnklePitch";
    joint_names[L_ANKLE_ROLL]     += "LAnkleRoll";
    joint_names[L_ELBOW_ROLL]     += "LElbowRoll";
    joint_names[L_ELBOW_YAW]      += "LElbowYaw";
    joint_names[L_HIP_PITCH]      += "LHipPitch";
    joint_names[L_HIP_ROLL]       += "LHipRoll";
    joint_names[L_HIP_YAW_PITCH]  += "LHipYawPitch";
    joint_names[L_KNEE_PITCH]     += "LKneePitch";
    joint_names[L_SHOULDER_PITCH] += "LShoulderPitch";
    joint_names[L_SHOULDER_ROLL]  += "LShoulderRoll";
    joint_names[L_WRIST_YAW]      += "LWristYaw";

    joint_names[R_ANKLE_PITCH]    += "RAnklePitch";
    joint_names[R_ANKLE_ROLL]     += "RAnkleRoll";
    joint_names[R_ELBOW_ROLL]     += "RElbowRoll";
    joint_names[R_ELBOW_YAW]      += "RElbowYaw";
    joint_names[R_HIP_PITCH]      += "RHipPitch";
    joint_names[R_HIP_ROLL]       += "RHipRoll";
    joint_names[R_HIP_YAW_PITCH]  += "RHipYawPitch";
    joint_names[R_KNEE_PITCH]     += "RKneePitch";
    joint_names[R_SHOULDER_PITCH] += "RShoulderPitch";
    joint_names[R_SHOULDER_ROLL]  += "RShoulderRoll";
    joint_names[R_WRIST_YAW]      += "RWristYaw";


    initFastRead(joint_names);
    initFastWrite(joint_names);
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
        ORUW_THROW("Wrong parameters");
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
        ORUW_THROW_ERROR("Error on DCM getTime : ", e);
    }


    try
    {
        dcmProxy->set(stiffnessCommands);
    }
    catch (const ALError &e)
    {
        ORUW_THROW_ERROR("Error when sending stiffness to DCM : ", e);
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
        ORUW_THROW_ERROR("Error on DCM getTime : ", e);
    }


    // send commands
    try
    {
        dcmProxy->setAlias(initPositionCommands);
    }
    catch (const AL::ALError &e)
    {
        ORUW_THROW_ERROR("Error with DCM setAlias : ", e);
    }

    qi::os::msleep(init_time);
    qiLogInfo ("module.oru_walk", "Execution of initPosition() is finished.");
}
