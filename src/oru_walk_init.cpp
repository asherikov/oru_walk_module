/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"


/**
 * @brief Initializes variables, that are necessary for fast reading of data from memory.
 */
void oru_walk::initFastRead()
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
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    fSensorKeys[R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    fSensorKeys[R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    fSensorKeys[R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    fSensorKeys[R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    fSensorKeys[R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

    // Create the fast memory access
    accessSensorValues->ConnectToVariables(getParentBroker(), fSensorKeys, false);


    // Joints Sensor list
    fSensorKeys[HEAD_PITCH]       = std::string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    fSensorKeys[HEAD_YAW]         = std::string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");

    fSensorKeys[L_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    fSensorKeys[L_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    fSensorKeys[L_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    fSensorKeys[L_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    fSensorKeys[L_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    fSensorKeys[L_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    fSensorKeys[L_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    fSensorKeys[L_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    fSensorKeys[L_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    fSensorKeys[L_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    fSensorKeys[L_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");

    fSensorKeys[R_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    fSensorKeys[R_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    fSensorKeys[R_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    fSensorKeys[R_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    fSensorKeys[R_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    fSensorKeys[R_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    fSensorKeys[R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Actuator/Value");
    fSensorKeys[R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    fSensorKeys[R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    fSensorKeys[R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    fSensorKeys[R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");

    // Create the fast memory access
    accessActuatorValues->ConnectToVariables(getParentBroker(), fSensorKeys, false);
}



/**
 * @brief Initializes variables, that are necessary for fast sending of parameters to DCM.
 */
void oru_walk::initFastWrite()
{
    ALValue jointAliases;

    jointAliases.arraySetSize(2);
    jointAliases[1].arraySetSize(JOINTS_NUM);

    // positions of actuators.
    jointAliases[0] = std::string("jointActuator"); // Alias for all joint actuators
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
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    jointAliases[1][R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    jointAliases[1][R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    jointAliases[1][R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    jointAliases[1][R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    jointAliases[1][R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");

    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliases);
    }
    catch (const ALError &e)
    {
        ORUW_THROW_ERROR("Error when creating Alias: ", e);
    }



    //  stiffness of actuators.
    jointAliases[0] = std::string("jointStiffness"); // Alias for all actuators
    jointAliases[1][HEAD_PITCH]       = std::string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value");
    jointAliases[1][HEAD_YAW]         = std::string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value");
    jointAliases[1][L_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value");
    jointAliases[1][L_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value");
    jointAliases[1][L_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value");
    jointAliases[1][L_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value");
    jointAliases[1][L_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value");
    jointAliases[1][L_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value");
    jointAliases[1][L_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value");
    jointAliases[1][L_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value");
    jointAliases[1][L_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value");
    jointAliases[1][L_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value");
    jointAliases[1][L_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value");

    jointAliases[1][R_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value");
    jointAliases[1][R_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value");
    jointAliases[1][R_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value");
    jointAliases[1][R_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value");
    jointAliases[1][R_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value");
    jointAliases[1][R_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value");
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    jointAliases[1][R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Hardness/Sensor/Value");
    jointAliases[1][R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value");
    jointAliases[1][R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value");
    jointAliases[1][R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value");
    jointAliases[1][R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value");

    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliases);
    }
    catch (const ALError &e)
    {
        ORUW_THROW_ERROR("Error when creating Alias: ", e);
    }



    // access to the actuators in the lower body only
    jointAliases[1].clear();
    jointAliases[1].arraySetSize(LOWER_JOINTS_NUM);

    // positions of actuators.
    jointAliases[0] = std::string("lowerJointActuator"); // Alias for all joint actuators
    // Joints actuator list
    jointAliases[1][L_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    jointAliases[1][L_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    jointAliases[1][L_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    jointAliases[1][L_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    jointAliases[1][L_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    jointAliases[1][L_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");

    jointAliases[1][R_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    jointAliases[1][R_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    jointAliases[1][R_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    jointAliases[1][R_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    jointAliases[1][R_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    jointAliases[1][R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");

    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliases);
    }
    catch (const ALError &e)
    {
        ORUW_THROW_ERROR("Error when creating Alias: ", e);
    }
}



/**
 * @brief Initialize commands, that will be sent to DCM.
 */
void oru_walk::initWalkCommands()
{
    // create the structure of the commands
    walkCommands.arraySetSize(6);
    walkCommands[0] = string("lowerJointActuator");
    walkCommands[1] = string("ClearAll");
    walkCommands[2] = string("time-separate");
    walkCommands[3] = 0;

    walkCommands[4].arraySetSize(2);

    walkCommands[5].arraySetSize(LOWER_JOINTS_NUM); // For all joints
    for (int i=0; i < LOWER_JOINTS_NUM; i++)
    {
        walkCommands[5][i].arraySetSize(2);
    }
}


void oru_walk::initJointAngles()
{
    init_joint_angles[L_HIP_YAW_PITCH]  =  0.0;
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    init_joint_angles[R_HIP_YAW_PITCH]  =  0.0;

    init_joint_angles[L_HIP_ROLL]       = -0.000384;
    init_joint_angles[L_HIP_PITCH]      = -0.598291;
    init_joint_angles[L_KNEE_PITCH]     =  1.009413;
    init_joint_angles[L_ANKLE_PITCH]    = -0.492352;
    init_joint_angles[L_ANKLE_ROLL]     =  0.000469;

    init_joint_angles[R_HIP_ROLL]       = -0.000384;
    init_joint_angles[R_HIP_PITCH]      = -0.598219;
    init_joint_angles[R_KNEE_PITCH]     =  1.009237;
    init_joint_angles[R_ANKLE_PITCH]    = -0.492248;
    init_joint_angles[R_ANKLE_ROLL]     =  0.000469;

    init_joint_angles[L_SHOULDER_PITCH] =  1.418908;
    init_joint_angles[L_SHOULDER_ROLL]  =  0.332836;
    init_joint_angles[L_ELBOW_YAW]      = -1.379108;
    init_joint_angles[L_ELBOW_ROLL]     = -1.021602;
    init_joint_angles[L_WRIST_YAW]      = -0.013848;

    init_joint_angles[R_SHOULDER_PITCH] =  1.425128;
    init_joint_angles[R_SHOULDER_ROLL]  = -0.331386;
    init_joint_angles[R_ELBOW_YAW]      =  1.383626;
    init_joint_angles[R_ELBOW_ROLL]     =  1.029356;
    init_joint_angles[R_WRIST_YAW]      = -0.01078; 

    init_joint_angles[HEAD_PITCH]       =  0.0;     
    init_joint_angles[HEAD_YAW]         =  0.0;
}
