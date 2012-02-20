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
void oru_walk::initFastRead(const vector<string>& joint_names)
{
    // Sensors names
    vector<string> fSensorKeys;

    fSensorKeys.clear();

    //  Here as an example /*inertial*/ + joints + /*FSR*/ are read
    fSensorKeys.resize(/*7 +*/ JOINTS_NUM /*+ 6*/);


    // connect to sensors
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        fSensorKeys[i] = joint_names[i] + "/Position/Sensor/Value";
    }
    // Create the fast memory access
    accessSensorValues->ConnectToVariables(getParentBroker(), fSensorKeys, false);


    // connect to actuators
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        fSensorKeys[i] = joint_names[i] + "/Position/Actuator/Value";
    }
    // Create the fast memory access
    accessActuatorValues->ConnectToVariables(getParentBroker(), fSensorKeys, false);
}



/**
 * @brief Initializes variables, that are necessary for fast sending of parameters to DCM.
 */
void oru_walk::initFastWrite(const vector<string>& joint_names)
{
    ALValue jointAliases;

    jointAliases.arraySetSize(2);
    jointAliases[1].arraySetSize(JOINTS_NUM);


    // positions of actuators.
    jointAliases[0] = std::string("jointActuator"); // Alias for all joint actuators
    // Create alias 
    try
    {
        for (int i = 0; i < JOINTS_NUM; i++)
        {
            jointAliases[1][i] = joint_names[i] + "/Position/Actuator/Value";
        }
        dcmProxy->createAlias(jointAliases);
    }
    catch (const ALError &e)
    {
        ORUW_THROW_ERROR("Error when creating Alias: ", e);
    }


    //  stiffness of actuators.
    jointAliases[0] = std::string("jointStiffness"); // Alias for all actuators
    // Create alias
    try
    {
        for (int i = 0; i < JOINTS_NUM; i++)
        {
            jointAliases[1][i] = joint_names[i] + "/Hardness/Actuator/Value";
        }
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
    // Create alias
    try
    {
        for (int i = 0; i < LOWER_JOINTS_NUM; i++)
        {
            jointAliases[1][i] = joint_names[i] + "/Position/Actuator/Value";
        }
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

    walkCommands[4].arraySetSize(1);

    walkCommands[5].arraySetSize(LOWER_JOINTS_NUM); // For all joints
    for (int i=0; i < LOWER_JOINTS_NUM; i++)
    {
        walkCommands[5][i].arraySetSize(1);
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
