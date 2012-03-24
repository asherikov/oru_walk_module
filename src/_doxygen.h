/**
 * @file
 * @brief This file contains only doxygen definitions and should not be 
 *  included anywhere.
 *
 * @author Alexander Sherikov
 */


#ifndef DOXYGEN_H
#define DOXYGEN_H

/**
 * @mainpage A sparse MPC solver for walking motion generation.
 *
 * @par Contents
 * - @ref MainIntro
 * - @ref MainLicense
 * - @ref MainRef
 *
 * @par
 * - @ref pInstall
 * - @ref pArch
 * - @ref pImprove
 * - @ref pNotes
 * 
 * @par
 * - Homepage of the project: http://asherikov.github.com/Projects/naowalk.html
 * - Sources: http://github.com/asherikov/oru_walk_module
 * - Documentation: http://asherikov.github.com/oru_walk_module/index.html
 * \n
 *
 *
 * @section MainIntro Introduction
 * @verbinclude "README"
 * \n
 *
 *
 * @section MainLicense License
 * @verbinclude "LICENSE"
 * \n
 *
 *
 * @section MainRef References
 * @anchor AldNaoPaper
 * D. Gouaillier, C. Collette, and C. Kilner\n 
 * "Omni-directional Closed-loop Walk for NAO"\n 
 * IEEE-RAS International Conference on Humanoid Robots, pp. 448-454, 2010.\n
 */


/**
 * @page pInstall Installation and running
 * @verbinclude "INSTALL"
 */


/**
 * @page pArch Architecture and operation
 * 
 * @tableofcontents
 *
 * @section pGraph Connections between submodules
    @dot
    digraph arch {
        compound=true
        node [shape=box, style=filled];


        subgraph cluster0 {
            subgraph cluster1 {
                thread [label="Control thread"];
                igm [label="IGM:\n Inverse\n kinematics\n library"];
                solver [label="SMPC\n solver\n library"];
                wmg [label="WMG:\n footstep\n pattern\n generator"];

                thread -> igm;
                thread -> solver;
                thread -> wmg;

                label="Walking module";
                color=burlywood;
                style=filled;
                labeljust=l;
            }

            label="NaoQI";
            color=coral;
            style=filled;
            labeljust=l;
        }

        control [label="Remote control tool\n (oru_walk_control.py)"];
        control -> thread [lhead=cluster1];
    }
    @enddot
 * \n\n
 *
 * @section pControlLoop Control loop
 * A sequence diagram of one control loop:
  @msc
    dcm [label="DCM thread"], cb [label="DCM callback"], ct [label="Control thread"];
    dcm =>> cb [label = "Execute"];
    cb  =>  cb [label = "NOOP"];
    dcm =>> cb [label = "Execute"];
    cb  ->  ct [label = "Activate"];
    ct  =>  ct [label = "Feedback error"];
    ct  =>  ct [label = "Solve MPC"];
    ct  =>  ct [label = "Solve IK (1)"];
    ct  =>  ct [label = "Send joint angles (1)"];
    ct  =>  ct [label = "Solve IK (2)"];
    ct  =>  ct [label = "Send joint angles (2)"];
    ---        [ label = "Repeat until walk is finished"];
  @endmsc
 * The interconnection between DCM thread, DCM callback and control thread is explained
 * in section @ref pControlThread, this section also describes the parameters of the
 * control thread.
 *
 * Operations displayed on this diagram are described below.
 * \n\n
 *
 *
 * @section pControlThread Control thread
 *
 * We use a separate thread to control a robot, this thread is synchronized
 * with DCM. The control sampling time using in this thread is 20 ms. The
 * thread is spawned when the 'walk' command is received by the module.
 *
 * For some time we have been using hook (callback) to ALMotion module. The 
 * main reason why we switched to a separate thread was the lack of 
 * synchronization between DCM and ALMotion threads. This is important since 
 * DCM updates joint angles in the memory, and there is no other way to get 
 * the values of the angles.
 *
 * This is probably not a good solution, which would not work as well in future
 * releases of firmware.
 *
 * @subsection pDCMsync Synchronization with DCM
 * The DCM is activated each 10 ms. We register a callback function, which is
 * called when DCM finishes its work. Thus the callback function is also
 * executed each 10ms. The callback has a counter of executions, on each even
 * execution (i.e. each 20 ms.) it wakes up the control thread using conditional
 * variable -- a synchronization tool provided by the Boost.
 *
 * @subsection pThreadPriority Priority of the control thread
 * The priority of the control thread plays a critical role, since sometimes
 * it is not possible to complete all necessary computations in 20ms. using 
 * default priority. 
 *
 * We use SCHED_FIFO scheduling policy, the same as DCM thread uses. The priority
 * is set to 65. Note, that NaoQi does not run with root privileges and it limits
 * the possible changes of priorities. 
 * \n\n
 * 
 *
 * @section pFeedback Feedback
 * There are two types of feedback. 
 *
 * The first one is shown on sequence diagram above. The position of center of mass, that
 * is used as the initial position in the MPC solver, is corrected using the position
 * of center of mass computed from the sensor data. The feedback is applied only if the
 * difference between these positions is bigger than predefined threshold. Also, the error
 * is multiplied by a gain (< 1.0) before addition.
 *
 * The second one is required to avoid jumps of center of mass, when the support foot is 
 * changed. In this case the position of new support foot is not the same as expected due
 * to imprecision of movements. Therefore, we have to find the new position using sensors
 * and change this position in the footstep pattern. It is also necessary to synchronize
 * joint angles in the model of the robot with the joint angles obtained from sensors.
 * \n\n
 *
 *
 * @section pIGM Inverse kinematics
 * We compute inverse kinematics and set joint angles twice in one control loop. 
 *
 * The first computation produces the joint angles that must be reached in one
 * control sampling period, i.e. when the control thread is activated again. 
 * Since these joint angles are reached when the control loop is activated next 
 * time, the controllers have to have some commands to execute while the control 
 * loop works. The second solution is produced to keep controllers busy, it prevents 
 * jerky motions. The joint angles obtained in the second computation are expected 
 * to be reached in two control sampling periods. 
 * \n\n
 */


/**
 * @page pNotes Notes
 *
 * @section pNotesDoc Exerpts from the documentation
 * - LHipYawPitch and RHipYawPitch share the same motor so they move 
 *   simultaneously and symmetrically. In case of conflicting orders
 *   LHipYawPitch always takes the priority.
 *
 *
 * @section pNotesGen General
 * - A robot cannot execute the desired foot trajectories precisely. 
 *   The errors may be very different for the right and left foot.
 *
 * - If the sensor input from a joint is used as a command for this joint
 *   in a callback function, the joint becomes compliant. 
 *
 * - The callback function that controls the robot, can be hooked to DCM
 *   module or ALMotion module, in the first case it would be called each 
 *   10ms, in the second - 20 ms.
 *
 * - The DCM and ALMotion threads are not synchronized.
 *
 * - When the module is running on PC in simulation, RHipYawPitch is not
 *   updated and is always equal to 0.0. We haven't checked it on th robot
 *   but it is likely to behave the same way.
 *
 *
 *
 * @section pNotesOur Our module
 * - In the current version we switch between the left and the right 
 *   foot. We need to choose one foot as a reference to find a position 
 *   of CoM. In theory, it is possible to use the same foot, but when 
 *   this foot is lifted it's real position can be found only using the 
 *   other foot as a reference due to errors. A big error in foot position 
 *   will lead to a big error in CoM position.
 *
 * - Since we cannot follow the desired feet trajectories precisely (the real
 *   positions are lower), there is a significant error in CoM position along
 *   Z and Y axes.
 *
 *
 * @section pNotesMPC MPC solver
 * - The length of the preview window affects the ability of the robot to
 *   compensate for disturbances. On the other hand a long preview window 
 *   increases the number of constraints in the MPC problem. 
 *
 * - Number of active constraints has strong impact on performance. Under 
 *   strong disturbances the number of activated constraints in the MPC 
 *   makes it impossible to solve the problem in the available time.
 *
 *
 * @section pNotesIGM IGM
 * - When the orientaion of the torso is fixed (x and y axes), the hip roll
 *   joints are highly loaded and may hit their limits depending on the MPC
 *   gains.
 *
 *
 * @section pNotesLinks Useful resources
 * Documentation and software from Aldebaran Robotics are available on their
 * website:\n
 * http://users.aldebaran-robotics.com/
 *
 * Nice introductory tutorial on NAO:\n
 * http://robotica.unileon.es/mediawiki/index.php/Nao_tutorial_1:_First_steps
 * \n
 * http://robotica.unileon.es/mediawiki/index.php/Nao_tutorial_2:_First_module
 * \n
 * http://robotica.unileon.es/mediawiki/index.php/Nao_troubleshooting
 *
 * Cross-compilation using cmake:
 * http://www.cmake.org/Wiki/CMake_Cross_Compiling
 * \n\n
 */


/**
 * @page pImprove Possible improvements and other ideas
 *
 * @todo Inclination of the sole of the swing foot at the beginning of the swing.
 *
 * @todo Use a fan to produce disturbances.
 *
 * @todo Make a demo with variable height of the CoM.
 *
 * @todo It might be helpful to use interior point method instead of active
 * set method, since in the former case we may implement a mechanism to limit 
 * the time available for solution. Alternatively, we can limit the number of
 * activated constraints or execution time of active set method.
 */
#endif /*DOXYGEN_H*/
 
