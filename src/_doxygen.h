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
 * @par Contents & links
 * - @ref MainIntro
 * - @ref MainLicense
 * - @ref MainInstall
 * - @ref MainRef
 * @par
 * - Sources: http://github.com/asherikov/oru_walk_module
 * - Documentation: http://asherikov.github.com/oru_walk_module/index.html
 * \n\n
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
 * @section MainInstall Installation
 * @verbinclude "INSTALL"
 * \n
 *
 *
 * @section MainRef References
 * @anchor AldNaoPaper
 * D. Gouaillier, C. Collette, and C. Kilner\n 
 * "Omni-directional Closed-loop Walk for NAO"\n 
 * IEEE-RAS International Conference on Humanoid Robots, pp. 448-454, 2010.\n
 * \n\n
 *
 *
 * @section MainAbout Description of the module
 *  - @ref pNotes
 * \n\n
 *
 *
 * @section MainLinks Useful resources
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
 * @page pNotes Notes
 *
 * Some notes are obvious, but sometimes they weren't taken into account, when 
 * it was necessary.
 *
 *
 * @section NotesDoc Exerpts from the documentation
 * - init() method of the module is called automatically when a library 
 *   is loaded.
 *
 * - LHipYawPitch and RHipYawPitch share the same motor so they move 
 *   simultaneously and symmetrically. In case of conflicting orders
 *   LHipYawPitch always takes the priority.
 *
 *
 * @section NotesGen General
 * - A robot cannot execute the desired foot trajectories precisely. 
 *   The errors may be very different for the right and left foot.
 *
 * - If the sensor input from a joint is used as a command for this joint
 *   in a callback function, the joint becomes compliant. 
 *
 * - It seems that, controllers of the joints work better, when the commands
 *   are sent more than 10ms in future.
 *
 * - The callback function that controls the robot, can be hooked to DCM
 *   module or ALMotion module, in the first case it would be called each 
 *   10ms, in the second - 20 ms.
 *
 *
 * @section NotesOur Our module
 * - In the current version we switch between the left and the right 
 *   foot. We need to choose one foot as a reference to find a position 
 *   of CoM. In theory, it is possible to use the same foot, but when 
 *   this foot is lifted it's real position can be found only using the 
 *   other foot as a reference due to errors. A big error in foot position 
 *   will lead to a big error in CoM position.
 *
 * - When the orientaion of the torso is fixed (x and y axes), the hip roll
 *   joints are highly loaded and may hit their limits depending on the MPC
 *   gains.
 *
 * - The length of the preview window affects the ability of the robot to
 *   compensate for disturbances. 
 *
 * - The long preview window increases the number of constraints in the MPC 
 *   problem thus increasing the time requred to solve it.
 *
 * - Under strong disturbances the number of activated constraints in the 
 *   MPC makes it impossible to solve the problem in the available time.
 *
 * @todo It might be helpful to use interior point method instead of active
 * set method, since in the former case we may implement a mechanism to limit 
 * the time available for solution.
 *
 * - Since we cannot follow the desired feet trajectories precisely (the real
 *   positions are lower), there is a significant error in CoM position along
 *   Z and Y axes, which doesn't allow to use lower error threshold and higher 
 *   feedback gain.
 */

#endif /*DOXYGEN_H*/
 
