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
 * @page pNotes Notes
 * - If the sensor input from a joint is used as a command for this joint
 *   in a callback function, the joint becomes compliant. 
 *
 * - The DCM and ALMotion threads are not synchronized (at least in simulation).
 *
 * - When the module is running on PC in simulation, RHipYawPitch is not
 *   updated and is always equal to 0.0. We haven't checked it on th robot
 *   but it is likely to behave the same way.
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
#endif /*DOXYGEN_H*/
 
