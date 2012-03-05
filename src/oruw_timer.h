/**
 * @file
 * @author Alexander Sherikov
 * @date 05.03.2012 17:35:04 MSK
 */


#ifndef ORUW_TIMER_H
#define ORUW_TIMER_H

#include <qi/os.hpp>

#include "oruw_log.h"


/**
 * @brief Log time of existance on destruction. Throw an error, if the given
 * upper limit is not satisfied.
 */
class oruw_timer
{
    public:
        /**
         * @brief Create a timer
         *
         * @param[in] timer_id identifier of the timer
         * @param[in] timer_limit the upper limit
         */
        oruw_timer(const char* timer_id, const unsigned int timer_limit_ms) : id(timer_id)
        {
            qi::os::gettimeofday (&start_time);
            limit = (double) timer_limit_ms / 1000; // in seconds
        }


        /**
         * @brief Reset timer to the current time
         */
        void reset()
        {
            qi::os::gettimeofday (&start_time);
        }


        /**
         * @brief Checks if the limit is satisfied, logs a message.
         *
         * @return true if the limit is satisfied, false otherwise.
         */
        bool check()
        {
            qi::os::gettimeofday (&end_time);
            double timediff = ((double) end_time.tv_sec - start_time.tv_sec 
                            + 0.000001* (end_time.tv_usec - start_time.tv_usec));
            ORUW_LOG_MESSAGE("Checking timer '%s': value = %f / limit = %f\n", id, timediff, limit);
            return (timediff <= limit);
        }


    private:
        qi::os::timeval end_time;
        qi::os::timeval start_time;

        const char *id;
        double limit;
};
#endif // ORUW_TIMER_H
