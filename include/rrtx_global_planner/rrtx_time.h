#ifndef RRTX_TIME_H
#define RRTX_TIME_H

#include <chrono>



// from OMPL
namespace rrtx_time
{
    /** \brief Representation of a point in time */
    using point = std::chrono::system_clock::time_point;

    /** \brief Representation of a time duration */
    using duration = std::chrono::system_clock::duration;

    /** \brief Get the current time point */
    inline point now()
    {
        return std::chrono::system_clock::now();
    }

    /** \brief Return the time duration representing a given number of seconds */
    inline duration seconds(double sec)
    {
        auto s = (long)sec;
        auto us = (long)((sec - (double)s) * 1000000);
        return std::chrono::seconds(s) + std::chrono::microseconds(us);
    }

    /** \brief Return the number of seconds that a time duration represents */
    inline double seconds(const duration &d)
    {
        return std::chrono::duration<double>(d).count();
    }
}

#endif