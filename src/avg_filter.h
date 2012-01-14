/**
 * @file
 * @author Alexander Sherikov
 * @date 14.01.2012 18:47:33 MSK
 */


#ifndef AVG_FILTER_H
#define AVG_FILTER_H

/****************************************
 * INCLUDES 
 ****************************************/

#include <vector>
#include <list>

/****************************************
 * DEFINES
 ****************************************/

/****************************************
 * TYPEDEFS 
 ****************************************/

using namespace std;


/**
 * @brief Moving average filter for two values.
 */
class avgFilter 
{
    public:
        /**
         * @brief Initialize using equal weights.
         *
         * @param[in] length length of the window.
         */
        avgFilter(const unsigned int length) : weights(length, 1.0) {};


        /**
         * @brief Initialize using given weights.
         *
         * @param[in] weights_ a vector of weights
         *
         * @note The size of the window = the size of the wector of weights.
         */
        avgFilter(const vector<double>& weights_) : weights (weights_) {};


        /**
         * @brief Adds a new value and computes average using updated values.
         *
         * @param[in] xval new value x
         * @param[in] yval new value y
         * @param[out] xavg averaged x
         * @param[out] yavg averaged y
         */
        void addValue (const double xval, const double yval, double& xavg, double& yavg)
        {
            xvalues.push_front(xval);
            yvalues.push_front(yval);
            if (xvalues.size() > weights.size())
            {
                xvalues.pop_back();
                yvalues.pop_back();
            }

            double total_weight = 0.0;
            unsigned int i = 0;
            list<double>::iterator xit = xvalues.begin();
            list<double>::iterator yit = yvalues.begin(); 
            xavg = yavg = 0.0;
            for (; xit != xvalues.end(); xit++, yit++)
            {
                total_weight += weights[i];
                xavg += weights[i] * (*xit);
                yavg += weights[i] * (*yit);
                i++;
            }
            xavg /= total_weight;
            yavg /= total_weight;
        };


// variables
        list<double> xvalues;
        list<double> yvalues;
        vector<double> weights;
};
#endif // AVG_FILTER_H

