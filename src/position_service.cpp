/**
 *  \file position_service.cpp
 *  \brief A node implementing the random position service
 */
#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 *  \brief Get a random number from M to N
 *  
 *  \param M Minimum value
 *  \param N Maximum value
 *  \return A random number from M to N
 *  
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 *  \brief Random Position server callback
 *  
 *  \param req Service request
 *  \param res Servise response
 *  \return True is succeeded
 *  
 *  \details Responds with a random position.
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
