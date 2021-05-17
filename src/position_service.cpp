/**
 * @file position_service.cpp
 * @brief A node implementing the random position service
 */
#include "rt2_assignment1/srv/random_position.hpp"
#include <memory>
#include <inttypes.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

/**
 * @brief A class implementing the random position service
 * 
 */
class PositionServer : public rclcpp::Node
{

public:
/**
 * @brief Construct a new Position Server object
 * 
 * @param options 
 */
  PositionServer(const rclcpp::NodeOptions &options) : Node("random_position_server", options)
  {
    service_c = this->create_service<RandomPosition>( "/position_server", std::bind(&PositionServer::myrandom, this, _1, _2, _3));
  }
  
private:

/**
 * @brief Get a random number from M to N
 * 
 * @param M Minimum value
 * @param N Maximum value
 * @return double A random number from M to N
 */
  double randMToN(double M, double N)
  {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * @brief Random Position server callback
 * 
 * @param request_id Request ID
 * @param request Service request
 * @param response Service response
 */
  void myrandom (
      const std::shared_ptr<rmw_request_id_t> request_id,
      const std::shared_ptr<RandomPosition::Request> request,
      const std::shared_ptr<RandomPosition::Response> response)
  {
    (void)request_id;
    response->x = randMToN(request->x_min, request->x_max);
    response->y = randMToN(request->y_min, request->y_max);
    response->theta = randMToN(-3.14, 3.14);
    
  }
  /**
   * @brief A shared pointer to the service.
   * 
   */
  rclcpp::Service<RandomPosition>::SharedPtr service_c;

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionServer)