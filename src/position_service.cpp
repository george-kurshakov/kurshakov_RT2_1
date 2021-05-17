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

class PositionServer : public rclcpp::Node
{

public:
  PositionServer(const rclcpp::NodeOptions &options) : Node("random_position_server", options)
  {
    /* Initialize the service */
    service_c = this->create_service<RandomPosition>( "/position_server", std::bind(&PositionServer::myrandom, this, _1, _2, _3));
  }
  
private:

  double randMToN(double M, double N)
  {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


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
  rclcpp::Service<RandomPosition>::SharedPtr service_c;

}; /* class ended*/

} /* namespace ended*/

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionServer)