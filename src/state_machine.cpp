/**
 * @file state_machine.cpp
 * @brief A node implementing a state machine.
 *  
 *  The node is used to communicate with the user interface processing user commands and managing the robot behaviour.
 */
#include <chrono>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using namespace std::chrono_literals;

using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
/**
 * @brief A class implementing a state machine.
 * 
 */
class StateMachine : public rclcpp::Node
{
public:
/**
 * @brief Construct a new State Machine object
 * 
 * @param options 
 */
  StateMachine(const rclcpp::NodeOptions & options)
  : Node("state_machine",options)
  {
    start = false;
    goal_reached = true;
    /* Initialize the service */
    service_c = this->create_service<Command>( "/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
    
    /* Initialize the clients */
    client_rp = this->create_client<RandomPosition>("/position_server");
    while (!client_rp->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    client_p = this->create_client<Position>("/go_to_point");
    while (!client_p->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    request_rp = std::make_shared<RandomPosition::Request>();
    response_rp = std::make_shared<RandomPosition::Response>();
    request_p = std::make_shared<Position::Request>();
    
    request_rp->x_max = 5.0;
    request_rp->x_min = -5.0;
    request_rp->y_max = 5.0;
    request_rp->y_min = -5.0; 
    
    timer_ = this-> create_wall_timer( 500ms, std::bind(&StateMachine::state_goal,this));
  }
  
  
 private:
 
 /**
  * @brief A user interface server callback
  * 
  * @param request_id Request ID
  * @param request Service request
  * @param response Service response
  */
   void user_interface(
         const std::shared_ptr<rmw_request_id_t> request_id,
         const std::shared_ptr<Command::Request> request,
         const std::shared_ptr<Command::Response> response)
   {
     (void) request_id;
     if(request->command == "start"){
       
       start = true;
       
       }  
     else{start = false;}
     response->ok = start;
     RCLCPP_INFO(this->get_logger(), "Request received %s", request->command.c_str());
   }
   
   /**
    * @brief A function checking the state of the goal.
    * Calls go_to_point if requested.
    */
  void state_goal(){
    
     if(!goal_reached) return;
     
     if(!start) return;
     
     goToPoint_call();
   
   }
   
   /**
    * @brief A function calling go_to_point service.
    * 
    */
   void goToPoint_call(){
   
     randomPosition_call();
     
     goal_reached = false;
     
     request_p->x = response_rp->x;
     request_p->y = response_rp->y;
     request_p->theta = response_rp->theta;
     
     RCLCPP_INFO(this->get_logger(), "Going to position: x = %f, y = %f, theta = %f", request_p->x, request_p->y, request_p->theta); 
     
     auto goal_reached_callback = [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future){(void)future; goal_reached = true;
                                  RCLCPP_INFO(this->get_logger(), "Goal is reached!!!");};
     auto fResult = client_p -> async_send_request(request_p, goal_reached_callback);
   }
   
   /**
    * @brief A function calling random_position_server service.
    * 
    */
   void randomPosition_call(){
   
     auto response_rp_callback = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future){response_rp = future.get();};
     
     auto fResult = client_rp-> async_send_request(request_rp, response_rp_callback);
   }

   /**
    * @brief A shared pointer to user_interface service
    */
   rclcpp::Service<Command>::SharedPtr service_c;
   /**
    * @brief A shared pointer to random_position_service client
    */
   rclcpp::Client<RandomPosition>::SharedPtr client_rp;
   /**
    * @brief A shared pointer to go_to_point client
    */
   rclcpp::Client<Position>::SharedPtr client_p;
   
   std::shared_ptr<RandomPosition::Request> request_rp;
   std::shared_ptr<RandomPosition::Response> response_rp;
   std::shared_ptr<Position::Request> request_p;
   
   bool start;
   bool goal_reached;
   
   rclcpp::TimerBase::SharedPtr timer_;
 };

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)