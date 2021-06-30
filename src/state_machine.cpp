/**
 *  \file state_machine.cpp
 *  \brief A node implementing a state machine.
 *  
 *  The node is used to communicate with the user interface processing user commands and managing the robot behaviour.
 */

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PositionAction.h>

#define IDLE            0
#define SETTING_GOAL    1
#define WAITING_SUCCESS 2
#define STOPPING        3

/** \brief State variable.
 *  
 *  Can have following values:
 *  Value           |   Behaviour
 *  --------------- | -----------------
 *  IDLE            |   do not send any requests
 *  SETTING_GOAL    |   get a random position value from /position_server and send it as a goal to /go_to_point
 *  WAITING_SUCCESS |   check if the goal position has been reached
 *  STOPPING        |   send a goal cancel request to /go_to_point
 */
int state = IDLE;

bool isGoalCanceled = false;

/**
 *  \brief User interface server callback
 *  
 *  \param req service request
 *  \param res service response
 *  \return true if succeeded
 *  
 *  Callback is setting the state according to a received command:
 *     Command    |     State
 *  ------------- | -------------
 *      start     | SETTING_GOAL 
 *      stop      |     IDLE
 *     stop_now   |   STOPPING
 */
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	state = SETTING_GOAL;
    }
    if (req.command == "stop"){
    	state = IDLE;
    }
    if (req.command == "stop_now"){
    	if (isGoalCanceled)
    	    state = IDLE;
    	else
      	    state = STOPPING;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::PositionAction> ac("/go_to_point", true);
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::PositionGoal p;
   
   while(ros::ok()){
   	ros::spinOnce();
    switch (state) {
      case SETTING_GOAL:
        // Call /position_server for random position
        client_rp.call(rp);
        p.x = rp.response.x;
        p.y = rp.response.y;
        p.theta = rp.response.theta;
        std::cout << "\nGoing to the position: x= " << p.x << " y= " <<p.y << " theta = " <<p.theta << std::endl;
        // Send the position as a goal to /go_to_point
        ac.sendGoal(p);
        // Wait until the goal is reached
        state = WAITING_SUCCESS;
        break;
      case WAITING_SUCCESS:
        // Is the goal reached?
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          std::cout << "\nGoal position reached!" << std::endl;
          // Set a new goal
          state = SETTING_GOAL;
        }
        break;
      case STOPPING:
        // Send a goal cancel request to /go_to_point
        ac.cancelGoal();
        // Avoid canceling the goal when it's not set
        isGoalCanceled = true;
        // Wait for further instructions...
        state = IDLE;
    }
   }
   return 0;
}
