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

int state = IDLE;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	state = SETTING_GOAL;
    }
    if (req.command == "stop"){
    	state = IDLE;
    }
    if (req.command == "stop_now"){
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
        client_rp.call(rp);
        p.x = rp.response.x;
        p.y = rp.response.y;
        p.theta = rp.response.theta;
        std::cout << "\nGoing to the position: x= " << p.x << " y= " <<p.y << " theta = " <<p.theta << std::endl;
        ac.sendGoal(p);
        state = WAITING_SUCCESS;
        break;
      case WAITING_SUCCESS:
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          std::cout << "\nGoal position reached!" << std::endl;
          state = SETTING_GOAL;
        }
        break;
      case STOPPING:
        ac.cancelGoal();
        state = IDLE;
    }
   }
   return 0;
}
