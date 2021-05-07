#ifndef _ROFOUS_PLUGIN_HH_
#define _ROFOUS_PLUGIN_HH_

#include <vector>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "ros/ros.h"
#include "Aerial_Device.hpp"
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"


namespace gazebo
{
	class RofousPlugin : public ModelPlugin
	{
		private:
			std::string name;
			Aerial_Device device;
			physics::LinkPtr link;
			vector<float> throttle;
    		physics::ModelPtr model; // Pointer to the model
    		ros::Subscriber sim_clock;
    		ros::NodeHandle node_handle;
    		ros::Time elapsed_time;
    		transport::NodePtr node;
			transport::SubscriberPtr set_ThrottleSub;
			event::ConnectionPtr updateConnection;// Pointer to the update event connection
		public: 
			RofousPlugin();
			virtual void Load(physics::ModelPtr, sdf::ElementPtr);
			void OnUpdate();
			int connectRosNodes();
			//void throttleCallBack();
	};
	
	GZ_REGISTER_MODEL_PLUGIN(RofousPlugin)
}


#endif