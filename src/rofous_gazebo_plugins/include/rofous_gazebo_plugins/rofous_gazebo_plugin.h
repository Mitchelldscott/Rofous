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
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"


namespace gazebo
{
	class RofousPlugin : public ModelPlugin
	{
		private:
			std::string name;
			physics::LinkPtr link;
    		ros::Time elapsed_time;
    		physics::ModelPtr model; // Pointer to the model
    		ros::NodeHandle node_handle;
			event::ConnectionPtr updateFunc;// Pointer to the update event connection
		public: 
			RofousPlugin();
			virtual void Load(physics::ModelPtr, sdf::ElementPtr);
			void OnUpdate();
			int connectRosNodes();
			//void throttleCallBack();
	};

	GZ_REGISTER_MODEL_PLUGIN(RofousPlugin);
}
#endif