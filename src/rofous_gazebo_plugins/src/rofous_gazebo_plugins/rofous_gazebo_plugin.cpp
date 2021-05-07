#include <rofous_gazebo_plugin.hpp>



namespace gazebo {

	RofousPlugin::RofousPlugin()
	{
		float mass = 100;
		int nProps = 4;
		float drag_coef = -0.2;
		float max_thrust = 40;
		std::vector<std::vector<float>> config = {{2,2,0,0,0,135},
										{2,-2,0,0,0,45},
										{-2,-2,0,0,0,-45},
										{-2,2,0,0,0,-135}};
		std::vector<int> spin = {1, -1, 1, -1};

		device = Aerial_Device(nProps, mass, max_thrust, drag_coef, config, spin);
	}
				

	void RofousPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{

		model = _model;
		std::string link_name;
		name = _sdf->GetElement("robotNamespace")->Get<std::string>();
		link_name = _sdf->GetElement("linkName")->Get<std::string>();

		link = model->GetLink(link_name);
		if (link == NULL)
    		gzthrow("[gazebo_plugin] Couldn't find specified link \""<< link_name << "\".");
		
		ROS_INFO_STREAM("The plugin is attached to the model");
		ROS_INFO_STREAM("name " << name);
		ROS_INFO_STREAM("link_name " << link_name);
		ROS_INFO_STREAM("State:" << device.get_state().as_string());

		updateConnection = event::Events::ConnectWorldUpdateBegin(
		std::bind(&RofousPlugin::OnUpdate, this));

		elapsed_time = ros::Time::now();

		if (!ros::isInitialized())
		{
  			int argc = 0;
  			char **argv = NULL;
  			ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  			ROS_INFO("Initialized client");
  		}
		connectRosNodes();
	}

	// Called by the world update start event
	void RofousPlugin::OnUpdate()
	{
		ros::Duration cycle_time = ros::Time::now() - elapsed_time;
		device.update_odometry(cycle_time.toSec());
		elapsed_time = ros::Time::now();
		//ROS_INFO_STREAM("State: "  << device.get_state().as_string());

		vector<float> fNet = device.get_Fnet();
		const ignition::math::Vector3d forces = ignition::math::Vector3d (fNet[0], fNet[1], fNet[2]);
		const ignition::math::Vector3d moments = ignition::math::Vector3d (fNet[3], fNet[4], fNet[5]);
		link->AddRelativeForce(forces);
		link->AddRelativeTorque(moments);
	}

	int RofousPlugin::connectRosNodes()
	{
		// create the clock listener
		std::string set_throttle_topic = "/" + name + "/set_throttle";
		node = transport::NodePtr(new transport::Node());
		node->Init(model->GetWorld()->Name());
		//set_ThrottleSub = node->Subscribe(set_throttle_topic, &RofousPlugin::throttleCallBack, this);
	}

}