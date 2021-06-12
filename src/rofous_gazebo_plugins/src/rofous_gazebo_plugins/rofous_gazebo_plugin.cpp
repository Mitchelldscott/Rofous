#include <rofous_gazebo_plugin.h>



namespace gazebo {

	RofousPlugin::RofousPlugin()
	{}

	void RofousPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{

		model = _model;
		std::string link_name;
		name = _sdf->GetElement("robotNamespace")->Get<std::string>();
		link_name = _sdf->GetElement("linkName")->Get<std::string>();

		link = model->GetLink(link_name);
		if (link == NULL)
    		gzthrow("[gazebo_plugin] Couldn't find specified link \""<< link_name << "\".");
		
		ROS_INFO_STREAM("Namespace: " << name);
		ROS_INFO_STREAM("link_name " << link_name);

		updateFunc = event::Events::ConnectWorldUpdateBegin(std::bind(&RofousPlugin::OnUpdate, this));

		elapsed_time = ros::Time::now();

		if (!ros::isInitialized())
		{
  			int argc = 0;
  			char **argv = NULL;
  			ros::init(argc, argv, "gazebo_client");
  			ROS_INFO("Initialized client");
  		}
		connectRosNodes();
	}

	void RofousPlugin::OnUpdate()
	{
		ros::Duration cycle_time = ros::Time::now() - elapsed_time;
		elapsed_time = ros::Time::now();
	}

	int RofousPlugin::connectRosNodes()
	{
		return 0;
	}

}