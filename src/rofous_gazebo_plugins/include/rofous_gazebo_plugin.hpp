#ifndef _ROFOUS_PLUGIN_HH_
#define _ROFOUS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
	class RofousPlugin : public ModelPlugin
	{
		public : RofousPlugin(){}

		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    	{
      		// Just output a message for now
      		std::cerr << "\nThe Rofous plugin is attached to model[" <<
        	_model->GetName() << "]\n";
    	}

	};
	
	GZ_REGISTER_MODEL_PLUGIN(RofousPlugin)
}


#endif