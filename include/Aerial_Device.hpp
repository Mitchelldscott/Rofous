#include <vector>
#include <string>
#include <fstream> 
#include <iostream>

using namespace std;

struct Pose3D
{
	float x;
	float y;
	float z;
	float phi;
	float theta;
	float psi;
};

class Analytic_Solver
{
	private:

		float thrust_proportionality;
		float drag_coefficient;

	public:

		Analytic_Solver();
		Analytic_Solver(float, float);
		float wrap_angle(float*);
		void R(Pose3D*, float, float, float);
		void T(Pose3D*, float, float, float);
		void transform(Pose3D*, Pose3D);
		Pose3D throttle_to_net_force(Pose3D, std::vector<float>, float);
		void update_odometry(Pose3D*, Pose3D*, Pose3D, float);

};

class AerialDevice
{
	private:

		Pose3D pose = {0, 0, 0, 0, 0, 0};
		Pose3D speed = {0, 0, 0, 0, 0, 0};
		Pose3D force = {0, 0, 0, 0, 0, 0};
		Analytic_Solver ans();


	public:

		Pose3D get_pose();
		void set_pose(Pose3D);
		void adjust_pose(Pose3D);
		Pose3D get_speed();
		void set_speed(Pose3D);
		void adjust_speed(Pose3D);
		Pose3D get_force();
		void set_force(Pose3D);
		void adjust_force(Pose3D);
};

string pose_to_string(Pose3D);

