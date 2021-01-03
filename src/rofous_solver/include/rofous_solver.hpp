#include <vector>
#include <string>
#include <fstream> 
#include <iostream>

using namespace std;

#ifndef __ROFOUS_SOLVER_H__
#define __ROFOUS_SOLVER_H__

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
		Pose3D quadcopter_throttle_to_force(Pose3D, std::vector<float>, float);
		void update_3D_odometry(Pose3D*, Pose3D*, Pose3D, float);
};

# endif