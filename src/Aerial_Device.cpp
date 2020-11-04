#include <math.h>
#include <stdio.h>
#include <Aerial_Device.hpp>

using namespace std;

string pose_to_string(Pose3D pose)
{
	return "( " + to_string(pose.x) + ", " + to_string(pose.y) + ", " + to_string(pose.z) + ", " + to_string(pose.phi) + ", " + to_string(pose.theta) + ", " + to_string(pose.psi) + " )";
}

Analytic_Solver::Analytic_Solver()
{
	thrust_proportionality = 0.01;
	drag_coefficient = -0.3;
}

Analytic_Solver::Analytic_Solver(float tp, float dc)
{
	thrust_proportionality = tp;
	drag_coefficient = dc;
}

float Analytic_Solver::wrap_angle(float* radians)
{
	*radians = fmod((*radians + M_PI), (2 * M_PI));
	if(*radians > 0)
		*radians -= M_PI;
	else
		*radians += M_PI;
}

void Analytic_Solver::R(Pose3D* pose, float phi, float theta, float psi)
{
	vector<float> c1 = 
	{
		cos(theta) * cos(psi),
		sin(phi) * sin(theta) * cos(psi) - (cos(phi) * sin(psi)),
		cos(phi) * sin(theta) * cos(psi) + (sin(phi) * sin(psi))
	};

	vector<float> c2 = 
	{
		cos(theta) * sin(psi),
		sin(phi) * sin(theta) * sin(psi) + (cos(phi) * cos(psi)),
		cos(phi) * sin(theta) * sin(psi) - (sin(phi) * cos(psi))
	};

	vector<float> c3 =
	{ 
		- sin(theta),
		sin(phi) * cos(theta),
		cos(phi) * cos(theta)
	};

	pose->x += (c1[0] * pose->x) + (c1[0] * pose->y) + (c1[0] * pose->z);
	pose->y += (c2[1] * pose->x) + (c2[1] * pose->y) + (c2[1] * pose->z);
	pose->z += (c3[2] * pose->x) + (c3[2] * pose->y) + (c3[2] * pose->z);
	pose->phi += phi;
	pose->theta += theta;
	pose->psi += psi;
	wrap_angle(&pose->phi);
	wrap_angle(&pose->theta);
	wrap_angle(&pose->psi);
}

void Analytic_Solver::T(Pose3D* pose, float x, float y, float z)
{
	pose->x += x;
	pose->y += y;
	pose->z += z;
}

void Analytic_Solver::transform(Pose3D* pose, Pose3D ref)
{
	R(pose, ref.phi, ref.theta, ref.psi);
	T(pose, ref.x, ref.y, ref.z);
}

Pose3D Analytic_Solver::throttle_to_net_force(Pose3D pose, vector<float> throttle, float radius)
{
	vector<float> thrust = 
	{
		powf(throttle[0], 2.0) * thrust_proportionality,
		powf(throttle[1], 2.0) * thrust_proportionality,
		powf(throttle[2], 2.0) * thrust_proportionality,
		powf(throttle[3], 2.0) * thrust_proportionality,
	};

	vector<float> torque = 
	{
		- thrust[0] + thrust[1] + thrust[2] - thrust[3] * radius,
		thrust[0] + thrust[1] - thrust[2] - thrust[3] * radius,
		thrust[0] - thrust[1] + thrust[2] - thrust[3]
	};

	Pose3D thrust_earth = {0, 0, thrust[0] + thrust[1] + thrust[2] + thrust[3], torque[0], torque[1], torque[2]};
	R(&thrust_earth, pose.phi, pose.theta, pose.psi);
	return thrust_earth;
}

void Analytic_Solver::update_odometry(Pose3D* pose, Pose3D* speed, Pose3D acceleration, float t)
{
	speed->x += acceleration.x * t;
	speed->y += acceleration.y * t;
	speed->z += acceleration.z * t;
	speed->phi += acceleration.phi * t;
	speed->theta += acceleration.theta * t;
	speed->psi += acceleration.psi * t;

	pose->x += acceleration.x * pow(t, 2.0) / 2.0 + (speed->x * t);
	pose->y += acceleration.y * pow(t, 2.0) / 2.0 + (speed->y * t);
	pose->z += acceleration.z * pow(t, 2.0) / 2.0 + (speed->z * t);
	pose->phi += acceleration.phi * pow(t, 2.0) / 2.0 + (speed->phi * t);
	pose->theta += acceleration.theta * pow(t, 2.0) / 2.0 + (speed->theta * t);
	pose->psi += acceleration.psi * pow(t, 2.0) / 2.0 + (speed->psi * t);
}