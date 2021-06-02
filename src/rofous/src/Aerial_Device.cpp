/**
	Aerial Device class

	Author  : Mitchell Scott (misc4432@colorado.edu)
	Project : Rofous
	Version : 0.1
	Date    : January 31, 2020

**/
#include <cmath>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <ostream>
#include <Aerial_Device.hpp>
#define Pi 3.1415926535

using namespace std;

Aerial_Device::Aerial_Device()
{
	p = 0.0;
	nProps = 0;
	mass = 0.0;
	maxThrust = 0.0;
	drag_coef = 0.0;
}

Aerial_Device::Aerial_Device(int props, float m, float max_thrust, float drag, vector<vector<float>> config, vector<int> spin)
{
	assert(m >= 0.0);
	assert(drag <= 0.0);
	assert(max_thrust >= 0.0);
	assert(spin.size() == props);
	assert(config.size() == props);

	mass = m;
	nProps = props;
	drag_coef = drag;
	spin_config = spin;
	p = max_thrust / 10.0;
	maxThrust = max_thrust;
	prop_config.set_matrix(config);
	vector<vector<float>> default_state = {{0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0}};
	stateActual.set_matrix(default_state);
	vector<float> default_sig(nProps, 0.0);
	sigActual = default_sig;
	reset();
	build_transformers();
}

Aerial_Device::~Aerial_Device()
{

}

float Aerial_Device::radians(float degree)
{
	return (degree * Pi) / 180.0;
}

float Aerial_Device::wrap_angle(float alpha)
{
	return ((alpha / Pi) - floor(alpha / Pi)) * Pi;
}

Matrix Aerial_Device::get_state()
{
	Matrix state = stateActual;
	return state;
}

int Aerial_Device::get_nProps()
{
	return nProps;
}

vector<float> Aerial_Device::get_throttle()
{
	return sigActual;
}

void Aerial_Device::get_joint_pose(int i, Matrix* pose)
{
	if(i >= -1 && i < nProps)
		autobots[i+1]->transform(pose, true);
}

void Aerial_Device::set_throttle(vector<float> throttle)
{
	assert(throttle.size() == nProps);
	for(int i=0; i < nProps; i++)
		sigActual[i] = fmin(100.0, fmax(0.0, throttle[i]));
}

void Aerial_Device::adjust_throttle(vector<float> throttle)
{
	assert(throttle.size() == nProps);
	for(int i=0; i < nProps; i++)
		sigActual[i] = fmin(100.0, fmax(0.0, sigActual[i] + throttle[i]));
}

void Aerial_Device::build_transformers()
{
	autobots.clear();
	vector<char> proto = {'x', 'y', 'z'};
	vector<int> pose = {0,1,2};
	vector<int> ref = {3,4,5};
	vector<float> v = {0.0, 0.0, 0.0};
	autobots.push_back(new Transformer(stateActual.get(0,pose), stateActual.get(0,ref), proto, "Body"));
	for(int i=0; i < nProps; i++)
	{
		for(int j=3; j < 6; j++)
		{
			v[j-3] = radians(prop_config.get(i,j));
		}
		autobots.push_back(new Transformer(prop_config.get(i, pose), v, proto, "Joint" + to_string(i)));
		autobots[i+1]->parent = autobots[0];
	}
}

vector<float> Aerial_Device::get_Fnet()
{
	float d;
	float sig;
	vector<float> lin_thrust(3, 0.0);
	vector<float> torque(3, 0.0);
	vector<vector<float>> t_vect;
	Matrix thrust;

	for(int i=0; i < nProps; i++)
	{
		sig = pow(sigActual[i], 2.0) * p;
		t_vect = {{0.0}, {0.0}, {sig}};
		thrust.set_matrix(t_vect);
		autobots[i+1]->rotate(&thrust, true);
		lin_thrust[0] += thrust.get(0,0);
		lin_thrust[1] += thrust.get(1,0);
		lin_thrust[2] += thrust.get(2,0);
		d = sqrt(pow(prop_config.get(i,0), 2.0) + pow(prop_config.get(i,1), 2.0) + pow(prop_config.get(i,2), 2.0));
		t_vect = {{-sig * d}, {0.0}, {sig * spin_config[i]}};
		thrust.set_matrix(t_vect);
		autobots[i+1]->rotate(&thrust, true);
		torque[0] += thrust.get(0,0);
		torque[1] += thrust.get(1,0);
		torque[2] += thrust.get(2,0);
	}

	for(int i=0; i < torque.size(); i++)
	{
		lin_thrust[i] += (stateActual.get(1, i) * drag_coef);
		torque[i] += (stateActual.get(1, i+3) * drag_coef);
	}

	lin_thrust.insert(lin_thrust.end(), torque.begin(), torque.end());
	return lin_thrust;
}

Matrix Aerial_Device::update_odometry(float t)
{
	vector<float> accel = get_Fnet();
	//vector<float> imu = read_IMU();
	vector<int> pose = {0,1,2};
	vector<int> ref = {3,4,5};
	vector<char> proto = {'x','y','z'};

	for(int i=0; i < accel.size() / 2; i++)
	{
		accel[i] = accel[i] / mass;
		stateActual.set(0, i, stateActual.get(0,i) + (stateActual.get(1, i) * t));
		stateActual.set(1, i, stateActual.get(1, i) + (accel[i] * t));
		accel[i+3] = accel[i+3] / mass;
		stateActual.set(0, i+3, wrap_angle(stateActual.get(0, i+3) + (stateActual.get(1, i+3) * t)));
		stateActual.set(1, i+3, wrap_angle(stateActual.get(1, i+3) + (accel[i+3] * t)));
	}
	autobots[0] = new Transformer(stateActual.get(0,pose), stateActual.get(0,ref), proto, "Body");
	for(int i=1; i <= nProps; i ++)
	{
		autobots[i]->parent = autobots[0];
	}
	
	Matrix m;
	ref = {0,1};
	stateActual.get(ref, pose, &m);
	return m;
}

int Aerial_Device::reset()
{
	stateActual.clear();
	fill(sigActual.begin(), sigActual.end(), 0.0);
	
	assert(stateActual.rows() == 2);
	assert(stateActual.columns() == 6);
	assert(sigActual.size() == nProps);
}