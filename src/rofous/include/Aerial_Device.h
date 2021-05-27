/**
	Aerial Device class
	Author  : Mitchell Scott (misc4432@colorado.edu)
	Project : Rofous
	Version : 0.1
	Date    : January 31, 2020
**/
#include <Transformer.hpp>

using namespace std;

#ifndef __Aerial_Device_H__
#define __Aerial_Device_H__


class Aerial_Device
{
	private:
		float p;
		int nProps;
		float mass;
		float maxThrust;
		float drag_coef;
		Matrix stateActual;
		Matrix prop_config; 
		vector<int> spin_config;
		vector<float> sigActual;
		vector<Transformer*> autobots;

	public:
		Aerial_Device();
		Aerial_Device(int, float, float, float, vector<vector<float>>, vector<int>);
		~Aerial_Device();
		float get_p();
		float get_mass();
		float get_maxThrust();
		float get_drag_coef();
		int get_nProps();
		Matrix get_state();
		vector<float> get_throttle();
		void get_joint_pose(int, Matrix*);
		float radians(float);
		float wrap_angle(float);
		void build_transformers();
		vector<float> get_Fnet();
		Matrix update_odometry(float);
		void set_throttle(vector<float>);
		void adjust_throttle(vector<float>);
		int reset();
};

#endif