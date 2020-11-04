/*
	Project: Rofous
	Author: Mitchell Scott

	TODO: Finish U-Tests and implement
	the Aerial Device dynamics simulator
*/

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "U_Test.hpp"

using namespace std;

void Journal::log(string s)
{
	history.push_back(s);
}

void Journal::dump(ostream& dest)
{
	dest << "----U-Test Log----" << endl;
	for(auto n:history)
	{
		dest << n << endl;
	}
	dest << "---- End ----" << endl;
}

int wrap_angle_test(Analytic_Solver ans, Journal* book)
{
	vector<float> angles = {0, M_PI, -M_PI, 2 * M_PI, -2 * M_PI, 3 * M_PI, -3 * M_PI, 4 * M_PI};
	vector<float> solutions = {0, -M_PI, M_PI, 0, 0, -M_PI, M_PI, 0};
	int e_ctr = 0;

	for(int i=0; i < angles.size(); i++)
	{
		ans.wrap_angle(&angles[i]);

		if(fabs(angles[i] - solutions[i]) >= 0.01)
		{
			book->log("Wrap angles failed on " + to_string(angles[i]) + " : " + to_string(solutions[i]));
			e_ctr ++;
		}
		else
			book->log("Wrap angles " + to_string(i) + " passed!");
	}
	return e_ctr;
}

int translation_test(vector<Pose3D> v, Analytic_Solver ans, Journal* book)
{
	int e_ctr = 0;
	for(int i=0; i < v.size(); i++)
	{
		bool passed = true;

		ans.T(&v[i], -v[i].x, -v[i].y, -v[i].z);

		if(v[i].x != 0)
		{
			book->log("Translation Failed on x for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].y != 0)
		{
			book->log("Translation Failed on y for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].z != 0)
		{
			book->log("Translation Failed on z for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(passed)
			book->log("Translation " + to_string(i) + " passed!");

	}
}

int rotation_test(vector<Pose3D> v, Analytic_Solver ans, Journal* book)
{
	int e_ctr = 0;
	for(int i=0; i < v.size(); i++)
	{
		bool passed = true;

		ans.R(&v[i], -v[i].phi, -v[i].theta, -v[i].psi);

		if(v[i].phi >= 0.0001)
		{
			book->log("Rotation Failed on phi for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].theta >= 0.0001)
		{
			book->log("Rotation Failed on theta for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].psi >= 0.0001)
		{
			book->log("Rotation Failed on psi for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(passed)
			book->log("Rotation " + to_string(i) + " passed!");
	}
	return e_ctr;
}

int transform_test(vector<Pose3D> v, Analytic_Solver ans, Journal* book)
{
	int e_ctr = 0;
	for(int i=0; i < v.size(); i++)
	{
		bool passed = true;

		Pose3D ref = {-v[i].x, -v[i].y, -v[i].z, -v[i].phi, -v[i].theta, -v[i].psi};
		Pose3D r_ref = ref;
		ans.R(&r_ref, ref.phi, ref.theta, ref.psi);

		r_ref.phi = ref.phi;
		r_ref.theta = ref.theta;
		r_ref.psi = ref.psi;
		
		ans.transform(&v[i], r_ref);

		if(v[i].x >= 0.0001)
		{
			book->log("Transform Failed on x for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].y >= 0.0001)
		{
			book->log("Transform Failed on y for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].z >= 0.0001)
		{
			book->log("Transform Failed on z for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].phi >= 0.0001)
		{
			book->log("Transform Failed on phi for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].theta >= 0.0001)
		{
			book->log("Transform Failed on theta for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(v[i].psi >= 0.0001)
		{
			book->log("Transform Failed on psi for pose " + pose_to_string(v[i]));
			passed = false;
			e_ctr ++;
		}
		if(passed)
			book->log("Transform " + to_string(i) + " passed!");
	}
	return e_ctr;
}

int run_tests()
{
	Journal book;
	Analytic_Solver ans;
	vector<Pose3D> samples;
	float shifter = RAND_MAX / 2;

	for(int i=0; i < 50; i++)
	{
		float phi = rand() - shifter;
		float theta = rand() - shifter;
		float psi = rand() - shifter;
		ans.wrap_angle(&phi);
		ans.wrap_angle(&theta);
		ans.wrap_angle(&psi);

		Pose3D pose = {rand() - shifter, rand() - shifter, rand() - shifter, phi, theta, psi};
		samples.push_back(pose);
	}

	int e_ctr = 0;

	e_ctr += wrap_angle_test(ans, &book);
	e_ctr += translation_test(samples, ans, &book);
	e_ctr += rotation_test(samples, ans, &book);
	e_ctr += transform_test(samples, ans, &book);
	
	book.dump(cout);
	return e_ctr;
}