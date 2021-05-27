#include <cmath>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <ostream>
#include <Aerial_Device.hpp>

using namespace std;

struct Test
{
	bool pass;
	string label;
	string info;
};

string test_writeup(Test* t)
{
	string s = "-----[" + t->label + "]-----\n";
	s += t->info;
	if(t->pass)
		s += "\n----- Test Passed -----\n";
	else
		s += "\n----- Test Failed -----\n";
	return s;
}

void validate_vector(vector<float> vect, Test* test, vector<float> check)
{
	string s;
	for(int i=0; i < vect.size(); i++)
	{
		if(fabs(vect[i] - check[i]) >= 1E-8)
		{
			test->pass = false;
		}
		s += to_string(vect[i]) + " ";
	}
	test->info += '\n' + s + '\n';

}

int main(int argc, char** argv)
{
	cout << "===== Begin AD Tests =====" << endl;
	Test test = {true, "Init", ""};
	float mass = 100;
	int nProps = 4;
	float drag_coef = -0.2;
	float max_thrust = 40;
	vector<vector<float>> config = {{2,2,0,0,0,135},
									{2,-2,0,0,0,45},
									{-2,-2,0,0,0,-45},
									{-2,2,0,0,0,-135}};
	vector<int> spin = {1, -1, 1, -1};

	Aerial_Device rofous(nProps, mass, max_thrust, drag_coef, config, spin);

	if(rofous.get_nProps() != 4)
		test.pass = false;
	test.info = "\tState:\n" + rofous.get_state().as_string();
	cout << test_writeup(&test);

	test = {true, "Throttle Set Test", ""};
	vector<float> check = {9, 9, 9, 9};
	rofous.set_throttle(check);
	vector<float> result = rofous.get_throttle();
	validate_vector(result, &test, check);
	cout << test_writeup(&test);

	test = {true, "Autobots build Test", ""};
	vector<vector<float>> point = {{0.0, 0.0, 0.0}};
	Matrix p(point);
	p.T();
	rofous.get_joint_pose(-1, &p);
	test.info = "Pose:\n" + p.as_string();
	cout << test_writeup(&test);
	point = {{0.0, 0.0, 0.0}};
	p.set_matrix(point);
	p.T();
	rofous.get_joint_pose(2, &p);
	test.info = "Pose:\n" + p.as_string();
	cout << test_writeup(&test);

	test = {true, "Fnet update Test", ""};
	vector<float> Fnet = rofous.get_Fnet();
	check = {0.0, 0.0, 315.0, 0.0, 0.0, 0.0};
	validate_vector(Fnet, &test, check);
	cout << test_writeup(&test);

	test = {true, "Odometry update Test", ""};
	Matrix state;
	for(int i=0; i < 10; i++)
	{
		state = rofous.update_odometry(0.05);
		test.info += "\n\nState{" + to_string(i) + "}:" + rofous.get_state().as_string();
	}
	cout << test_writeup(&test);
	cout << "===== End AD Tests =====" << endl;
	return 0;
}