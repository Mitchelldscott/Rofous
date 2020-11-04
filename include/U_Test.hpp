/*
	Project: Rofous
	Author: Mitchell Scott

	TODO: Finish U-Tests and implement
	the Aerial Device dynamics simulator
*/
#include <vector>
#include <string>
#include <fstream>
#include "Aerial_Device.hpp"
#ifndef __AD_U_TEST_H__
#define __AD_U_TEST_H__

class Journal
{
	private:

		std::vector<std::string> history;

	public:

		void log(std::string);
		void dump(std::ostream&);
};

// the tests
//int render_test();
int wrap_angle_test(Analytic_Solver, Journal*);
int rotation_test(std::vector<Pose3D>, Analytic_Solver, Journal*);
int translation_test(std::vector<Pose3D>, Analytic_Solver, Journal*);
int transform_test(std::vector<Pose3D>, Analytic_Solver, Journal*);

// basically main
int run_tests();




#endif