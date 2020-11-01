/*
	Project: Rofous
	Author: Mitchell Scott

	TODO: Finish U-Tests and implement
	the Aerial Device dynamics simulator
*/
#include <vector>
#include <string>
#ifndef __AD_U_TEST_H__
#define __AD_U_TEST_H__

class Journal
{
	private:
		vector<string> history;
	public:
		void log(string);
		void dump(ofstream);
};

// helpers
std::vector<std::string> split(std::string, char);
std::vector<float> split_to_f(std::string, char);

// to load the pre calculated test inputs and outputs
const std::vector<std::vector<std::vector<float>>> load_configs(std::string);

// the tests
int render_test();
int rotation_test(vector<vector<float>>, Aerial_Device, Journal*);
int translation_test(vector<vector<float>>, Aerial_Device, Jounal*);
int transform_test(vector<vector<float>>, Aerial_Device, Jounal*);
int pose_adjust_test(vector<vector<float>>, Aerial_Device, Jounal*);
int force_adjust_test(vector<vector<float>>, Aerial_Device, Jounal*);
int throttle_adjust_test(vector<vector<float>>, Aerial_Device, Jounal*);

// basically main
int run_tests();




#endif