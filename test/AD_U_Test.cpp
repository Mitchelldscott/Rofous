/*
	Project: Rofous
	Author: Mitchell Scott

	TODO: Finish U-Tests and implement
	the Aerial Device dynamics simulator
*/

#include <iostream>
#include <fstream>
#include "AD_U_Test.hpp"

using namespace std;

void Journal::log(string s)
{
	history.push_back(s);
}

void Journal::dump(ofstream dest)
{
	dest << "----U-Test Log----" << endl;
	for(auto n:history)
	{
		dest << n << endl;
	}
	dest << "---- End ----" << endl;
}

vector<string> split(std::string s, char c)
{
	string buff = {""};
	vector<string> v;

	for(auto n:s)
	{
		if(n != c) 
			buff+=n;
		if(n == c && buff != "")
		{
			v.push_back(buff); 
			buff = ""; 
		}
	}

	if(buff != "") v.push_back(buff);
	return v;
}


vector<float> split_to_f(std::string s, char c)
{
	string buff = {""};
	vector<float> v;

	for(auto n:s)
	{
		if(n != c) 
			buff+=n;
		if(n == c && buff != "")
		{
			v.push_back(stof(buff)); 
			buff = ""; 
		}
	}

	if(buff != "") v.push_back(stof(buff));
	return v;
}


const vector<vector<vector<float>>> load_configs(string fileName) 
{
	vector<vector<vector<float>>> data;
	vector<vector<float>> test_data;
	vector<float> temp2;
	string raw_data;
	ifstream configs(fileName);

	if(configs)
	{
		while(getline(configs, raw_data))
		{
			test_data = {};
			temp = split(raw_data, ':');

			for(auto n:temp)
			{
				temp2 = split_to_f(n, ',');
				test_data.push_back(temp2);
			}
			data.push_back(test_data);
		}

		configs.close();

		return data;
	}
	cout << "File failed to open" << endl;
	return data;
}


int rotation_test(vector<vector<float>> v, Aerial_Device* device, Journal* book)
{
	vector<float> result;
	int s = v.size() / 2
	for(int i = 0; i < s; i++)
	{
		result = device.R(v[i]);
		if(result[0] != v[i+s][0] || result[1] != v[i+s][1] || result[2] != v[i+s][2])
		{
			book->log("Test Failed for " + result[0] + "," + result[1] + "," + result[2] + "\t" + v[i+s][0] + "," + v[i+s][1] + "," + v[i+s][2])
		}
	}
}

int translation_test(vector<vector<float>> v, Aerial_Device device, Journal* book)
{

}

int transform_test(vector<vector<float>> v, Aerial_Device device, Journal* book)
{

}

int pose_adjust_test(vector<vector<float>> v, Aerial_Device device, Journal* book)
{

}

int force_adjust_test(vector<vector<float>> v, Aerial_Device device, Journal* book)
{

}

int throttle_adjust_test(vector<vector<float>> v, Aerial_Device device, Journal* book)
{

}


int run_tests()
{
	Aerial_Device device();
	vector<vector<vector<float>>> data = load_configs("configs/AD_Tests.txt");
	int e_ctr = 0;

	

	return e_ctr;
}