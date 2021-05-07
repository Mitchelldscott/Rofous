/**
	Unit tests for Transformer

	Author  : Mitchell Scott (misc4432@colorado.edu)
	Project : Rofous
	Version : 0.1
	Date    : January 31, 2020

**/
#include <vector>
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <Transformer.hpp>
#define Pi 3.141590

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

int validate_vector(vector<float> vect, Test* test, vector<float> check)
{
	string s;
	int e = 0;
	for(int i=0; i < vect.size(); i++)
	{
		if(fabs(vect[i] - check[i]) >= 1E-5)
			test->pass = false;
	}
}

void matrix_tests()
{
	cout << "-----Transformer Tests Begin-----\n";
	Test test = {true, "Getter Setter Test", ""};

	float val1 = Pi;
	float val2;
	vector<vector<float>> vect = {{1,2,3,1},{4,5,6,1},{7,8,9,1},{0,0,0,1}};
	vector<vector<float>> vect0 = {{2,0,0,0},{0,2,0,0},{0,0,2,0},{0,0,0,2}};
	Matrix m1(4,4);
	Matrix m2(vect);
	Matrix m3(vect0);

	auto start = chrono::high_resolution_clock::now();
	for(int i=0; i < vect.size() * vect[0].size(); i++)
		m1.set(i, val1);
	auto stop = chrono::high_resolution_clock::now();
	chrono::duration<double> set_time = stop - start;
	start = chrono::high_resolution_clock::now();
	for(int i=0; i < vect.size(); i++)
	{
		val2 = m1.get(i);
		if(val1 != val2)
		{
			test.pass = false;
			test.info += "\tFailed to read value from " + to_string(i) +" got " + to_string(val2) + "\n";
		}
	}

	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> get_time = stop - start;
	test.info += "\tget duration: " + to_string(set_time.count()) + 
	"\n\tset duration: " + to_string(get_time.count());
	cout << test_writeup(&test);

	test.label = "Slice Test";
	test.info = "";
	vector<vector<int>> slice1 = {{0,1,2}, {1,2,3}};
	start = chrono::high_resolution_clock::now();
	m2.get(slice1[0], slice1[0], &m1);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> slice_time = stop - start;
	test.info += "\tduration: " + to_string(slice_time.count());

	for(int i=0; i < slice1[0].size(); i++)
	{
		for(int j=0; j < slice1[0].size(); j++)
		{
			val1 = m1.get(i,j);
			val2 = m2.get(i,j);
			if(val1 != val2)
			{
				test.pass = false;
				test.info += "\n\tValues from (" + to_string(i) + "," + to_string(j) + ") do not match";
			}
		}
	}
	cout << test_writeup(&test);

	test.label = "Transpose Test";
	test.info = "";
	start = chrono::high_resolution_clock::now();
	m1.T();
	m2.T();
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> transpose_time = stop - start;
	test.info += "\n\tMatrix 1 Transpose:\n" + m1.as_string() + "\n\tMatrix 2 Transpose:\n" + m2.as_string() + "\n\tTranspose duration: " + to_string(transpose_time.count());
	cout << test_writeup(&test);

	test.label = "Reshape Test";
	test.info = "";
	start = chrono::high_resolution_clock::now();
	m1.reshape(4,4);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> reshape_time = stop - start;
	test.info +=  "\n\tReshape:\n" + m1.as_string() + "\n\tduration: " + to_string(reshape_time.count());
	cout << test_writeup(&test);

	test.label = "MatMul";
	test.info = "";
	start = chrono::high_resolution_clock::now();
	m2.matmul(&m3);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> mult_time = stop - start;
	test.info +=  "\n\tReturned\n" + m2.as_string() + "\n\tduration: " + to_string(mult_time.count());
	cout << test_writeup(&test);

	cout << "----- Testing Complete -----" << endl;
}

void transformer_tests()
{
	cout << "----- Transformer test begin -----" << endl;
	Test test = {true, "Transform Sample 0", ""};
	vector<vector<float>> x = {{1.0,0.0,0.0}};
	vector<vector<float>> y = {{0.0,1.0,0.0}};
	vector<vector<float>> z = {{0.0,0.0,1.0}};
	vector<float> check = {1.0,0.0,0.0};
	vector<int> temp = {0, 1, 2};

	vector<float> translation = {0.0, 0.0, 0.0};
	vector<float> reference = {(90.0 * Pi) / 180.0, 0.0, 0.0};
	vector<char> protocol = {'x', 'y', 'z'};
	string name = "World";
	Transformer tf(translation, reference, protocol, name);
	Matrix x_axis(x);
	Matrix y_axis(y);
	Matrix z_axis(z);

	test.info += "\n\tRaw X:" + x_axis.as_string() + "\n" +
	"\n\tRaw Y:" + y_axis.as_string() + "\n" +
	"\n\tRaw Z:" + z_axis.as_string() + "\n";
	auto start = chrono::high_resolution_clock::now();
	x_axis.T();
	y_axis.T();
	z_axis.T();
	test.info += "\n\tTransposed X:" + x_axis.as_string() + "\n" +
	"\n\tTransposed Y:" + y_axis.as_string() + "\n" +
	"\n\tTransposed Z:" + z_axis.as_string() + "\n";
	tf.rotate(&x_axis, true);
	tf.rotate(&y_axis, true);
	tf.rotate(&z_axis, true);
	auto stop = chrono::high_resolution_clock::now();
	chrono::duration<double> rot_time = stop - start;
	test.info += "\n\tRotated X:" + x_axis.as_string() + "\n" +
	"\n\tRotated Y:" + y_axis.as_string() + "\n" +
	"\n\tRotated Z:" + z_axis.as_string() + "\n" +
	"\n\tduration: " + to_string(rot_time.count()) + "\n";
	validate_vector(x_axis.get(temp, 0), &test, check);
	check = {0.0,0.0,1.0};
	validate_vector(y_axis.get(temp, 0), &test, check);
	check = {0.0,-1.0,0.0};
	validate_vector(z_axis.get(temp, 0), &test, check);
	cout << test_writeup(&test);


	test.label = "Transform Sample 1";
	test.info = "";
	test.info += "\n\tRaw X:" + x_axis.as_string() + "\n" +
	"\n\tRaw Y:" + y_axis.as_string() + "\n" +
	"\n\tRaw Z:" + z_axis.as_string() + "\n";
	x_axis.set_matrix(x);
	y_axis.set_matrix(y);
	z_axis.set_matrix(z);
	start = chrono::high_resolution_clock::now();
	x_axis.T();
	y_axis.T();
	z_axis.T();
	test.info += "\n\tTransposed X:" + x_axis.as_string() + "\n" +
	"\n\tTransposed Y:" + y_axis.as_string() + "\n" +
	"\n\tTransposed Z:" + z_axis.as_string() + "\n";
	tf.transform(&x_axis, true);
	tf.transform(&y_axis, true);
	tf.transform(&z_axis, true);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> tr_mult_time = stop - start;
	test.info += "\n\tTransformed X:" + x_axis.as_string() + "\n" +
	"\n\tTransformed Y:" + y_axis.as_string() + "\n" +
	"\n\tTransformed Z:" + z_axis.as_string() + "\n" +
	"\n\tduration: " + to_string(tr_mult_time.count()) + "\n";
	check = {1.0, 0.0, 0.0};
	validate_vector(x_axis.get(temp, 0), &test, check);
	check = {0.0,0.0,1.0};
	validate_vector(y_axis.get(temp, 0), &test, check);
	check = {0.0,-1.0,0.0};
	validate_vector(z_axis.get(temp, 0), &test, check);
	cout << test_writeup(&test);

	test.label = "Transform Sample 2";
	test.info = "";
	translation = {10.0, 0.0, 0.0};
	reference = {0.0, 0.0, (90.0 * Pi) / 180.0};
	Transformer tf1(translation, reference, protocol, "Body");
	tf1.parent = &tf;
	x_axis.set_matrix(x);
	y_axis.set_matrix(y);
	z_axis.set_matrix(z);
	start = chrono::high_resolution_clock::now();
	x_axis.T();
	y_axis.T();
	z_axis.T();
	tf1.transform(&x_axis, true);
	tf1.transform(&y_axis, true);
	tf1.transform(&z_axis, true);
	stop = chrono::high_resolution_clock::now();
	tr_mult_time = stop - start;
	test.info += "\n\tTransformed X:" + x_axis.as_string() + "\n" +
	"\n\tTransformed Y:" + y_axis.as_string() + "\n" +
	"\n\tTransformed Z:" + z_axis.as_string() + "\n" +
	"\n\tduration: " + to_string(tr_mult_time.count()) + "\n";
	check = {10.0, 0.0, 1.0};
	validate_vector(x_axis.get(temp, 0), &test, check);
	check = {9.0,0.0,0.0};
	validate_vector(y_axis.get(temp, 0), &test, check);
	check = {10.0,-1.0,0.0};
	validate_vector(z_axis.get(temp, 0), &test, check);
	cout << test_writeup(&test);

	cout << "\n\n----- Testing Complete -----" << endl;
}


int main(int argc, char** argv)
{
	
	matrix_tests();
	transformer_tests();
	return 0;
}