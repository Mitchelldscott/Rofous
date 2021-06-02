/**
	Matrix class

	Author  : Mitchell Scott (misc4432@colorado.edu)
	Project : Rofous
	Version : 0.1
	Date    : January 31, 2020

**/
#include <vector>

using namespace std;

#ifndef __Matrix_H__
#define __Matrix_H__


class Matrix
{
	private:
		vector<float> data;
		int n;
		int m;
	public:
		Matrix();
		Matrix(vector<vector<float>>);
		Matrix(int, int);
		~Matrix();
		string as_string();
		int index2addr(int, int);
		int columns();
		int rows();
		void matmul(Matrix*);
		void reshape(int, int);
		void T();
		void clear();
		void set_matrix(vector<vector<float>>);
		void set(int, float);
		void set(int, int, float);
		void set(int, vector<int>, vector<float>);
		void set(vector<int>, int, vector<float>);
		void set(vector<int>, vector<int>, vector<vector<float>>);
		float get(int);
		float get(int, int);
		vector<float> get(int, vector<int>);
		vector<float> get(vector<int>, int);
		void get(vector<int>, vector<int>, Matrix*);
};

# endif