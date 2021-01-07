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
		~Matrix();
		string matrix_as_string();
		int index2addr(int, int);
		int columns();
		int rows();
		void matmul(Matrix*);
		void reshape(int, int);
		int T();
		void clear();
		void set_matrix(vector<vector<float>>);
		int set(int, float);
		int set(int, int, float);
		int set(int, vector<int>, vector<float>);
		int set(vector<int>, int, vector<float>);
		int set(vector<int>, vector<int>, vector<vector<float>>);
		float get(int);
		float get(int, int);
		vector<float> get(int, vector<int>);
		vector<float> get(vector<int>, int);
		void get(vector<int>, vector<int>, Matrix*);
};

# endif