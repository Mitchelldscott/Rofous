/**
	Matrix class

	Author  : Mitchell Scott (misc4432@colorado.edu)
	Project : Rofous
	Version : 0.1
	Date    : January 31, 2020

**/
#include <math.h>
#include <stdio.h>
#include <ostream>
#include <iostream>
#include <assert.h>
#include <Matrix.hpp>
#define Pi 3.1415926535

using namespace std;

Matrix::Matrix()
{
	n = 3;
	m = 3;
	data = {1.0,0.0,0.0,
		0.0,1.0,0.0,
		0.0,0.0,1.0};
}

Matrix::Matrix(vector<vector<float>> mat)
{
	set_matrix(mat);
}

Matrix::Matrix(int r, int c)
{
	n = r;
	m = c;
	for(int i=0; i < r; i++)
	{
		for(int j=0; j < c; j++)
		{
			if(i == j)
				data.push_back(1.0);
			else
				data.push_back(0.0);
		}
	}
}

Matrix::~Matrix()
{}

string Matrix::as_string()
{
	string s = "";
	for(int i=0; i < (m * n); i++)
	{
		if(i % m == 0)
			s += "\n";
		s += to_string(data[i]) + ' ';
	}
	return s;
}

int Matrix::index2addr(int i, int j)
{
	if(j >= columns() || i >= rows())
		return -1;
	return (i * columns()) + j;
}

int Matrix::columns()
{
	return m;
}

int Matrix::rows()
{
	return n;
}

void Matrix::reshape(int r, int c)
{
	int old_ind;

	vector<float> new_data;

	for(int i=0; i < r; i++)
	{
		for(int j=0; j < c; j++)
		{
			old_ind = index2addr(i, j);
			if(old_ind >= 0)
				new_data.push_back(data[old_ind]);
			else
				new_data.push_back(0.0);
		}
	}

	n = r;
	m = c;
	data = new_data;
}

void Matrix::T()
{
	int temp = n;
	int new_index;
	int old_index;

	vector<float> new_data;

	for(int i=0; i < n * m; i++)
		new_data.push_back(data[((i % n) * m) + int(i / n)]);
	
	n = m;
	m = temp;
	data.assign(new_data.begin(), new_data.end());
}

void Matrix::matmul(Matrix* m2)
{
	assert(m2->columns() == n);

	vector<vector<float>> mat;
	vector<float> vect;
	float acc = 0.0;


	for(int i=0; i < m2->rows(); i++)
	{
		vect.clear();
		for(int j=0; j < m; j++)
		{
			acc = 0.0;
			for(int k=0; k < m2->columns(); k++)
			{
				acc += m2->get(i, k) * data[index2addr(k, j)];
			}
			vect.push_back(acc);
		}
		mat.push_back(vect);
	}
	set_matrix(mat);
}

void Matrix::clear()
{
	fill(data.begin(), data.end(), 0.0);
}

void Matrix::set_matrix(vector<vector<float>> vect)
{
	n = vect.size();
	m = vect[0].size();

	vector<float> new_data;

	for(vector<float> r: vect)
	{
		assert(m == r.size());
		for(float x: r)
		{
			new_data.push_back(x);
		}
	}
	data.assign(new_data.begin(), new_data.end());
}

void Matrix::set(int i, float num)
{
	assert(i < n * m && i >= 0);
	data[i] = num;
}

void Matrix::set(int i, int j, float num)
{
	int addr = index2addr(i, j);
	set(addr, num);
}

void Matrix::set(int i, vector<int> j, vector<float> nums)
{	
	for(int k=0; k < j.size(); k++)
		set(i, j[k], nums[k]);
}

void Matrix::set(vector<int> i, int j, vector<float> nums)
{
	for(int k=0; k < i.size(); k++)
		set(i[k], j, nums[k]);
}

void Matrix::set(vector<int> i, vector<int> j, vector<vector<float>> nums)
{
	for(int k=0; k < i.size(); k++)
		set(i[k], j, nums[k]);
}

float Matrix::get(int i)
{
	assert(i < data.size() && i >= 0);
	return data[i];
}

float Matrix::get(int i, int j)
{
	int addr = index2addr(i, j);
	assert(addr < data.size() && addr >= 0);
	return data[addr];
}

vector<float> Matrix::get(int i, vector<int> j)
{
	vector<float> vect;
	for(int k=0; k < j.size(); k++)
		vect.push_back(get(i, j[k]));
	return vect;
}

vector<float> Matrix::get(vector<int> i, int j)
{
	vector<float> vect;
	for(int k=0; k < i.size(); k++)
		vect.push_back(get(i[k], j));
	return vect;
}

void Matrix::get(vector<int> i, vector<int> j, Matrix* m1)
{
	vector<vector<float>> vect;
	for(int k=0; k < i.size(); k++)
		vect.push_back(get(i[k], j));

	m1->set_matrix(vect);
}
