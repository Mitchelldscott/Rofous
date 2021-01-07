#include <cmath>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <ostream>
#include <Matrix.hpp>
#define Pi 3.1415926535

using namespace std;

Matrix::Matrix()
{
	n = 3;
	m = 3;
	int ctr = 0;
	for(int i=0; i<2; i++)
	{
		data.push_back(1.0);
		for(int j=0; j<3; j++)
		{
			data.push_back(0.0);
		}
	}
	data.push_back(1.0);
}

Matrix::Matrix(vector<vector<float>> mat)
{
	set_matrix(mat);
}

Matrix::~Matrix()
{
}

string Matrix::matrix_as_string()
{
	string s = "";
	for(int i=0; i < (m * n); i++)
	{
		s += to_string(get(i)) + ' ';
		if((i + 1) % m == 0)
			s += "\n";
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
	int addr;
	int prev_n = n;
	int prev_m = m;
	int old_ind = -1;
	n = r;
	m = c;
	vector<float> new_data(r * c, 0.0);

	for(int i=0; i < prev_n; i++)
	{
		for(int j=0; j < prev_m; j++)
		{
			addr = index2addr(i, j);
			old_ind = (i * prev_m) + j;
			if(addr < n * m && addr >= 0)
				new_data[addr] = data[old_ind];
		}
	}

	data = new_data;
}

int Matrix::T()
{
	int new_index = 0;
	int old_index = 0;
	int l = data.size();

	vector<float> new_data(l, 0.0);

	for(int i=0; i < n; i++)
	{
		for(int j=0; j < m; j++)
		{
			old_index = index2addr(i, j);
			new_index = (j * rows()) + i;
			new_data[new_index] = data[old_index];
		}
	}
	int temp = n;
	n = m;
	m = temp;
	data = new_data;
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
	data = {};
	n = vect.size();
	m = vect[0].size();
	for(vector<float> r: vect)
	{
		assert(m == r.size());
		for(float x: r)
		{
			data.push_back(x);
		}
	}
}

int Matrix::set(int i, float num)
{
	if(i < n * m && i >= 0)
		data[i] = num;
	else
		return 0;
	return 1;
}

int Matrix::set(int i, int j, float num)
{
	int addr = index2addr(i, j);
	return set(addr, num);
}

int Matrix::set(int i, vector<int> j, vector<float> nums)
{	
	int e = 1;
	for(int k=0; k < j.size(); k++)
		e *= set(i, j[k], nums[k]);

	return e;
}

int Matrix::set(vector<int> i, int j, vector<float> nums)
{
	int e = 1;
	for(int k=0; k < i.size(); k++)
		e *= set(i[k], j, nums[k]);

	return e;
}

int Matrix::set(vector<int> i, vector<int> j, vector<vector<float>> nums)
{
	int e = 1;
	for(int k=0; k < i.size(); k++)
	{
		e *= set(i[k], j, nums[k]);
	}

	return e;
}

float Matrix::get(int i)
{
	if(i >= data.size() || i < 0)
		return Pi;
	return data[i];
}

float Matrix::get(int i, int j)
{
	int addr = index2addr(i, j);
	if(addr >= data.size() || addr < 0)
		return Pi;
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
