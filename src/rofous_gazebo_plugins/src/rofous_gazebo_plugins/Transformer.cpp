/**
	Transformer class

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
#include <Transformer.hpp>
#define Pi 3.1415926535

using namespace std;

Transformer::Transformer(vector<float> translation, vector<float> reference, vector<char> protocol, string label)
{
	parent = NULL;
	child = NULL;
	name = label;
	build_transform(translation, reference, protocol);
}

Transformer::~Transformer()
{

}

string Transformer::get_name()
{
	return name;
}

void Transformer::set_name(string label)
{
	name = label;
}

void Transformer::Rx(float phi)
{
	vector<vector<float>> rotation = {{1.0, 0.0, 0.0},{0.0, cos(phi), sin(phi)},{0.0, -sin(phi), cos(phi)}};
	Matrix r_mat(rotation);
	R.matmul(&r_mat);
}

void Transformer::Ry(float theta)
{
	vector<vector<float>> rotation = {{cos(theta), 0.0, -sin(theta)},{0.0, 1.0, 0.0},{sin(theta), 0.0, cos(theta)}};
	Matrix r_mat(rotation);
	R.matmul(&r_mat);
}

void Transformer::Rz(float psi)
{
	vector<vector<float>> rotation = {{cos(psi), sin(psi), 0.0},{-sin(psi), cos(psi), 0.0},{0.0, 0.0, 1.0}};
	Matrix r_mat(rotation);
	R.matmul(&r_mat);
}

void Transformer::build_rotation(vector<float> reference, vector<char> protocol)
{
	assert(reference.size() == R.rows());
	for(char step: protocol)
	{
		switch(step){
			case 'x':
				Rx(reference[0]);
				break;
			case 'y':
				Ry(reference[1]);
				break;
			case 'z':
				Rz(reference[2]);
				break;
		}
	}
}

void Transformer::add_translation(vector<float> translation, Matrix* tf)
{
	assert(translation.size() == R.rows());
	vector<float> vect;
	vector<int> js = {0,1,2};
	vector<vector<float>> mat;
	for(int i=0; i < R.rows(); i++)
	{
		vect = R.get(i, js);
		vect.push_back(translation[i]);
		mat.push_back(vect);
		vect.clear();
	}
	mat.push_back({0.0,0.0,0.0,1.0});
	tf->set_matrix(mat);
}

void Transformer::build_transform(vector<float> translation, vector<float> reference, vector<char> protocol)
{
	build_rotation(reference, protocol);
	add_translation(translation, &tf);
	R.T();
	add_translation(translation, &tfI);
	R.T();
}

void Transformer::rotate(Matrix* point, bool inverse)
{
	assert(point->rows() == 3);
	switch(inverse){
		case true:
			R.T();
			point->matmul(&R);
			R.T();
			if(parent)
				parent->rotate(point, true);
			break;
		case false:
			point->matmul(&R);
			if(child)
				child->rotate(point, false);
			break;
	}
}

void Transformer::transform(Matrix* point, bool inverse)
{
	if(point->rows() < 4)
	{
		point->reshape(4,1);
		point->set(3, 1.0);
	}
	switch(inverse){
		case true:
			point->matmul(&tfI);
			if(parent)
				parent->transform(point, true);
			break;
		case false:
			point->matmul(&tf);
			if(child)
				child->transform(point, false);
			break;
	}
	if(point->rows() == 4)
		point->reshape(3,1);
}