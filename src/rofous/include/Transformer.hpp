/**
	Transformer class

	Author  : Mitchell Scott (misc4432@colorado.edu)
	Project : Rofous
	Version : 0.1
	Date    : January 31, 2020

**/
#include <Matrix.hpp>

using namespace std;

#ifndef __Transformer_H__
#define __Transformer_H__

class Transformer
{
	private:
		string name;
		Matrix R;
		Matrix tf;
		Matrix tfI;
	public:
		Transformer* parent;
		Transformer* child;
		Transformer(vector<float>, vector<float>, vector<char>, string);
		~Transformer();
		string get_name();
		void set_name(string);
		void Rx(float);
		void Ry(float);
		void Rz(float);
		void add_translation(vector<float>, Matrix*);
		void build_rotation(vector<float>, vector<char>);
		void build_transform(vector<float>, vector<float>, vector<char>);
		void rotate(Matrix*, bool);
		void transform(Matrix*, bool);
};

#endif