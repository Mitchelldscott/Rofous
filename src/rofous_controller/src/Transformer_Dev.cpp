#include <vector>
#include <stdio.h>
#include <iostream>
#include <ostream>
#include <chrono>
#include <Transformer.hpp>
#define Pi 3.14159

using namespace std;

void matrix_tests()
{

	vector<vector<float>> vect = {{1,2,3,1},{4,5,6,1},{7,8,9,1},{0,0,0,1}};
	Matrix m1(vect);
	Matrix m2(vect);

	auto start = chrono::high_resolution_clock::now();
	int e1 = m1.set(0, 1);
	int e2 = m2.set(5, 5);
	auto stop = chrono::high_resolution_clock::now();
	chrono::duration<double> set_time = stop - start;

	start = chrono::high_resolution_clock::now();
	float num1 = m1.get(0);
	float num2 = m2.get(5);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> get_time = stop - start;

	vector<vector<int>> slice1 = {{1,2,3}, {1,2,3}};
	vector<vector<int>> slice2 = {{0,1,2}, {0,1,2}};
	start = chrono::high_resolution_clock::now();
	float num1a = m1.get(0,4);
	float num2a = m2.get(1,2);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> slice_time = stop - start;
	
	start = chrono::high_resolution_clock::now();
	vector<float> nums1 = m1.get(1,slice1[0]);
	vector<float> nums2 = m2.get(slice2[0],2);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> slice0_time = stop - start;

	Matrix m1_slice;
	Matrix m2_slice;
	start = chrono::high_resolution_clock::now();
	m1.get(slice1[0],slice1[0], &m1_slice);
	m2.get(slice2[0],slice2[1], &m2_slice);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> slice1_time = stop - start;

	cout << "----- Matrix testing begin -----" << endl;
	cout << "Initialized As:" << endl;
	cout << "Matrix 1:\n" + m1.matrix_as_string() << endl;
	cout << "Matrix 2:\n" + m2.matrix_as_string() << endl;
	cout << "Sliced Matrices:" << endl;
	cout << "Matrix 1 set0:\n" << e1 << endl;
	cout << "Matrix 2 set0:\n" << e2 << endl;
	cout << "Time: " << set_time.count() / 2 << endl;
	cout << "Matrix 1 get0:\n" << num1 << endl;
	cout << "Matrix 2 get0:\n" << num2 << endl;
	cout << "Time: " << get_time.count() / 2 << endl;
	cout << "Matrix 1 slice0:\n" << num1a << endl;
	cout << "Matrix 2 slice0:\n" << num2a << endl;
	cout << "Time: " << slice_time.count() / 2 << endl;
	cout << "Matrix 1 slice1:" << endl;
	for(int i=0; i < nums1.size(); i++)
		cout << nums1[i] << " ";
	cout << endl;
	cout << "Matrix 2 slice1:" << endl;
	for(int i=0; i < nums2.size(); i++)
		cout << nums2[i] << " ";
	cout << endl;
	cout << "Time: " << slice0_time.count() / 2 << endl;
	cout << "Matrix 1 slice2:\n" << m1_slice.matrix_as_string() << endl;
	cout << "Matrix 2 slice2:\n" << m2_slice.matrix_as_string() << endl;
	cout << "Time: " << slice1_time.count() / 2 << endl;

	start = chrono::high_resolution_clock::now();
	m1.T();
	m2.T();
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> transpose_time = stop - start;
	cout << "Matrix 1 Transpose:\n" << m1.matrix_as_string() << endl;
	cout << "Matrix 2 Transpose:\n" << m2.matrix_as_string() << endl;
	cout << "Time: " << transpose_time.count() / 2 << endl;

	start = chrono::high_resolution_clock::now();
	m2.matmul(&m1);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> mult_time = stop - start;
	cout << "MatMul0:\n" << m2.matrix_as_string() << endl;
	cout << "Time: " << mult_time.count() << endl;

	vector<vector<float>> vect1 = {{1, 2,3}};
	vector<vector<float>> vect2 = {{0,1,0},{1,0,0},{0,0,1}};
	Matrix m3(vect1);
	Matrix m4(vect2);
	start = chrono::high_resolution_clock::now();
	m3.T();
	m3.matmul(&m4);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> tr_mult_time = stop - start;
	cout << "MatMul1:\n" << m3.matrix_as_string() << endl;
	cout << "Time: " << tr_mult_time.count() << endl;
	cout << "----- Testing Complete -----" << endl;
}

void transformer_tests()
{
	cout << "----- Transformer test begin -----" << endl;
	vector<vector<float>> x = {{1.0,0.0,0.0}};
	vector<vector<float>> y = {{0.0,1.0,0.0}};
	vector<vector<float>> z = {{0.0,0.0,1.0}};

	vector<float> translation = {0.0, 0.0, 0.0};
	vector<float> reference = {(90.0 * Pi) / 180, 0.0, 0.0};
	vector<char> protocol = {'x', 'y', 'z'};
	string name = "World";
	Transformer tf(translation, reference, protocol, name);
	Matrix x_axis(x);
	Matrix y_axis(y);
	Matrix z_axis(z);

	auto start = chrono::high_resolution_clock::now();
	x_axis.T();
	y_axis.T();
	z_axis.T();
	tf.rotate(&x_axis, true);
	tf.rotate(&y_axis, true);
	tf.rotate(&z_axis, true);
	auto stop = chrono::high_resolution_clock::now();
	chrono::duration<double> rot_time = stop - start;
	cout << "0 Rotated X:\n" << x_axis.matrix_as_string() << endl;
	cout << "0 Rotated Y:\n" << y_axis.matrix_as_string() << endl;
	cout << "0 Rotated Z:\n" << z_axis.matrix_as_string() << endl;
	cout << "Time: " << rot_time.count() << endl;

	start = chrono::high_resolution_clock::now();
	x_axis.T();
	y_axis.T();
	z_axis.T();
	tf.transform(&x_axis, true);
	tf.transform(&y_axis, true);
	tf.transform(&z_axis, true);
	stop = chrono::high_resolution_clock::now();
	chrono::duration<double> tr_mult_time = stop - start;
	cout << "0 Transformed X:\n" << x_axis.matrix_as_string() << endl;
	cout << "0 Transformed Y:\n" << y_axis.matrix_as_string() << endl;
	cout << "0 Transformed Z:\n" << z_axis.matrix_as_string() << endl;
	cout << "Time: " << tr_mult_time.count() << endl;

	translation = {10.0, 0.0, 0.0};
	reference = {0.0, 0.0, (90.0 * Pi) / 180};
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
	cout << "1 Transformed X:\n" << x_axis.matrix_as_string() << endl;
	cout << "1 Transformed Y:\n" << y_axis.matrix_as_string() << endl;
	cout << "1 Transformed Z:\n" << z_axis.matrix_as_string() << endl;
	cout << "Time: " << tr_mult_time.count() << endl;
	cout << "----- Testing Complete -----" << endl;
}


int main(int argc, char** argv)
{
	
	matrix_tests();
	transformer_tests();
	return 0;
}