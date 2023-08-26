/********************************************************************************
 * 
 *      ____                     ____          __           __       _          
 *	   / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *	  / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *	 / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  ) 
 *	/_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/  
 *	      /____/                                                                
 * 
 * 
 * 
 ********************************************************************************/

#include <Arduino.h>
#include "utilities/blink.h"
#include "utilities/splash.h"
#include "utilities/vector.h"
#include "utilities/assertions.h"


Vector<float> incremental_vector_fill(int target_length) {
	Vector<float> data(target_length);

	for (int i = 0; i < target_length; i++) {
		data[i] = i + 1;
	}

	return data;
}

int index_operator_test(int target_length) {
	int errors = 0;

	Vector<float> data = incremental_vector_fill(target_length);

	for (int i = 0; i < target_length; i++) {
		errors += assert_eq<float>(data[i], float(i+1), "Vector operator test [" + String(i) + "]");
	}

	data.clear();
	errors += assert_eq<float>(data.as_array(), 0.0f, "Vector fill and Clear test", target_length);

	return errors;
}

int assign_operator_test(Vector<float> v1, Vector<float> v2) {
	int errors = 0;

	v1 = v2;
	errors += assert_eq<float>(v1.as_array(), v2.as_array(), "Vector Assign test", v1.size());

	v2.reset(0);
	errors += assert_neq<float>(v1.as_array(), 0.0f, "Vector Assign and No Clear test", v1.size());
	errors += assert_eq<float>(v2.as_array(), 0.0f, "Vector Assign and Clear test", v2.size());
	return errors;
}

int reset_and_fill_test(int target_length) {
	Vector<float> data = incremental_vector_fill(target_length);

	data.reset(target_length);
	return assert_eq<float>(data.as_array(), 0.0f, "Vector Reset test", target_length);
}

int append_test(Vector<float> v1) {
	int errors = 0;
	int og_length = v1.size();
	float slice1[og_length];
	float slice2[og_length];

	Vector<float> v2 = incremental_vector_fill(v1.size());

	v1.append(&v2);

	for (int i = 0; i < og_length; i++) {
		assert_eq<float>(v1[i], 0.0f, "Vector append test [" + String(i) + "]");
	}
	for (int i = 0; i < v2.size(); i++) {
		assert_eq<float>(v1[i+og_length], v2[i], "Vector append test [" + String(i) + "]");
	}

	v1.slice(slice1, 0, og_length);
	v1.slice(slice2, og_length, v2.size());
	errors += assert_eq<float>(slice1, 0.0f, "Vector append and slice1 test", og_length);
	errors += assert_eq<float>(v2.as_array(), slice2, "Vector append and slice2 test", v2.size());

	v1.clear();
	errors += assert_eq<float>(v1.as_array(), 0.0f, "Vector Clear test", v1.size());
	errors += assert_neq<float>(v2.as_array(), 0.0f, "Vector Append and No Clear test", v2.size());

	return errors;
}

int iterative_test(int size, int depth) {
	int errors = 0;

	Vector<float> v1(size);
	for (int i = 0; i < depth; i++) {
		Vector<float> v2(size);
		v1 = v2;
		errors += assert_eq<float>(v1.as_array(), v2.as_array(), "Vector iterative assign test", size);
	}

	return errors;
}

int recursive_test(Vector<float>* v1, int size, int depth) {
	int errors = 0;

	Vector<float> v2(size);
	errors += assert_eq<float>(v2.as_array(), 0.0f, "Vector recursion init test [" + String(depth) + "]", size);

	for (int i = 0; i < size; i++) {
		v2[i] = float(depth);
	}

	*v1 = v2;
	errors += assert_eq<float>(v1->as_array(), float(depth), "Vector recursion assign test [" + String(depth) + "]", size);

	if (depth <= 0) {
		for (int i = 0; i < size; i++) {
			(*v1)[i] = -1;
		}
		return errors;
	}

	errors += recursive_test(v1, size, depth - 1);

	errors += assert_eq<float>(v1->as_array(), -1, "Vector recursion return check [" + String(depth) + "]", size);

	return errors;
}


int assign_from_pointer(int size) {
	int errors = 0;
	Vector<float>* v1 = new Vector<float>(size);
	Vector<float> v2 = incremental_vector_fill(size);

	v2 = v1;

	for (int i = 0; i < size; i++) {
		(*v1)[i] = float(size);
	}

	for (int i = 0; i < size; i++) {
		v2[i] = float(size) / 2;
	}

	errors += assert_eq<float>(v1->as_array(), float(size), "Pointer assign original modified check", size);
	errors += assert_eq<float>(v2.as_array(), float(size)/2, "Pointer assign assignee check", size);
	delete v1;
	
	v2.clear();
	errors += assert_eq<float>(v2.as_array(), 0.0, "Pointer assign assignee check", size);

	return errors;
}


int pop_test(int size) {
	int errors = 0;
	Vector<float> v = incremental_vector_fill(size);

	for (int i = 0; i < size + 3; i++) {
		float f = v.pop();
		errors += assert_eq<float>(f, i+1, "Vector pop failed");
		errors += assert_eq<float>(v.size(), size - i - 1, "Vector pop failed");
	}

	return errors;
}


Vector<float> test_return() {

	float tmp[9] = {1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 1.0, 1.0, 1.0};
	Vector<float> v(9);
	v.from_array(tmp, 9);
	return v;
}

int main() {
	setup_blink();
	
	unit_test_splash("Vector", -1);

	int n = 5;
	int total_errors = 0;
	// tests index operator, returning vector from function,
	// clearing vector and getting buffer as array
	total_errors += index_operator_test(n);

	// initialize and return incremental vector
	Vector<float> v = incremental_vector_fill(n);
	// Tests assigning and getting buffer as array
	// also clearing and getting buffer as array
	total_errors += assign_operator_test(Vector<float>(n), v);

	// test resetting vector
	total_errors += reset_and_fill_test(n);

	Vector<float> v2(n);
	// test as arg, appending to
	// slicing and clearing vector
	total_errors += append_test(v2);

	// test creating and assigning vector n times
	total_errors += iterative_test(100, 100);

	Vector<float> data(0);
	// test ptr as arg, index assignment,
	// recursively initializing and assigning,
	total_errors += recursive_test(&data, 10, 100);

	total_errors += assign_from_pointer(10);

	total_errors += pop_test(100);

	Serial.printf("=== Finished Vector tests with %i errors ===\n", total_errors);
}