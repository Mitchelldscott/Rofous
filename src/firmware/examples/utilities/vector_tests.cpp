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

// void from_array_fill_test(Vector<float> data, int target_length) {
// 	float test_data[target_length];

// 	for (int i = 0; i < target_length; i++) {
// 		test_data[i] = i;
// 	}
// 	Serial.println("From array vector");
// 	Serial.print("Pre-Fill    \t"); data.print();
// 	data.from_array(test_data, target_length);
// 	Serial.print("Post-Fill    \t"); data.print();
// 	data.clear();
// 	Serial.print("Post-Clear\t"); data.print();
// }

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
		return errors;
	}

	errors += recursive_test(v1, size, depth - 1);

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
	while(!Serial){}

	Serial.println("=== Starting Vector tests ===");

	int n = 5;
	int total_errors = 0;
	total_errors += index_operator_test(n);

	Vector<float> v = incremental_vector_fill(n);
	total_errors += assign_operator_test(Vector<float>(n), v);

	total_errors += reset_and_fill_test(n);

	Vector<float> v2(n);
	total_errors += append_test(v2);

	total_errors += iterative_test(100, 100);

	Vector<float> data(0);
	total_errors += recursive_test(&data, 10, 100);

	Serial.printf("=== Finished Vector tests with %i errors ===\n", total_errors);
}