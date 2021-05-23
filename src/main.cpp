
#include <iostream>
#include "U_Test.hpp"
#define name "[EXE-MANAGER]"

using namespace std;


int main(int argc, char *argv[])
{
	int e_ctr = 0;

	if(argc == 1)
		cout << name << " Usage: no jobs specified" << endl;

	for(int i = 1; i < argc; i++)
	{
		if(argv[i] == std::string("U-Test"))
			run_tests();
	}
	
	cout << name << " Finished All Jobs with " << e_ctr << " errors" << endl;
	return 0;
}