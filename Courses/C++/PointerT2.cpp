#include <iostream>
using namespace std;

int main()
{ 
	int firstvalue, secondvalue;
	// Declare the pointers
	int * p1, * p2;

	// p1 points to firstvalue variable
	p1 = &firstvalue;
	// p2 points to firstvalue variable
	p2 = &secondvalue;
	// firstvalue = 10
	*p1 = 10;
	// secondvalue = firstvalue
	*p2 = *p1;
	// Assign the address of p2 to p1
	p1 = p2;
	// secondvalue = 20
	*p1 = 20;

	cout << "firstvalue is " << firstvalue << '\n';
	cout << "secondvalue is " << secondvalue << '\n';

	return 0;
}