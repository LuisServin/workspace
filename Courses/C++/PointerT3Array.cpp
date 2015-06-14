#include <iostream>
using namespace std;

int main() 
{
	int numbers[5];
	int * p;

	// assing the array numbers to pointer p
	// pointer will point to the first element of
	// the array numbers
	p = numbers;
	// numbers[0] = 10
	*p = 10;
	// index = index + 1;
	p++;
	// numbers[1] = 20
	*p = 20;

	// p points to number[2]
	p = &numbers[2];
	// numbers[2] = 30
	*p = 30;
	// p points to numbers[3]
	p = numbers + 3;
	// numbers[3] = 40
	*p = 40;
	// p points the first element of numbers
	p = numbers;
	// fourth element of numbers equal to 50
	*(p+4) = 50;
	// print all the values in the array	
	for (int n = 0; n < 5; n++) {
		cout << numbers[n] << ", ";
	}
	cout << '\n';

	return 0;
}