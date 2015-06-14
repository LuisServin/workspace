#include <iostream>
#include <new>

using namespace std;

int main ()
{
	int i, n;
	int * p;
	cout << "How many numbers would you like to type? ";
	cin >> i;
	// Create a array dynamically 'cause it depends
	// from a value specified by the user and can't be
	// set until that moment
	// to avoid exception we use (nonthrow) with new 
	// if the allocation fails p = nullptr
	p = new (nothrow) int[i];
	// check if the memory was allocated correctly by comparing
	// to a nullptr pointer
	if (p == nullptr) {
		cout << "Error: memory could not be allocated";
	} else {
		for (int n = 0; n < i; n++) {
			cout << "Enter number: ";
			cin >> p[n];
		}
		cout << "You have entered: ";
		for (int n = 0; n < i; n++) {
			cout << p[n] << ", ";
		}
		cout << '\n';
		delete[] p;
	}
	return 0;
}