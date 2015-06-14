#include <iostream>

using namespace std;

void swap(int& x, int& y) {
	int temp;
	temp = x;
	x = y;
	y = temp;
}

int main()
{
	int a = 100;
	int b = 200;

	cout << "Before swap value of a: " << a << '\n';
	cout << "Before swap value of b: " << b << '\n';

	swap(a, b);
	cout << "After swap value of a: " << a << '\n';
	cout << "After swap value of b: " << b << '\n';

	return 0;
}