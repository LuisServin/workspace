#include <iostream>

using namespace std;

void af(int& g) {
	g++;
	cout << g << '\n';
}

int main() 
{
	int g = 123;
	cout << g << '\n';
	af(g);
	cout << g << '\n';
	return 0;
}