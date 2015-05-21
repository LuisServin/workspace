#include <iostream>
using namespace std;

void increment_all (int* start, int* stop) {
	int * current = start;
	while (current != stop) {
		// increment value pointed
		++(*current);
		// increment pointer
		++current;
	}
}

void print_all (const int* start, const int* stop) {
	// in this case current is a pointer of type const int*
	// current can't modify the value it points to but
	// the pointer itself can change
	const int* current = start;
	while (current != stop) {
		cout << *current << '\n';
		// increment pointer
		++current;
	}
}

int main() 
{
	int numbers[] = {10, 20, 30};
	increment_all(numbers, numbers + 3);
	print_all(numbers, numbers + 3);
	return 0;
}