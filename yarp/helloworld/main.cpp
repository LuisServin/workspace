#include <stdio.h>
#include "world.h"

int main() {
	printf("hello %s.\n", getWorld().c_str());
	return 0;
}