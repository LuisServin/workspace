#!/usr/bin/env python

import random

# the neigbor moves are orden in.
# right, down, left, up
# the matrix first index is 0 not 1
neighbor = [[1, 3, 0, 0], [2, 4, 0, 1], [2, 5, 1, 2],
			[4, 6, 3, 0], [5, 7, 3, 1], [5, 8, 4, 2],
			[7, 6, 6, 3], [8, 7, 6, 4], [8, 8, 7, 5]]

t_max = 12
# starting site for the algorithm
site = 8
# current move
t = 0
print site
while t < t_max:
	t += 1
	# with the random function we choose a move from
	# the possible four options
	site = neighbor[site][random.randint(0, 3)]
	print site