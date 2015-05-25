#!/usr/bin/env python

import random

n_trials = 4000
n_hits = 0
for iter in range(n_trials):
	# here me make a random sampling for x and y
	x, y = random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)
	# check if x and y is inside the circle of radius 1 
	if x**2 + y**2 < 1.0:
		# if true, aument hits by 1
		n_hits += 1
# the relation between the area of circle and square
# is Pi / 4
print 4.0 * n_hits / float(n_trials)