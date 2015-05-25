#!/usr/bin/env python

# MonteCarlo heliport algorithm
import random

# set the beginning position
x, y = 1.0, 1.0
# maximun delta for the trial
delta = 0.1
n_trials = 4000
n_hints = 0
for i in range(n_trials):
	# calculate the new delta for the current trial
	del_x, del_y = random.uniform(-delta, delta), random.uniform(-delta, delta)
	# check if we are still inside the square
	if abs(x + del_x) < 1.0 and abs(y + del_y) < 1.0:
		# if true move to the new position
		x, y = x + del_x, y + del_y
	# in the new position check if we are inside the circle
	if x**2 + y**2 < 1.0:
		n_hints += 1
print 4.0 * n_hints / float(n_trials)