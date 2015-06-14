#!/usr/bin/env python

import random

def markov_pi(N, delta):
	n_hints = 0	
	x, y = 1.0, 1.0
	for i in range(n_trials):
		del_x, del_y = random.uniform(-delta, delta), random.uniform(-delta, delta)
		if abs(x + del_x) < 1.0 and abs(y + del_y) < 1.0:
			x, y = x + del_x, y + del_y
		if x**2 + y**2 < 1.0:
			n_hints += 1
	return n_hints

n_runs = 100
n_trials = 4000
delta = 0.1
for run in range(n_runs):
	print 4.0 * markov_pi(n_trials, delta) / float(n_trials)