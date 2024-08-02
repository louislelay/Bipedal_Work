#!/usr/bin/env python3

def abs_bounded_derivative_filter(
	prev_output: float,
	new_input: float,
	dt: float,
	max_output: float,
	max_derivative: float,
) -> float:

	return bounded_derivative_filter(
		prev_output,
		new_input,
		dt,
		(-max_output, max_output),
		(-max_derivative, max_derivative),
	)


def bounded_derivative_filter(
	prev_output: float,
	new_input: float,
	dt: float,
	output_bounds: (float, float),
	derivative_bounds: (float, float),
) -> float:

	derivative = (new_input - prev_output) / dt
	bounded_derivative = min(derivative_bounds[1], max(derivative, derivative_bounds[0]))

	output = prev_output + derivative * dt
	bounded_output = min(output_bounds[1], max(output, output_bounds[0]))

	return bounded_output


def low_pass_filter(
	prev_output: float,
	cutoff_period: float,
	new_input: float,
	dt: float,
) -> float:

	alpha = dt / cutoff_period
	assert alpha < 0.5
	return prev_output + alpha * (new_input - prev_output)