#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Keep the robot up using its wheels."""

from typing import Tuple

import gin
import numpy as np
from numpy.typing import NDArray

class WheelController:
	"""
	Balancing by proportional-derivative feedback of the body pitch error to
	wheel accelerations:

		body pitch error --(PD)--> wheel accelerations

	"""

	class Gains:
		"""Gains for the wheel controller."""

		pitch_damping: float
		pitch_stiffness: float
		position_damping: float
		position_stiffness: float

		def __init__(
			self,
			pitch_damping: float,
			pitch_stiffness: float,
			position_damping: float,
			position_stiffness: float,
		):
		
			"""Initialize gains."""
			self.pitch_damping = pitch_damping
			self.pitch_stiffness = pitch_stiffness
			self.position_damping = position_damping
			self.position_stiffness = position_stiffness

		def set(
			self,
			pitch_damping: float,
			pitch_stiffness: float,
			position_damping: float,
			position_stiffness: float,
		) -> None:

			self.pitch_damping = pitch_damping
			self.pitch_stiffness = pitch_stiffness
			self.position_damping = position_damping
			self.position_stiffness = position_stiffness

		def __repr__(self):
			"""!
			Represent gains as a readable string.
			"""
			return (
				"WheelController.Gains("
				f"pitch_stiffness={self.pitch_stiffness}, "
				f"pitch_damping={self.pitch_damping}, "
				f"position_stiffness={self.position_stiffness}, "
				f"position_damping={self.position_damping})"
			)

	air_return_period: float				# Cutoff period for resetting integrators while the robot is in the air in [s].
	error: NDArray[float]					# Two-dimensional vector of ground position and base pitch errors.
	fall_pitch: float						# Fall pitch angle, in radians.
	gains: Gains							# Velocity controller gains.
	ground_velocity: float					# Sagittal velocity in [m] / [s].
	integral_error_velocity: float			# Integral term contributing to the sagittal velocity, in [m] / [s].
	max_ground_velocity: float				# Maximum commanded ground velocity no matter what, in [m] / [s].
	max_integral_error_velocity: float		# Maximum integral error velocity, in [m] / [s].
	max_target_accel: float					# Maximum acceleration for the ground target, in [m] / [s]². Does not affect the commanded ground velocity.
	max_target_distance: float				# Maximum distance from the current ground position to the target, in [m].
	max_target_velocity: float				# Maximum velocity for the ground target, in [m] / [s]. Indirectly affects the commanded ground velocity.
	max_yaw_accel: float					# Maximum yaw angular acceleration in [rad] / [s]².
	max_yaw_velocity: float					# Maximum yaw angular velocity in [rad] / [s].
	pitch: float							# Current IMU pitch angle in [rad].
	target_ground_position: float			# Target ground sagittal position in [m].
	target_ground_velocity: float			# Target ground sagittal velocity in [m] / [s].
	target_yaw_position: float				# Target yaw position in [rad].
	target_yaw_velocity: float				# Target yaw velocity in [rad] / [s].
	turning_deadband: float					# Joystick axis value between 0.0 and 1.0 below which legs stiffen but the turning motion doesn't start.
	turning_decision_time: float			# Probability that the user wants to turn based on the joystick axis value.
	turning_probability: float				# Minimum duration in [s] for the turning probability to switch from zero to one and converesly.
	wheel_radius: float						# Wheel radius in [m].

	def __init__(
		self,
		air_return_period: float,
		fall_pitch: float,
		max_ground_velocity: float,
		max_integral_error_velocity: float,
		max_target_accel: float,
		max_target_distance: float,
		max_target_velocity: float,
		max_yaw_accel: float,
		max_yaw_velocity: float,
		turning_deadband: float,
		turning_decision_time: float,
		wheel_radius: float,
	):
		r"""!
		Initialize balancer.
		"""
		assert 0.0 <= turning_deadband <= 1.0
		self.air_return_period = air_return_period
		self.error = np.zeros(2)
		self.fall_pitch = fall_pitch
		self.gains = WheelController.Gains()  # type: ignore
		self.ground_velocity = 0.0
		self.integral_error_velocity = 0.0
		self.max_ground_velocity = max_ground_velocity
		self.max_integral_error_velocity = max_integral_error_velocity
		self.max_target_accel = max_target_accel
		self.max_target_distance = max_target_distance
		self.max_target_velocity = max_target_velocity
		self.max_yaw_accel = max_yaw_accel
		self.max_yaw_velocity = max_yaw_velocity
		self.pitch = 0.0
		self.target_ground_position = 0.0
		self.target_ground_velocity = 0.0
		self.target_yaw_position = 0.0
		self.target_yaw_velocity = 0.0
		self.turning_deadband = turning_deadband
		self.turning_decision_time = turning_decision_time
		self.turning_probability = 0.0
		self.wheel_radius = wheel_radius


	def cycle(self, observation: dict, dt: float) -> None:
		r"""!
		Compute a new ground velocity.
		"""

		pitch = get_pitch()
		
		self.pitch = pitch
		if abs(pitch) > self.fall_pitch:
			self.integral_error_velocity = 0.0  # [m] / [s]
			self.ground_velocity = 0.0  # [m] / [s]
			return

		ground_position = observation["wheel_odometry"]["position"]
		floor_contact = observation["floor_contact"]["contact"]

		target_pitch: float = 0.0  # [rad]
		error = np.array(
			[
				self.target_ground_position - ground_position,
				target_pitch - pitch,
			]
		)
		self.error = error

		if not floor_contact:
			self.integral_error_velocity = low_pass_filter(
				self.integral_error_velocity, self.air_return_period, 0.0, dt
			)
			# We don't reset self.target_ground_velocity: either takeoff
			# detection is a false positive and we should resume close to the
			# pre-takeoff state, or the robot is really in the air and the user
			# should stop smashing the joystick like a bittern ;p
			self.target_ground_position = low_pass_filter(
				self.target_ground_position,
				self.air_return_period,
				ground_position,
				dt,
			)
		else:  # floor_contact:
			ki = np.array(
				[
					self.gains.position_stiffness,
					self.gains.pitch_stiffness,
				]
			)
			self.integral_error_velocity += ki.dot(error) * dt
			self.integral_error_velocity = clamp_abs(
				self.integral_error_velocity, self.max_integral_error_velocity
			)
			self.target_ground_position += self.target_ground_velocity * dt
			self.target_ground_position = clamp(
				self.target_ground_position,
				ground_position - self.max_target_distance,
				ground_position + self.max_target_distance,
			)

		kp = np.array(
			[
				self.gains.position_damping,
				self.gains.pitch_damping,
			]
		)

		upkie_trick_velocity = -self.target_ground_velocity

		self.ground_velocity = (
			upkie_trick_velocity - kp.dot(error) - self.integral_error_velocity
		)
		self.ground_velocity = clamp_abs(
			self.ground_velocity, self.max_ground_velocity
		)

	def get_wheel_velocities(
		self,
		position_right_in_left: NDArray[float],
	) -> Tuple[float, float]:
		r"""!
		Get left and right wheel velocities.
		"""
		# Sagittal translation
		left_wheel_velocity: float = +self.ground_velocity / self.wheel_radius
		right_wheel_velocity: float = -self.ground_velocity / self.wheel_radius

		# Yaw rotation
		contact_radius = 0.5 * np.linalg.norm(position_right_in_left)
		yaw_to_wheel = contact_radius / self.wheel_radius
		left_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity
		right_wheel_velocity += yaw_to_wheel * self.target_yaw_velocity

		return left_wheel_velocity, right_wheel_velocity