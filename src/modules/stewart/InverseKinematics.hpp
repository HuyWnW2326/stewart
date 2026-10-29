
#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_attitude.h>

class InverseKinematics
{
public:
	InverseKinematics() = default;
	~InverseKinematics()= default;

	/**
	 * Compute the servo angles based on the normalized vector and height
	 * @param n_x
	 * @param n_y
	 * @param h_z
	 * @return The servo angles in degree for the three legs of the Stewart platform
	 */
	matrix::Vector3f computeServoAngles( float n_x,  float n_y,  float h_z);

	matrix::Vector2f angleToNormVectorConv( float roll_des,  float pitch_des);

	matrix::Vector2f stewartBalanceCtrl(const matrix::Quatf &q);

	void setMaxAng(const float max_angle) { _max_angle = max_angle;};
private:

	static constexpr float SQRT3_DIV_2 = 0.8660254037844386f;
	static constexpr float SQRT3       = 1.7320508075688772f;
	static constexpr float ONE_DIV_SQRT3 = 0.5773502691896257f;
	static constexpr float MAX_ACOS =  1.0f;
	static constexpr float MIN_ACOS = -1.0f;
	float _max_angle = 0;
};
