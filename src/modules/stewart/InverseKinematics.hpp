
#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>


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
private:

	static constexpr float SQRT3_DIV_2 = 0.8660254037844386f;
	static constexpr float SQRT3       = 1.7320508075688772f;
	static constexpr float ONE_DIV_SQRT3 = 0.5773502691896257f;
};
