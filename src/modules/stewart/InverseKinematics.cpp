/**
 * @file InverseKinematics.cpp
 */

#include "InverseKinematics.hpp"

#include <mathlib/math/Functions.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

#include <px4_platform_common/module_params.h>


using namespace matrix;


matrix::Vector2f InverseKinematics::stewartBalanceCtrl(const matrix::Quatf &q){

	const matrix::Eulerf euler{q};

	const float roll_ctrl   = math::constrain(euler.phi(),-math::radians(13.0f), math::radians(13.0f));
	const float pitch_ctrl  = math::constrain(euler.theta(),-math::radians(13.0f), math::radians(13.0f));


	const float n_x_des = tanf(pitch_ctrl);
	const float n_y_des = tanf(roll_ctrl) / cosf(pitch_ctrl);

	return matrix::Vector2f{n_x_des, n_y_des};
}

matrix::Vector2f InverseKinematics::angleToNormVectorConv(float roll_des, float pitch_des){

	// Convert roll and pitch angles to radians
	float roll_rad = math::radians(roll_des);
	float pitch_rad = math::radians(pitch_des);

	const float n_x_des = tanf(pitch_rad);
	const float n_y_des = -tanf(roll_rad) / cosf(pitch_rad);

	return matrix::Vector2f{n_x_des, n_y_des};
}

matrix::Vector3f InverseKinematics::computeServoAngles(float n_x, float n_y, float h_z)
{
	float n_mag = sqrtf(n_x * n_x + n_y * n_y + 1);  //magnitude of the normal vector
	n_x = n_x / n_mag;
	n_y = n_y / n_mag;
	float n_z = 1.0f / n_mag;

	float f = 230.0f; //mm
	float g = 560.0f; //mm
	float e = 450.0f; //mm
	float d = 680.0f; //mm

	/*Calculate the servo angles based on the normalized vector*/
	/*=========================================================================================================================*/
	//LEG A
	float z_A = h_z + n_x * e;

	float x_A = d + (e / 2.0f) *
	(
		1 - ((n_y *n_y + 3.0f  * n_z *n_z + 3.0f  * n_z) / (1.0f  + n_z - n_y *n_y))

		+ (n_y *n_y *n_y *n_y - 3.0f  *n_x *n_x *n_y *n_y) / ((1.0f  +n_z)*(1.0f  + n_z - n_y *n_y))
	);

	float mag_A = sqrtf(x_A * x_A + z_A * z_A);

	const float cos_beta_leg_A = math::constrain(x_A / mag_A, MIN_ACOS, MAX_ACOS);

	const float cos_gama_leg_A = math::constrain((mag_A * mag_A + f * f - g * g) / (2.0f * mag_A * f), MIN_ACOS, MAX_ACOS);

	float ang_LegA = acosf(-cos_beta_leg_A) + acosf(cos_gama_leg_A);
	/*=========================================================================================================================*/
	//LEG B
	float z_B = h_z - (e / 2.0f) * (n_x + SQRT3 * n_y);

	float y_B = SQRT3_DIV_2 *
	(
		e * (1 -(n_y * n_y + SQRT3 * n_x *n_y) / (1.0f + n_z)) - d
	);

	float x_B = y_B * ONE_DIV_SQRT3;

	float mag_B = sqrtf(x_B * x_B + y_B * y_B + z_B * z_B);

	const float cos_beta_leg_B = math::constrain((x_B + SQRT3 * y_B) / (-2.0f * mag_B), MIN_ACOS, MAX_ACOS);

	const float cos_gama_leg_B = math::constrain((mag_B * mag_B + f * f - g * g) / (2.0f * mag_B * f), MIN_ACOS, MAX_ACOS);

	float ang_LegB = acosf(-cos_beta_leg_B) + acosf(cos_gama_leg_B);
	/*=========================================================================================================================*/
	//LEG C
	float z_C = h_z - (e / 2.0f) * (n_x - SQRT3 * n_y);

	float y_C = SQRT3_DIV_2 *
	(
		d - e * (1 -(n_y * n_y - SQRT3 * n_x *n_y) / (1.0f + n_z))
	);

	float x_C = -y_C * ONE_DIV_SQRT3;

	float mag_C = sqrtf(x_C * x_C + y_C * y_C + z_C * z_C);

	const float cos_beta_leg_C = math::constrain((-x_C + SQRT3 * y_C) / (2.0f * mag_C), MIN_ACOS, MAX_ACOS);

	const float cos_gama_leg_C = math::constrain((mag_C * mag_C + f * f - g * g) / (2.0f * mag_C * f), MIN_ACOS, MAX_ACOS);

	float ang_LegC = acosf(-cos_beta_leg_C) + acosf(cos_gama_leg_C);
	/*=========================================================================================================================*/
	ang_LegA = _max_angle - math::degrees(ang_LegA);
	ang_LegB = _max_angle - math::degrees(ang_LegB);
	ang_LegC = _max_angle - math::degrees(ang_LegC);

	Vector3f angCtrl = {ang_LegA, ang_LegB, ang_LegC};

	return angCtrl;
}
