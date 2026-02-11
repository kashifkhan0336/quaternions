#include <iostream>
#include <map>
#include <ranges>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <unordered_map>
#include <catch2/catch_approx.hpp> // Required header for Approx
#include <random>
#include "quaternion.hpp"


TEST_CASE("Identity Rotation", "[quaternion]") {
	REQUIRE(calc(Quaternion{ 1, 0, 0, 0 }, Quaternion{ 0, 3, -5, 7 }) == Quaternion{ 0,3,-5,7 });
}

TEST_CASE("Zero vector", "[quaternion]") {
	REQUIRE(calc(Quaternion{ 1, 0, 0, 0 }, Quaternion{ 0, 0, 0, 0 }) == Quaternion{ 0,0,0,0 });
}

TEST_CASE("90 degree about Z axis", "[quaternion]") {
	REQUIRE(calc(Quaternion{ 0.70710678, 0, 0, 0.70710678 }, Quaternion{ 0, 1, 0, 0 }) == Quaternion{ 0,0,1,0 });
}

TEST_CASE("90 degree about Y axis", "[quaternion]") {
	REQUIRE(calc(Quaternion{ 0.70710678, 0, 0.70710678, 0 }, Quaternion{ 0, 1, 0, 0 }) == Quaternion{ 0,0,0,-1 });
}

TEST_CASE("90 degree about X axis", "[quaternion]") {
	REQUIRE(calc(
		Quaternion{ 0.70710678, 0.70710678, 0, 0 },
		Quaternion{ 0, 0, 1, 0 }
	) == Quaternion{ 0, 0, 0, 1 });
}

TEST_CASE("180 degree about Z axis", "[quaternion]") {
	REQUIRE(calc(
		Quaternion{ 0, 0, 0, 1 },
		Quaternion{ 0, 2, 3, 4 }
	) == Quaternion{ 0, -2, -3, 4 });
}

TEST_CASE("180 degree about Y axis", "[quaternion]") {
	REQUIRE(calc(
		Quaternion{ 0, 0, 1, 0 },
		Quaternion{ 0, 5, -2, 7 }
	) == Quaternion{ 0, -5, -2, -7 });
}
TEST_CASE("120 degree rotation about (1,1,1)", "[quaternion]") {

	REQUIRE(calc(
		Quaternion{ 0.5, 0.5, 0.5, 0.5 },
		Quaternion{ 0, 1, 0, 0 }
	) == Quaternion{ 0,0,1,0 });
}

TEST_CASE("Rotation preserves vector magnitude", "[quaternion]") {
	Quaternion v{ 0, 3, 4, 12 };
	Quaternion result = calc(
		Quaternion{ 0.9238795, 0, 0.3826834, 0 },
		v
	);

	float originalNorm =
		std::sqrt(3 * 3 + 4 * 4 + 12 * 12);
	float rotatedNorm =
		std::sqrt(result.x * result.x +
			result.y * result.y +
			result.z * result.z);

	REQUIRE_THAT(originalNorm, Catch::Matchers::WithinRel(rotatedNorm, LOOSE_EPS));
}
TEST_CASE("Rotation followed by inverse recovers original vector", "[quaternion]") {
	Quaternion q{ 0.8660254, 0, 0, 0.5 };
	Quaternion q_conj{ 0.8660254, 0, 0, -0.5 };
	Quaternion v{ 0, 4, 1, -2 };

	Quaternion v1 = calc(q, v);
	Quaternion v2 = calc(q_conj, v1);

	REQUIRE(v2 == v);
}
TEST_CASE("Very small angle rotation", "[quaternion]") {
	float theta = 0.0001;
	Quaternion q{
		std::cos(theta / 2),
		0,
		0,
		std::sin(theta / 2)
	};

	Quaternion result = calc(q, Quaternion{ 0, 1000, 0, 0 });

	REQUIRE(result.x == Catch::Approx(1000).epsilon(1e-5));
	REQUIRE(result.y == Catch::Approx(0.1).epsilon(1e-5));
	REQUIRE(result.z == Catch::Approx(0).epsilon(1e-5));
}
TEST_CASE("Non-unit quaternion is rejected or normalized", "[quaternion]") {
	Quaternion q{ 2, 0, 0, 0, true };
	Quaternion v{ 0, 1, 2, 3 };

	Quaternion result = calc(q, v);


	REQUIRE(result == Quaternion{ 0, 1, 2, 3 });
}


TEST_CASE("Randomized quaternion rotation invariants", "[quaternion][random]") {
	std::mt19937 gen(42);
	std::uniform_real_distribution<float> dist(-1.0, 1.0);

	auto randomUnitQuaternion = [&]() -> Quaternion {
		float x = dist(gen), y = dist(gen), z = dist(gen), w = dist(gen);
		float norm = std::sqrt(w * w + x * x + y * y + z * z);
		return Quaternion{ w / norm, x / norm, y / norm, z / norm };
		};

	auto randomVector = [&]() -> Quaternion {
		return Quaternion{ 0, dist(gen) * 10, dist(gen) * 10, dist(gen) * 10 };
		};

	for (int i = 0; i < 1000; ++i) {
		Quaternion q = randomUnitQuaternion();
		Quaternion q_conj = Quaternion{ q.w, -q.x, -q.y, -q.z };
		Quaternion v = randomVector();

		Quaternion v_rot = calc(q, v);
		Quaternion v_back = calc(q_conj, v_rot);

		// Magnitude preserved
		float norm_v = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		float norm_v_rot = std::sqrt(v_rot.x * v_rot.x + v_rot.y * v_rot.y + v_rot.z * v_rot.z);
		REQUIRE(norm_v_rot == Catch::Approx(norm_v).epsilon(1e-6));
		//REQUIRE(epsilonComp(norm_v_rot, norm_v, 1e-6));
		// Forward + inverse returns original
		REQUIRE(v_back == v);

		// Zero vector stays zero
		Quaternion zero{ 0,0,0,0 };
		REQUIRE(calc(q, zero) == zero);
	}
}
TEST_CASE("Sequential rotations match combined quaternion", "[quaternion][composition]") {
	Quaternion q1{ 0.9238795, 0.3826834, 0, 0 }; // 45° X
	Quaternion q2{ 0.9238795, 0, 0.3826834, 0 }; // 45° Y
	Quaternion v{ 0, 1, 0, 0 };

	// Sequential rotations
	Quaternion v_seq = calc(q2, calc(q1, v));

	// Combined rotation
	auto combine = [](const Quaternion& a, const Quaternion& b) {
		return Quaternion{
			a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
			a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
			a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
			a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
		};
		};

	Quaternion q_combined = combine(q2, q1);
	Quaternion v_combined = calc(q_combined, v);

	REQUIRE(v_seq == v_combined);
}
TEST_CASE("Quaternion rotation matches rotation matrix", "[quaternion][matrix]") {
	auto toMatrix = [](const Quaternion& q) {
		float w = q.w, x = q.x, y = q.y, z = q.z;
		// Row-major 3x3 rotation matrix
		return std::array<std::array<float, 3>, 3>{{
			{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
			{ 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w) },
			{ 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y) }
			}};
		};

	Quaternion q{ 0.70710678, 0.70710678, 0, 0 }; // 90° about X
	Quaternion v{ 0, 0, 1, 0 };

	Quaternion v_rot = calc(q, v);
	auto m = toMatrix(q);

	std::array<float, 3> v_mat{
		m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
		m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
		m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
	};

	REQUIRE(epsilonComp(v_rot.x, v_mat[0], 1e-6));
	REQUIRE(epsilonComp(v_rot.y, v_mat[1], 1e-6));
	REQUIRE(epsilonComp(v_rot.z, v_mat[2], 1e-6));
}
TEST_CASE("Long-term rotation drift", "[quaternion][drift]") {
	const int steps = 10000;
	const float theta = 0.01; // 0.01 rad ~ 0.57°
	Quaternion q{ std::cos(theta / 2), 0, 0, std::sin(theta / 2) }; // Z-axis
	Quaternion v{ 0, 1, 0, 0 }; // Start along X-axis

	Quaternion v_rot = v;
	Quaternion q_unit = q;

	for (int i = 0; i < steps; ++i) {
		v_rot = calc(q_unit, v_rot);

		// Optional: renormalize quaternion to prevent drift
		float norm_q = std::sqrt(q_unit.w * q_unit.w + q_unit.x * q_unit.x + q_unit.y * q_unit.y + q_unit.z * q_unit.z);
		q_unit.w /= norm_q;
		q_unit.x /= norm_q;
		q_unit.y /= norm_q;
		q_unit.z /= norm_q;
	}

	// Magnitude should still be ~1
	float v_norm = std::sqrt(v_rot.x * v_rot.x + v_rot.y * v_rot.y + v_rot.z * v_rot.z);
	//REQUIRE(v_norm == Catch::Approx(1.0).epsilon(1e-5));
	REQUIRE(epsilonComp(v_norm, 1.0, 1e-3));
	// Expected angle after 10k steps: total rotation = steps * theta
	float total_angle = steps * theta;
	float expected_x = std::cos(total_angle);
	float expected_y = std::sin(total_angle);
	REQUIRE(epsilonComp(v_rot.x, expected_x, 1e-3));
	REQUIRE(epsilonComp(v_rot.y, expected_y, 1e-3));

}
