#include <iostream>
#include <map>
#include <ranges>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <unordered_map>
#include "build/_deps/catch2-src/src/catch2/matchers/catch_matchers_floating_point.hpp"
#include "build/_deps/catch2-src/extras/catch_amalgamated.hpp"
struct Quaternion {
	float w, x, y, z;
	std::unordered_map<std::string, float> quat;
	using iterator = std::unordered_map<std::string, float>::iterator;
	using const_iterator = std::unordered_map<std::string, float>::const_iterator;
	friend bool operator==(const Quaternion& lhs, const Quaternion& rhs);
	iterator begin() {
		return quat.begin();
	}
	iterator end() {
		return quat.end();
	}
	const_iterator begin() const {
		return quat.cbegin();
	}
	const_iterator end() const {
		return quat.cend();
	}
	Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {
		quat["w"] = w;
		quat["i"] = x;
		quat["j"] = y;
		quat["k"] = z;
	}
	Quaternion(float w, float x, float y, float z, bool normalize) : w(w), x(x), y(y), z(z) {
		if (normalize) {
			Quaternion normalizedQuat = normalizeUnit(Quaternion{ w,x,y,z });
			w = normalizedQuat.w;
			x = normalizedQuat.x;
			y = normalizedQuat.y;
			z = normalizedQuat.z;
			quat["w"] = normalizedQuat.w;
			quat["i"] = normalizedQuat.x;
			quat["j"] = normalizedQuat.y;
			quat["k"] = normalizedQuat.z;
		}
	}
	Quaternion normalizeUnit(Quaternion quat) {
		float sqr = sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
		return Quaternion{ quat.w / sqr, quat.x / sqr, quat.y / sqr, quat.z / sqr };
	}
};

constexpr float LOOSE_EPS = 1e-5;
constexpr float TIGHT_EPS = 1e-5;
static bool epsilonComp(float a, float b, float epsilon = TIGHT_EPS) {
	return std::fabs(a - b) <= epsilon;
}
bool operator==(const Quaternion& lhs, const Quaternion& rhs)
{
	if (epsilonComp(lhs.w, rhs.w) && epsilonComp(lhs.x, rhs.x), epsilonComp(lhs.y, rhs.y) && epsilonComp(lhs.z, rhs.z)) {
		return true;
	}
	else {
		return false;
	}
}

std::string q_mult(std::string u_key, std::string p_key) {
	if ((u_key + p_key) == "ii") {
		return "-1";
	}

	if ((u_key + p_key) == "ij") {
		return "k";
	}

	if ((u_key + p_key) == "ik") {
		return "-j";
	}
	if ((u_key + p_key) == "ji") {
		return "-k";
	}

	if ((u_key + p_key) == "jj") {
		return "-1";
	}

	if ((u_key + p_key) == "jk") {
		return "i";
	}

	if ((u_key + p_key) == "ki") {
		return "j";
	}

	if ((u_key + p_key) == "kj") {
		return "-i";
	}

	if ((u_key + p_key) == "kk") {
		return "-1";
	}
	if (u_key == "w") {
		return p_key;
	}
	if (p_key == "w") {
		return u_key;
	}
	return u_key + p_key;
};
static Quaternion calc(const Quaternion& unit_q, const Quaternion& pure_q) {
	//calculate conjugate of unit quaternion
	std::map<std::string, float> unit_q_conjugate;
	for (auto& [key, value] : unit_q) {
		if (key != "w")
			unit_q_conjugate[key] = (float)-1 * value;
		else
			unit_q_conjugate[key] = value;
	}
	//std::cout << "Conjugate of unit quaternion :- " << "\n";
	for (auto& [key, value] : unit_q_conjugate) {
		std::cout << value << key << "\n";
	}

	//multiply unit quaternion to pure quaternion
	std::multimap<std::string, float> res;
	for (auto& [u_key, u_value] : unit_q) {
		for (auto& [p_key, p_value] : pure_q) {
			if (p_value != 0 && u_value != 0) {
				if ((u_value * p_value) != 0) {
					if (q_mult(u_key, p_key) == "-1") {
						res.insert({ "w", -(u_value * p_value) });
					}
					else {
						if (q_mult(u_key, p_key).contains("-")) {

							res.insert({ q_mult(u_key, p_key)
								.replace(0,1,""), -(u_value * p_value) });
						}
						else {
							res.insert({ q_mult(u_key, p_key) , u_value * p_value });
						}
					}

				}

			}

		}
	}
	auto [begin_w, end_w] = res.equal_range("w");
	auto w = std::ranges::subrange(begin_w, end_w);
	auto [begin_i, end_i] = res.equal_range("i");
	auto i = std::ranges::subrange(begin_i, end_i);
	auto [begin_j, end_j] = res.equal_range("j");
	auto j = std::ranges::subrange(begin_j, end_j);
	auto [begin_k, end_k] = res.equal_range("k");
	auto k = std::ranges::subrange(begin_k, end_k);
	float sum_w = 0;
	float sum_i = 0;
	float sum_j = 0;
	float sum_k = 0;
	for (auto& [_, value] : w) {
		sum_w += value;
	}
	for (auto& [_, value] : i) {
		sum_i += value;
	}
	for (auto& [_, value] : j) {
		sum_j += value;
	}
	for (auto& [_, value] : k) {
		sum_k += value;
	}
	std::map<std::string, float> qv{ {"w", sum_w},{"i", sum_i},{"j", sum_j},{"k", sum_k} };
	//std::cout << "Multiplication between q and v :- " << "\n";
	//std::cout << "(" << sum_w << "," << sum_i << "," << sum_j << "," << sum_k << ")" << "\n";
	//std::cout << "--------------------" << "\n";
	//for (auto& [key, value] : qv) {
	//	std::cout << value << key << "\n";
	//}

	//multiply qv to q(conjugate)
	std::multimap<std::string, float> qvq;
	for (auto& [u_key, u_value] : qv) {
		for (auto& [p_key, p_value] : unit_q_conjugate) {
			if (p_value != 0 && u_value != 0) {
				if ((u_value * p_value) != 0) {
					if (q_mult(u_key, p_key) == "-1") {
						qvq.insert({ "w", -(u_value * p_value) });
					}
					else {
						if (q_mult(u_key, p_key).contains("-")) {

							qvq.insert({ q_mult(u_key, p_key)
								.replace(0,1,""), -(u_value * p_value) });
						}
						else {
							qvq.insert({ q_mult(u_key, p_key) , u_value * p_value });
						}
					}

				}

			}

		}
	}
	/*for (auto& [key, value] : qvq) {
		std::cout << value << key;
	}*/
	auto [b_w, e_w] = qvq.equal_range("w");
	auto w_ = std::ranges::subrange(b_w, e_w);
	auto [b_i, e_i] = qvq.equal_range("i");
	auto i_ = std::ranges::subrange(b_i, e_i);
	auto [b_j, e_j] = qvq.equal_range("j");
	auto j_ = std::ranges::subrange(b_j, e_j);
	auto [b_k, e_k] = qvq.equal_range("k");
	auto k_ = std::ranges::subrange(b_k, e_k);
	float s_w = 0;
	float s_i = 0;
	float s_j = 0;
	float s_k = 0;
	for (auto& [_, value] : w_) {
		s_w += value;
	}
	for (auto& [_, value] : i_) {
		s_i += value;
	}
	for (auto& [_, value] : j_) {
		s_j += value;
	}
	for (auto& [_, value] : k_) {
		s_k += value;
	}
	//std::cout << "Multiplication between qv and q(conjugate) :- " << "\n";
	std::cout << "(" << s_w << "," << s_i << "," << s_j << "," << s_k << ")" << "\n";
	return { s_w, s_i, s_j, s_k };
}


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
	REQUIRE(calc(Quaternion{ 0.70710678, 0, 0.70710678, 0 },Quaternion{ 0, 1, 0, 0 }) == Quaternion{ 0,0,0,-1 });
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
	)==Quaternion{0,0,1,0});
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

	REQUIRE(v2== v);
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
#include <random>

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

		// Forward + inverse returns original
		REQUIRE(v_back == v );

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
