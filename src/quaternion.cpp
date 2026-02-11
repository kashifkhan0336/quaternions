#include "quaternion.hpp"
#include <iostream>
#include <map>
#include <ranges>
#include <unordered_map>
#include <random>


float w, x, y, z;
std::unordered_map<std::string, float> quat;
using iterator = std::unordered_map<std::string, float>::iterator;
using const_iterator = std::unordered_map<std::string, float>::const_iterator;
bool operator==(const Quaternion& lhs, const Quaternion& rhs);
iterator Quaternion::begin() {
	return quat.begin();
}
iterator Quaternion::end() {
	return quat.end();
}
const_iterator Quaternion::begin() const {
	return quat.cbegin();
}
const_iterator Quaternion::end() const {
	return quat.cend();
}
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {
	quat["w"] = w;
	quat["i"] = x;
	quat["j"] = y;
	quat["k"] = z;
}
Quaternion::Quaternion(float w, float x, float y, float z, bool normalize) : w(w), x(x), y(y), z(z) {
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
Quaternion Quaternion::normalizeUnit(Quaternion quat) {
	float sqr = sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
	return Quaternion{ quat.w / sqr, quat.x / sqr, quat.y / sqr, quat.z / sqr };
}
bool epsilonComp(float a, float b, float epsilon = TIGHT_EPS) {
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
Quaternion calc(const Quaternion& unit_q, const Quaternion& pure_q) {
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

