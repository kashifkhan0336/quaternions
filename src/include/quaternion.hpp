#pragma once
#include <unordered_map>
#include <string>
struct Quaternion
{
	float w, x, y, z;
	std::unordered_map<std::string, float> quat;
	using iterator = std::unordered_map<std::string, float>::iterator;
	using const_iterator = std::unordered_map<std::string, float>::const_iterator;
	friend bool operator==(const Quaternion& lhs, const Quaternion& rhs);
	iterator begin();
	iterator end();
	const_iterator begin() const;
	const_iterator end() const;
	Quaternion(float w, float x, float y, float z);
	Quaternion(float w, float x, float y, float z, bool normalize);
	Quaternion normalizeUnit(Quaternion quat);
};
Quaternion calc(const Quaternion& unit_q, const Quaternion& pure_q);
bool epsilonComp(float a, float b, float epsilon);
inline constexpr float LOOSE_EPS = 1e-5;
inline constexpr float TIGHT_EPS = 1e-5;