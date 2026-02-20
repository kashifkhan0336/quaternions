## Quaternions from First Principles
*Work in Progress: SLERP & Dual Quaternions coming soon*

Why? Because learnopengl.com said "out of scope" and I got annoyed.

What works now:
- ✅ Quaternion multiplication, conjugation, normalization
- ✅ Vector rotation (tested against rotation matrices)
- ✅ 16 test cases, 3021 assertions (see `tests/`)
- ✅ CMake build, GitHub Actions CI

What's next:
- 🚧 SLERP for smooth interpolation
- 🚧 Dual quaternions for rigid body transforms

Derivation journey: Unit circle → Rodrigues → Stereographic projection → Clifford algebras → SO(3) double cover. Notes coming soon.
