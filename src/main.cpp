#include <iostream>
#include <map>
#include <ranges>
#include <unordered_map>
#include <random>
#include <benchmark/benchmark.h>
#include "include/quaternion.hpp";
std::unordered_map<std::string, std::string> lookup_table_{
	{"ii","-1"}, 
	{"ij","k"}, 
	{"ik","-j"},
	{"ji", "-k"}, 
	{"jj", "-1"},
	{"jk", "i"},
	{"ki", "j"},
	{"kj", "-i"},
	{"kk", "-1"},
};

static void BM_QMULT_LOOKUP(benchmark::State& state) {
	for (auto _ : state) {
		lookup_table_["ii"];
	}
}
static void BM_ORIGINAL_CALC(benchmark::State& state) {
	for (auto _ : state) {
		calc(Quaternion{ 1, 0, 0,0 }, { 0,2,3,0 });
	}
}

// Register the function as a benchmark

BENCHMARK(BM_QMULT_LOOKUP);
BENCHMARK(BM_ORIGINAL_CALC);

BENCHMARK_MAIN();