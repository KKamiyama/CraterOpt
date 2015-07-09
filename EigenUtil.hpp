#pragma once
#include <Eigen/Dense>
#include <random>

namespace Eigen
{
	class Util{
	public:
		static void SetSeed(unsigned long seed);
		static void SetSeed(std::seed_seq &seed_seq);
		static Eigen::VectorXd GenerateRandomVector(int dim, double min, double max);
		static Eigen::VectorXi Eigen::Util::GenerateRandomVectorInt(int dim, int min, int max);
	private:
		Util();
		static std::mt19937 mt;
	};	
}
