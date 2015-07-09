#include "EigenUtil.hpp"

std::mt19937 Eigen::Util::mt = std::mt19937();
void Eigen::Util::SetSeed(unsigned long seed) {
	mt.seed(seed);
}
void Eigen::Util::SetSeed(std::seed_seq &seed_seq) {
	mt.seed(seed_seq);
}
Eigen::VectorXd Eigen::Util::GenerateRandomVector(int dim, double min, double max) {
	std::uniform_real_distribution<double> dist(min, max);
	Eigen::VectorXd v(dim);
	for (int i = 0; i < dim; i++)
		v[i] = dist(mt);
	return v;
}
Eigen::VectorXi Eigen::Util::GenerateRandomVectorInt(int dim, int min, int max) {
	std::uniform_int_distribution<int> dist(min, max);
	Eigen::VectorXi v(dim);
	for (int i = 0; i < dim; i++)
		v[i] = dist(mt);
	return v;
}
