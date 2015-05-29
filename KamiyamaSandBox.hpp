#pragma once
#define _USE_MATH_DEFINES
#include <cmath>

#include "PSO.hpp"
#include <iostream>

namespace kamiyama
{
	double rastrigin2d(const Eigen::VectorXd &x)
	{
		double sum = 0.0;
		for (int i = 0; i < x.size(); i++)
			sum += x[i] * x[i] - 10 * cos(2 * M_PI * x[i]);
		return sum;
	}

	void test()
	{
		using Eigen::VectorXd;
		const int dim = 2;

		Optimazation::PSO pso(dim, rastrigin2d, 20);

		VectorXd min(dim), max(dim);
		min << -5, -5;
		max << 5, 5;

		pso.initParticles(min, max);

		std::cout << "Not implemented yet!\nPress enter to exit." << std::endl;
		std::rewind(stdin);
		std::getchar();
	}
}
