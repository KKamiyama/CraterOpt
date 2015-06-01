#pragma once
#define _USE_MATH_DEFINES
#include <cmath>

#include "PSO.hpp"
#include <iostream>
#include <fstream>
#include <boost/format.hpp>

namespace kamiyama
{
	double rastrigin(const Eigen::VectorXd &x)
	{
		double sum = 0.0;
		for (int i = 0; i < x.size(); i++)
			sum += x[i] * x[i] - 10 * cos(2 * M_PI * x[i]);
		return sum + 10 * x.size();
	}
	double simple_pow(const Eigen::VectorXd &x)
	{
		return x.squaredNorm();
	}

	void print_particles(const Optimazation::PSO &pso)
	{
		static int count = 0;
		std::ofstream ofs((boost::format("p_%03d.txt") % count++).str());
		for (auto p = pso.particles.begin(); p != pso.particles.end(); p++)
			ofs << p->transpose() << " " << simple_pow(*p) << "\n";
	}

	void test()
	{
		using Eigen::VectorXd;
		const int dim = 2;

		Optimazation::PSO pso(dim, simple_pow, 10);

		pso.initParticles(VectorXd::Constant(dim, -5), VectorXd::Constant(dim, 5));

		for (int i = 0; i < 10; i++)
		{
			print_particles(pso);
			std::cout << "iter: " << i << ", gb = " << pso.global_best_val << " at " << pso.global_best.transpose() << std::endl;
			pso.update();
		}

		std::cout << "Not implemented yet!\nPress enter to exit." << std::endl;
		std::rewind(stdin);
		std::getchar();
	}
}
