#pragma once

#define USE_MATH_DEFINES

#include "EigenUtil.hpp"

#include <iostream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace okada
{
	// Number of Particles
	const int N = 10;
	// Dimension of State Variable
	const int DIM = 2;

	typedef  std::vector<Eigen::VectorXd> vecXd;
	void init(vecXd&, vecXd&, vecXd&, Eigen::VectorXd&, double*);
	double EvalFunc(Eigen::VectorXd);

	void PrintEvalFunc()
	{
		std::ofstream ofs("EvalFunc.txt");
		for (double x = -5; x <= 5; x += 0.1)
		{
			for (double y = -5; y <= 5; y += 0.1)
			{
				std::cout << x << " " << y << std::endl;
				ofs << x << " " << y << " " << EvalFunc((Eigen::VectorXd(2) << x, y).finished()) << "\n";
			}
			ofs << "\n";
		}
	}
	void test()
	{
		std::ofstream ofs("out.txt");
		double c1 = 0.5, c2 = 0.5, r1, r2, w = 0.8;
		double gb;
		srand(time(NULL));
		Eigen::VectorXd xg(DIM);
		vecXd x(N), v(N), xhat(N);

		init(x, v, xhat, xg, &gb);

		ofs << "#gb_pos gb_val" << "\n";
		for (int n = 0; n<100; n++)
		{
			r1 = (double)rand() / RAND_MAX;
			r2 = (double)rand() / RAND_MAX;
			for (int i = 0; i<N; i++)
			{
				v[i] = w*v[i] + c1*r1*(xhat[i] - x[i]) + c2*r2*(xg - x[i]);
				x[i] += v[i];
			}
			for (int i = 0; i<N; i++)
			{
				if (EvalFunc(xhat[i])>EvalFunc(x[i]))
				{
					xhat[i] = x[i];
				}
				if (gb>EvalFunc(x[i]))
				{
					xg = x[i];
					gb = EvalFunc(x[i]);
				}
			}
			ofs << xg.transpose() << " " << gb << std::endl;
		}
		std::cout << "fitness=" << gb << std::endl;
		std::cout << xg.transpose() << std::endl << std::endl;
		ofs.close();

		std::cout << "Press enter to exit." << std::endl;
		std::rewind(stdin);
		std::getchar();
	}

	double EvalFunc(Eigen::VectorXd x)
	{
		double ans = 0.0;
		for (int i = 0; i < DIM; i++)
			ans += x[i] * x[i] - 10 * cos(2 * M_PI * x[i]);
		return 10 * DIM + ans;
	}

	void init(vecXd &position, vecXd &velocity, vecXd& lb, Eigen::VectorXd &gbpos, double* gbval)
	{
		srand(time(NULL));
		for (int i = 0; i < N; i++)
		{
			position[i].resize(DIM);
			velocity[i].resize(DIM);
			for (int j = 0; j < DIM; j++)
			{
				position[i](j) = (double)rand() / RAND_MAX - 0.5;
				velocity[i](j) = (double)rand() / RAND_MAX - 0.5;
			}
			if (i == 0)
			{
				gbpos = position[i];
				*gbval = EvalFunc(position[i]);
			}
			lb[i].resize(DIM);
			lb[i] = position[i];
		}
	}
	class FA
	{
	public:
		FA(unsigned int dim, std::function<double(const Eigen::VectorXd &)> eval, unsigned int n_fireflies) :
			_dim(dim), _eval(eval), fireflies(n_fireflies), intensity(n_fireflies)
		{
			for (auto p = fireflies.begin(); p != fireflies.end(); p++)
			{
				*p = Eigen::VectorXd::Random(_dim);
			}

		}
		// TODO: min, maxが適用されていない
		void initFireflies(Eigen::VectorXd min, Eigen::VectorXd max)
		{
			_min = min;
			_max = max;
			for (int p = 0; p < fireflies.size(); p++)
			{
				fireflies[p] = Eigen::VectorXd::Random(_dim);//[-1:1]

				for (unsigned int i = 0; i < _dim; i++)
				{
					fireflies[p][i] = (max[i] - min[i])*(fireflies[p][i] + 1.0)/2.0 + min[i];
					intensity[p] = _eval(fireflies[p]);
					if (intensity[p] < global_best_val)
					{
						global_best_val = intensity[p];
						global_best = fireflies[p];
					}
				}
			}
		}
		std::vector<Eigen::VectorXd> fireflies;
		std::vector<double> intensity;
		Eigen::VectorXd global_best;
		double global_best_val = DBL_MAX;
	
		void update(double min, double max, int dim, bool option){
			for (int i = 0; i < fireflies.size(); i++)
			{
				for (int j = 0; j < i; j++)
				{
					// MEMO: 最小化問題を解くことにします。最大化問題は符号を反転すれば最小化問題に変換可能です。 by kamiyama
					if (intensity[j] < intensity[i])		//不等号の向きは解く問題の種類による?
					{
						e = Eigen::Util::GenerateRandomVector(dim, min, max);
						r = (fireflies[i] - fireflies[j]).norm();
						fireflies[i] += b*exp(-gamma*r*r)*(fireflies[j] - fireflies[i]) + a*e;
						intensity[i] = _eval(fireflies[i]);
						if (intensity[i] < global_best_val)
						{
							global_best_val = intensity[i];
							global_best = fireflies[i];
						}
					}
					if (option){
						u = Eigen::Util::GenerateRandomVector(dim, min, max);
						global_best += u;
						global_best_val = _eval(global_best);
					}
				}
			}
		}
	private:
		unsigned int _dim;
		Eigen::VectorXd _min, _max, e,u;
		std::function < double(const Eigen::VectorXd &)> _eval;
		double a = 0.1, b, r, gamma = 1.0;

	};
	
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
	void print_fireflies(const FA &fa)
	{
		static int count = 0;
		std::ofstream ofs((boost::format("p_%03d.txt") % count++).str());
		for (auto p = fa.fireflies.begin(); p != fa.fireflies.end(); p++)
			ofs << p->transpose() << " " << simple_pow(*p) << "\n";
	}
	void FAtest( bool option=false )
	{
		using Eigen::VectorXd;
		const int dim = 2;
/*
		Eigen::VectorXd v = Eigen::Util::GenerateRandomVector(5, 11.1, 22.2);

		std::cout << v << std::endl;
		rewind(stdin);
		getchar();
		return;*/

		FA fa(dim, rastrigin, 10);

		fa.initFireflies(Eigen::VectorXd::Constant(dim, -10), Eigen::VectorXd::Constant(dim, 10));

		for (int i = 0; i < 10; i++)
		{
			print_fireflies(fa);
			std::cout << "iter: " << i << ", gb = " << fa.global_best_val << " at " << fa.global_best.transpose() << std::endl;
			//min,max,gbの更新の有無(default=true)
			fa.update(-10, 10, dim, option);
		}

		std::cout << "Not implemented yet!\nPress enter to exit." << std::endl;
		std::rewind(stdin);
		std::getchar();
	}
}
