#pragma once
#include <Eigen/Core>
#include <vector>
#include <functional>
#include <iostream>

namespace Optimazation
{
	class PSO
	{
		public:
		PSO(unsigned int dim, std::function<double(const Eigen::VectorXd &)> eval, unsigned int n_particles) : 
			_dim(dim), _eval(eval), particles(n_particles), velocities(n_particles), local_best(n_particles), local_best_val(n_particles)
		{
			srand(NULL);
			for (int i = 0; i < n_particles; i++)
				local_best[i] = particles[i] = Eigen::VectorXd::Random(_dim);
			for (auto v = velocities.begin(); v != velocities.end(); v++)
				*v = Eigen::VectorXd::Random(_dim) * _init_vel_max;
		}
		void initParticles(Eigen::VectorXd min, Eigen::VectorXd max)
		{
			_min = min;
			_max = max;
			for (int p = 0; p < particles.size(); p++)
			{
				particles[p] = Eigen::VectorXd::Random(_dim); // [-1:1]?
				for (unsigned int i = 0; i < _dim; i++)
				{
					local_best[p][i] = particles[p][i] = (max[i] - min[i]) * (particles[p][i] + 1.0) / 2 + min[i];
					local_best_val[p] = _eval(particles[p]);
					if (local_best_val[p] < global_best_val)
					{
						global_best_val = local_best_val[p];
						global_best = particles[p];
					}
				}
			}
		}
		std::vector<Eigen::VectorXd> particles;
		std::vector<Eigen::VectorXd> velocities;
		std::vector<Eigen::VectorXd> local_best;
		std::vector<double> local_best_val;
		Eigen::VectorXd global_best;
		double global_best_val = DBL_MAX;
		void update()
		{
			for (int i = 0; i < particles.size(); i++)
			{
				double r1 = (double)rand() / RAND_MAX;
				double r2 = (double)rand() / RAND_MAX;
				velocities[i] = w * velocities[i]
					+ c1 * r1 * (local_best[i] - particles[i])
					+ c2 * r2 * (global_best - particles[i]);
				particles[i] += velocities[i];
				double v = _eval(particles[i]);
				if (v < local_best_val[i])
				{
					local_best_val[i] = v;
					local_best[i] = particles[i];
					if (v < global_best_val)
					{
						global_best_val = v;
						global_best = particles[i];
					}
				}
			}
		}
		private:
		unsigned int _dim;
		Eigen::VectorXd _min, _max;
		std::function<double(const Eigen::VectorXd &)> _eval;
		double _init_vel_max = 1;
		double w = 0.5, c1 = 0.2, c2 = 0.2;
		
	};
}
