#pragma once
#include <Eigen/Core>
#include <list>
#include <functional>

namespace Optimazation
{
	class PSO
	{
		public:
		PSO(unsigned int dim, std::function<double(const Eigen::VectorXd &)> eval, unsigned int n_particles) : 
			_dim(dim), _eval(eval), particles(n_particles)
		{
			for (auto p = particles.begin(); p != particles.end(); p++)
				*p = Eigen::VectorXd::Random(_dim);
		}
		void initParticles(Eigen::VectorXd min, Eigen::VectorXd max)
		{
			_min = min;
			_max = max;
			for (auto p = particles.begin(); p != particles.end(); p++)
			{
				*p = Eigen::VectorXd::Random(_dim);
				for (unsigned int i = 0; i < _dim; i++)
					(*p)[i] = (max[i] - min[i]) * (*p)[i] + min[i];
			}
		}
		std::list<Eigen::VectorXd> particles;
		std::list<Eigen::VectorXd> local_best;
		Eigen::VectorXd global_best;
		private:
		unsigned int _dim;
		Eigen::VectorXd _min, _max;
		std::function<double(const Eigen::VectorXd &)> _eval;
	};
}
