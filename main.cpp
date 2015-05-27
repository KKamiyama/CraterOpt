#define USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/StdVector>

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
int main()
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
		r1 = rand() / RAND_MAX;
		r2 = rand() / RAND_MAX;
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
