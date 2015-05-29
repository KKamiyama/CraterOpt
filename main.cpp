#define _USE_MATH_DEFINES
#include "KamiyamaSandBox.hpp"
#include "OkadaSandBox.hpp"

int main()
{
	std::cout << "Select test code [0:kamiyama, 1:okada]:" << std::endl;
	int test = 0;
	std::cin >> test;
	switch (test)
	{
	case 0:
		kamiyama::test();
		break;
	case 1:
		okada::test();
		break;
	default:
		break;
	}
}
