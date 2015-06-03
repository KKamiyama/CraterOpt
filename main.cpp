#define _USE_MATH_DEFINES
#include "KamiyamaSandBox.hpp"
#include "OkadaSandBox.hpp"

int main()
{
	std::cout << "Select test code [0:kamiyama, 1:okada]:" << std::endl;
	int test = 0,mode=0,option=0;
	std::cin >> test;
	switch (test)
	{
	case 0:
		kamiyama::test();
		break;
	case 1:
		std::cout << "Select mode [0:PSO, 1:Firefly]:" << std::endl;
		std::cin >> mode;
		switch (mode)
		{
		case 0:
			okada::test();
			break;
		case 1:
			std::cout << "最適解の更新 [0:なし　1：あり]:" << std::endl;
			std::cin >> option;
			okada::FAtest(option);
			break;
		default: break;
		}
		break;
	default:
		break;
	}
}
