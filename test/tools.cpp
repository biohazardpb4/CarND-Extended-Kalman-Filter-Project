#include "gtest/gtest.h"
#include <vector>
#include "../src/Eigen/Dense"
#include "../src/tools.h"

using Eigen::VectorXd;

TEST(RMSE, CanBeCalculated)
{
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	//the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

	Tools tools;
	
	auto got = tools.CalculateRMSE(estimations, ground_truth);
        VectorXd want(4);
	want << 0.1, 0.1, 0.1, 0.1;
	EXPECT_FLOAT_EQ(got[0], want[0]);
	EXPECT_FLOAT_EQ(got[1], want[1]);
	EXPECT_FLOAT_EQ(got[2], want[2]);
	EXPECT_FLOAT_EQ(got[3], want[3]);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
