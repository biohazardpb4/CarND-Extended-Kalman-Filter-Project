#include "gtest/gtest.h"
#include <vector>
#include "../src/Eigen/Dense"
#include "../src/tools.h"

using Eigen::VectorXd;

TEST(RMSE, CanBeCalculated)
{
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;
	Tools tools;
	
	auto got = tools.CalculateRMSE(estimations, ground_truth);
        VectorXd want(4);
	want << 0,0,0,0;
	EXPECT_EQ(got, want);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
