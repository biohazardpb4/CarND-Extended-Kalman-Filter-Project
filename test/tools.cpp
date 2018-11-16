#include "gtest/gtest.h"
#include <vector>
#include "../src/Eigen/Dense"
#include "../src/tools.h"

using Eigen::VectorXd;

TEST(RMSE, CanBeCalculated)
{
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;
	EXPECT_EQ(1, 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
