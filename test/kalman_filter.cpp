#include "gtest/gtest.h"
#include <vector>
#include "../src/Eigen/Dense"
#include "../src/kalman_filter.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

TEST(KalmanFilter, Predict)
{
	KalmanFilter filter;
	VectorXd x;
	MatrixXd P;
	MatrixXd F;
	MatrixXd H;
	MatrixXd R;
	MatrixXd Q;

	filter.Init(x, P, F, H, R, Q);
        VectorXd want;
	VectorXd got = filter.x_;
	EXPECT_TRUE(want.isApprox(got, 1e-4));
}

TEST(KalmanFilter, Update) {
	//design the KF with 1D motion
	VectorXd x(2);
	x << 0, 0;

	MatrixXd P(2, 2);
	P << 1000, 0, 0, 1000;

	VectorXd u(2);
	u << 0, 0;

	MatrixXd F(2, 2);
	F << 1, 1, 0, 1;

	MatrixXd H(1, 2);
	H << 1, 0;

	MatrixXd R(1, 1);
	R << 1;

	MatrixXd Q(2, 2);
	Q << 0, 0, 0, 0;

	//create a list of measurements
	vector<VectorXd> measurements;
	VectorXd single_meas(1);
	single_meas << 1;
	measurements.push_back(single_meas);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 3;
	measurements.push_back(single_meas);

	KalmanFilter filter;
	filter.Init(x, P, F, H, R, Q);
	for (unsigned int n = 0; n < measurements.size(); n++) {
		VectorXd z = measurements[n];
		filter.Update(z);
 		filter.Predict();
	}
	VectorXd got_x(filter.x_);
	MatrixXd got_P(filter.P_);

	VectorXd want_x(2);
	want_x << 3.9996664447958645, 0.9999998335552873;
	MatrixXd want_P(2, 2);
	want_P << 2.3318904241194827, 0.9991676099921091, 0.9991676099921067, 0.49950058263974184;
    
	EXPECT_TRUE(want_x.isApprox(got_x, 1e-4));
	EXPECT_TRUE(want_P.isApprox(got_P, 1e-4));
}

TEST(KalmanFilter, UpdateEKF) {

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
