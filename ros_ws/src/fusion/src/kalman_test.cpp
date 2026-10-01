#include "kalman.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool approx(double actual, double expected, double tolerance = 1e-6)
{
	return std::fabs(actual - expected) <= tolerance;
}

bool report_check(const std::string &name, double actual, double expected)
{
	const bool ok = approx(actual, expected);
	std::cout << (ok ? "[PASS] " : "[FAIL] ") << name
			  << " | expected=" << expected << ", actual=" << actual << std::endl;
	return ok;
}

bool test_from_Fusion(){

    std::cout << "\nTest: Fusion" << std::endl;
    Eigen::MatrixXd Q_state(4,4);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            Q_state(i, j) = 0.2;
        }
    }
    Q_state += Eigen::MatrixXd::Identity(4,4);
    Q_state.block(2,2,2,2) += 2 * Eigen::MatrixXd::Identity(2,2);
    KalmanFilter state_kalman = KalmanFilter(4, Q_state);

    double delta_t = 0.1;
    Eigen::Matrix<double, 4, 4> Fx;
    Fx << 1, 0, delta_t, 0,
        0, 1, 0, delta_t,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    Eigen::Matrix<double, 4, 4>  old_covariance;
    old_covariance << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    Eigen::Matrix<double, 4, 1> s;
    s(0) = 10;
    s(1) = 20;
    s(2) = 1;
    s(3) = 1;

    kalman_pair p;
    p.covariance = old_covariance;
    p.state = s;
    kalman_pair res = state_kalman.predict(p, Fx, nullptr);
    Eigen::Matrix<double, 4, 4> new_cov = res.covariance;

	//The bug was that I was not updating the state, only the covariance, so the state remained the same as the input which was incorrect. The covariance was being updated correctly which is why we were not seeing any issues with it.
	//Here is the bug fix
	p.state = res.state;

    if (!approx(res.state(0, 0), 10.1) || !approx(res.state(1, 0), 20.1) || !approx(res.state(2, 0), 1) || !approx(res.state(3, 0), 1))
    {
        std::cout << "[FAIL] State prediction incorrect: " << res.state.transpose() << std::endl;
        return false;
    }
    else
    {
        std::cout << "[PASS] State prediction correct: " << res.state.transpose() << std::endl;
    }

    if (!approx(res.state(0, 0), p.state(0)) || !approx(res.state(1, 0), p.state(1)) || !approx(res.state(2, 0), p.state(2)) || !approx(res.state(3, 0), p.state(3)))
    {
        std::cout << "[FAIL] BUG FOUND" << std::endl;
        return false;
    }
    else
    {
        std::cout << "[PASS] No Bug" << std::endl;
    }
    return true;
}

kalman_pair make_pair_1d(double state_value, double covariance_value)
{
	kalman_pair pair;
	pair.state = Eigen::MatrixXd::Constant(1, 1, state_value);
	pair.covariance = Eigen::MatrixXd::Constant(1, 1, covariance_value);
	return pair;
}

bool test_predict_identity_F()
{
	std::cout << "\nTest: predict with F=1" << std::endl;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(1, 1, 0.1);
	KalmanFilter kf(1, Q);

	kalman_pair prior = make_pair_1d(2.0, 1.0);
	Eigen::MatrixXd F = Eigen::MatrixXd::Identity(1, 1);

	kalman_pair predicted = kf.predict(prior, F, nullptr);

	// Expected by hand:
	// x' = F*x = 1*2.0 = 2.0
	// P' = F*P*F^T + Q = 1*1.0*1 + 0.1 = 1.1
	const bool state_ok = report_check("x'", predicted.state(0, 0), 2.0);
	const bool covariance_ok = report_check("P'", predicted.covariance(0, 0), 1.1);
	return state_ok && covariance_ok;
}

bool test_predict_scale_F()
{
	std::cout << "\nTest: predict with F=2" << std::endl;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(1, 1, 0.1);
	KalmanFilter kf(1, Q);

	kalman_pair prior = make_pair_1d(3.0, 0.5);
	Eigen::MatrixXd F = Eigen::MatrixXd::Constant(1, 1, 2.0);

	kalman_pair predicted = kf.predict(prior, F, nullptr);

	// Expected by hand:
	// x' = 2*3.0 = 6.0
	// P' = 2*0.5*2 + 0.1 = 2.1
	const bool state_ok = report_check("x'", predicted.state(0, 0), 6.0);
	const bool covariance_ok = report_check("P'", predicted.covariance(0, 0), 2.1);
	return state_ok && covariance_ok;
}

bool test_refine_single_observation()
{
	std::cout << "\nTest: refine with one observation" << std::endl;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(1, 1);
	KalmanFilter kf(1, Q);

	kalman_pair estimate = make_pair_1d(10.0, 4.0);
	std::vector<kalman_pair> observations = {make_pair_1d(12.0, 1.0)};
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(1, 1);

	kf.refine(estimate, observations, H, nullptr);

	// Expected by hand (1D Kalman update):
	// K = P/(P+R) = 4/(4+1) = 0.8
	// x = x + K*(z-x) = 10 + 0.8*(12-10) = 11.6
	// P = (1-K)*P = 0.2*4 = 0.8
	const bool state_ok = report_check("x", estimate.state(0, 0), 11.6);
	const bool covariance_ok = report_check("P", estimate.covariance(0, 0), 0.8);
	return state_ok && covariance_ok;
}

bool test_refine_two_observations()
{
	std::cout << "\nTest: refine with two observations" << std::endl;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(1, 1);
	KalmanFilter kf(1, Q);

	kalman_pair estimate = make_pair_1d(0.0, 1.0);
	std::vector<kalman_pair> observations = {
		make_pair_1d(1.0, 1.0),
		make_pair_1d(3.0, 1.0)};
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(1, 1);

	kf.refine(estimate, observations, H, nullptr);

	// Expected by hand for stacked measurements:
	// K = [1/3, 1/3]
	// x = 0 + [1/3,1/3]*[1,3]^T = 4/3
	// P = (1 - 2/3)*1 = 1/3
	const bool state_ok = report_check("x", estimate.state(0, 0), 4.0 / 3.0);
	const bool covariance_ok = report_check("P", estimate.covariance(0, 0), 1.0 / 3.0);
	return state_ok && covariance_ok;
}

} // namespace

int main()
{
	int failed = 0;

	if (!test_predict_identity_F())
	{
		failed++;
	}
	if (!test_predict_scale_F())
	{
		failed++;
	}
	if (!test_refine_single_observation())
	{
		failed++;
	}
	if (!test_refine_two_observations())
	{
		failed++;
	}

	if(!test_from_Fusion())
	{
		failed++;
	}

	std::cout << "\nSummary: " << (5 - failed) << "/5 tests passed." << std::endl;
	return failed == 0 ? 0 : 1;
}
