#include <Eigen/Dense>
#include <iostream>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

class KalmanFilter {
private:
    int nx;
    Matrix F, G, H, K, R, Q, P;
    Vector x;

public:
    KalmanFilter(int num_states, int num_inputs, int num_measurements)
        : F(num_states, num_states), G(num_states, num_inputs), 
          H(num_measurements, num_states), 
          Q(num_states, num_states), R(num_measurements, num_measurements),
          K(num_states, num_measurements),
          P(num_states, num_states), x(num_states) 
    {
      F.setIdentity();
      G.setZero();
      H.setZero();
      Q.setIdentity();
      R.setIdentity();
      K.setZero();
      P.setIdentity();
      x.setZero();
      nx = num_states;
    }

    void setSystemDynamics(const Matrix& state_transition, 
                           const Matrix& input_transition,
                           const Matrix& observation_matrix, 
                           const Matrix& measurement_noise_covariance, 
                           const Matrix& process_noise_covariance) {
        F = state_transition;
        G = input_transition;
        H = observation_matrix; 
        Q = process_noise_covariance;
        R = measurement_noise_covariance;
    }

    void initialize(const Vector& initial_state, const Matrix& initial_estimate_covariance) {
        x = initial_state;
        P = initial_estimate_covariance;
    }

    void predict(const Vector& control_input) { 
        if (control_input.size() != G.cols()) {
            std::cerr << "Error: Control input size does not match G.cols()\n";
            return;
        }
        x = F*x + G*control_input;
        P = F*P*F.transpose() + Q;
    }

    void update(const Vector& measurement) {
        Matrix S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();
        x = x + K * (measurement - H * x);
        Matrix I = Matrix::Identity(nx, nx);
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose(); 
    }
    
    Vector filter(const Vector& measurement, const Vector& control_input) {
        predict(control_input);
        update(measurement);
        return x;
    }

    Vector getState() const { return x; }

    Matrix getCovariance() const { return P; }
};

int main() {
    KalmanFilter kf(2, 2, 2);

    Matrix F(2, 2), G(2, 2), H(2, 2), Q(2, 2), R(2, 2);
    F << 1, 1,
         0, 1;
    G.setZero();
    H.setIdentity();
    Q.setIdentity();
    R.setIdentity();

    kf.setSystemDynamics(F, G, H, R, Q);

    Vector x0(2);
    Matrix P0(2, 2);
    x0 << 0, 1;
    P0.setIdentity();

    kf.initialize(x0, P0);

    std::cout << "Initial State x:\n" << kf.getState() << "\n";
    std::cout << "Initial Covariance P:\n" << kf.getCovariance() << "\n";

    return 0;
}
