#ifndef QUADROPTED_BENCHMARK_H
#define QUADROPTED_BENCHMARK_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>

namespace benchmark {

void print_header(const std::string& title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

void print_joints(const std::string& label, const Eigen::VectorXd& joints) {
    std::cout << label << ": [";
    for (int i = 0; i < joints.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << joints(i);
        if (i < joints.size() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
}

void print_foot_locations(const std::string& label, const Eigen::MatrixXd& feet) {
    std::cout << label << ":\n";
    std::cout << "  FR: [" << feet(0,0) << ", " << feet(1,0) << ", " << feet(2,0) << "]\n";
    std::cout << "  FL: [" << feet(0,1) << ", " << feet(1,1) << ", " << feet(2,1) << "]\n";
    std::cout << "  RR: [" << feet(0,2) << ", " << feet(1,2) << ", " << feet(2,2) << "]\n";
    std::cout << "  RL: [" << feet(0,3) << ", " << feet(1,3) << ", " << feet(2,3) << "]\n";
}

Eigen::MatrixXd create_default_stance() {
    double body[] = {0.3762, 0.0935};
    double legs[] = {0.0, 0.0955, 0.213, 0.213};
    double dx = body[0] * 0.5 + 0.02;
    double dy = body[1] * 0.5 + legs[1];
    
    Eigen::MatrixXd stance(3, 4);
    stance <<  dx,  dx, -dx, -dx,
              -dy,  dy, -dy,  dy,
                0,   0,   0,   0;
    return stance;
}

}

#endif // QUADROPTED_BENCHMARK_H
