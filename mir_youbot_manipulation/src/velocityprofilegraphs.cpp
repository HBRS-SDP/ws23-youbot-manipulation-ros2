#include "matplotlibcpp.h"
#include <cmath>

namespace plt = matplotlibcpp;

double calculatePosition(double amplitude, double start_pose, double target_pose, double current_pose) {
    return amplitude * std::sin((M_PI / (target_pose - start_pose)) * current_pose);
}

int main() 
{
    // Set the start, target, and current positions
    double Start_pos_x = 0.0, Start_pos_y = 0.0, Start_pos_z = 0.0;
    double Target_pos_x = 0.1, Target_pos_y = 0.03, Target_pos_z = 0.15;
    // double Target_pos_x = 0.25732, Target_pos_y = 0.0849215, Target_pos_z = 0.2807;

    // Set the amplitude
    double Amplitude = 1.0;

    // Calculate the end times based on the differences between target and start positions
    double End_time_x = Target_pos_x - Start_pos_x;
    double End_time_y = Target_pos_y - Start_pos_y;
    double End_time_z = Target_pos_z - Start_pos_z;

    // Create vectors to store the t values and corresponding values for x, y, and z
    std::vector<double> t_values_x, t_values_y, t_values_z;
    std::vector<double> x_values, y_values, z_values;

    // Generate values for t, x, y, and z based on the calculatePosition function for each dimension
    for (double t = 0.0; t <= End_time_x; t += 0.0001) {
        double current_x = Start_pos_x + t;
        double value_x = calculatePosition(Amplitude, Start_pos_x, Target_pos_x, current_x);

        t_values_x.push_back(t);
        x_values.push_back(value_x);
    }

    for (double t = 0.0; t <= End_time_y; t += 0.0001) {
        double current_y = Start_pos_y + t;
        double value_y = calculatePosition(Amplitude, Start_pos_y, Target_pos_y, current_y);

        t_values_y.push_back(t);
        y_values.push_back(value_y);
    }

    for (double t = 0.0; t <= End_time_z; t += 0.0001) {
        double current_z = Start_pos_z + t;
        double value_z = calculatePosition(Amplitude, Start_pos_z, Target_pos_z, current_z);

        t_values_z.push_back(t);
        z_values.push_back(value_z);
    }

    // Plot the results
    plt::figure();
    plt::plot(t_values_x, x_values, "r-", {{"label", "X-coordinate"}});
    plt::title("X-coordinate");
    plt::legend();
    plt::show();

    plt::figure();
    plt::plot(t_values_y, y_values, "g-", {{"label", "Y-coordinate"}});
    plt::title("Y-coordinate");
    plt::legend();
    plt::show();

    plt::figure();
    plt::plot(t_values_z, z_values, "b-", {{"label", "Z-coordinate"}});
    plt::title("Z-coordinate");
    plt::legend();
    plt::show();

    return 0;
}
