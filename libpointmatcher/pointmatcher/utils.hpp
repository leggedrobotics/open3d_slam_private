/*!
 * @file
 * @author Turcan Tuna (ANYbotics)
 * @date   02/07/2021
 * @brief  The declaration of the utilization functions of the slam_investigation class.
 */

#pragma once

// C++ standard library
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct DegeneracySolverOptions
{
bool isEnabled_{ true };

    bool useSophusParametrization_{ false };
bool useSophusAutoDiffParametrization_{ false };
bool useAngleAxisParametrization_{ true };

bool usePointToPoint_{ false };
bool usePointToPlane_{ true };
bool usePointToLine_{ false };
bool useSymmetricPointToPlane_{ false };

bool useBoundConstraints_{ false };
bool useSixDofRegularization_{ false };
bool useThreeDofRegularization_{ false };
float regularizationWeight_{ 0.0f };
};
