#ifndef VEGASIK_H
#define VEGASIK_H

#include "urdf_import.h"
#include "math3d.h"

// Forward declarations
typedef struct VegasIK VegasIK;
typedef struct IKSolution IKSolution;

// Perturbation strategies
typedef enum {
    PERTURB_UNIFORM,     // Same std_dev for all joints
    PERTURB_POSITION,    // Larger for base joints, smaller for wrist
    PERTURB_WEIGHTED,    // Using joint weights
    PERTURB_ENDEFFECTOR, // Smaller base joints, larger wrist joints
    PERTURB_MULTISCALE   // New: Large variance followed by refinement
} PerturbStrategy;

// Structures
struct IKSolution {
    char** joint_names;
    double* joint_angles;
    int num_joints;
    double cost;
};

struct VegasIK {
    URDFRobot* robot;
    double learning_rate;
    double exploration_std;
    double min_exploration;
    double max_exploration;
    double* joint_weights;
    int num_joints;
    PerturbStrategy perturb_strategy;
    int verbose;
    double coarse_exploration;
    double fine_exploration;
    int refinement_steps;
    IKSolution* best_solution;  // Store best solution between calls
};

// Function declarations
VegasIK* vegas_ik_create(URDFRobot* robot);
void vegas_ik_destroy(VegasIK* solver);

// Solver operations
IKSolution* vegas_ik_solve(VegasIK* solver,
                          const Matrix4* target_pose,
                          const char** joint_names,
                          const double* initial_angles,
                          int num_joints,
                          int num_rollouts,
                          int num_iterations);

IKSolution* vegas_ik_get_rollout(VegasIK* solver,
                                const Matrix4* target_pose,
                                const char** joint_names,
                                const double* initial_angles,
                                int num_joints);

// Solution operations
void ik_solution_destroy(IKSolution* solution);

// Add function declarations
void vegas_ik_set_verbose(VegasIK* solver, int verbose);
void vegas_ik_set_strategy(VegasIK* solver, PerturbStrategy strategy);
void vegas_ik_set_seed(VegasIK* solver, unsigned int seed);

#endif // VEGASIK_H 