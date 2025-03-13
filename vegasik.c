#include "vegasik.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <float.h>

// Helper functions
static void* malloc_safe(size_t size) {
    void* ptr = malloc(size);
    if (!ptr) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    memset(ptr, 0, size);
    return ptr;
}

static char* strdup_safe(const char* str) {
    if (!str) return NULL;
    char* dup = strdup(str);
    if (!dup) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    return dup;
}

// Solution functions
static IKSolution* ik_solution_create(int num_joints) {
    IKSolution* solution = (IKSolution*)malloc_safe(sizeof(IKSolution));
    solution->num_joints = num_joints;
    solution->joint_names = (char**)malloc_safe(num_joints * sizeof(char*));
    solution->joint_angles = (double*)malloc_safe(num_joints * sizeof(double));
    solution->cost = DBL_MAX;
    return solution;
}

void ik_solution_destroy(IKSolution* solution) {
    if (!solution) return;
    for (int i = 0; i < solution->num_joints; i++) {
        free(solution->joint_names[i]);
    }
    free(solution->joint_names);
    free(solution->joint_angles);
    free(solution);
}

// Solver functions
VegasIK* vegas_ik_create(URDFRobot* robot) {
    if (!robot) return NULL;
    
    // Seed random number generator with current time
    srand((unsigned int)time(NULL));
    
    VegasIK* solver = (VegasIK*)malloc_safe(sizeof(VegasIK));
    solver->robot = robot;
    solver->learning_rate = 0.1;
    solver->exploration_std = 0.05;
    solver->min_exploration = 0.001;
    solver->max_exploration = 0.08;
    solver->perturb_strategy = PERTURB_UNIFORM;  // Default strategy
    solver->verbose = 1;  // Enable logging by default
    
    // Count active joints
    solver->num_joints = robot->num_joints;
    solver->joint_weights = (double*)malloc_safe(solver->num_joints * sizeof(double));
    for (int i = 0; i < solver->num_joints; i++) {
        solver->joint_weights[i] = 1.0;
    }
    
    solver->coarse_exploration = 0.2;     // Larger initial steps for better exploration
    solver->fine_exploration = 0.002;     // Much finer steps for precise optimization
    solver->refinement_steps = 3;
    
    // Add best solution storage
    solver->best_solution = NULL;
    
    return solver;
}

void vegas_ik_destroy(VegasIK* solver) {
    if (!solver) return;
    if (solver->best_solution) {
        ik_solution_destroy(solver->best_solution);
    }
    free(solver->joint_weights);
    free(solver);
}

// Helper function to calculate rotation distance
static double rotation_distance(Matrix3 rot1, Matrix3 rot2) {
    // Convert difference to axis-angle representation
    Matrix3 rot_diff = matrix3_multiply(matrix3_transpose(rot1), rot2);
    
    // Get angle from trace: tr = 1 + 2cos(theta)
    double trace = rot_diff.data[0] + rot_diff.data[4] + rot_diff.data[8];
    double cos_theta = (trace - 1.0) / 2.0;
    
    // Clamp to valid range due to numerical errors
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    
    return acos(cos_theta);  // Return angle in radians
}

static double calculate_cost(URDFRobot* robot, const Matrix4* target_pose, VegasIK* solver) {
    // Get current end effector pose
    Matrix4 current_pose = urdf_robot_get_link_fk(robot, 
                                                 urdf_robot_get_end_effector_link(robot));
    
    // Extract positions
    Vector3 current_pos = {
        current_pose.data[12],
        current_pose.data[13],
        current_pose.data[14]
    };
    
    Vector3 target_pos = {
        target_pose->data[12],
        target_pose->data[13],
        target_pose->data[14]
    };
    
    // Calculate position error (Euclidean distance)
    Vector3 pos_diff = vector3_sub(current_pos, target_pos);
    double pos_error = vector3_norm(pos_diff);
    
    // Extract rotation matrices (top-left 3x3)
    Matrix3 current_rot = {
        .data = {
            current_pose.data[0], current_pose.data[1], current_pose.data[2],
            current_pose.data[4], current_pose.data[5], current_pose.data[6],
            current_pose.data[8], current_pose.data[9], current_pose.data[10]
        }
    };
    
    Matrix3 target_rot = {
        .data = {
            target_pose->data[0], target_pose->data[1], target_pose->data[2],
            target_pose->data[4], target_pose->data[5], target_pose->data[6],
            target_pose->data[8], target_pose->data[9], target_pose->data[10]
        }
    };

    // Extract axis vectors from rotation matrices
    Vector3 x1 = {current_rot.data[0], current_rot.data[3], current_rot.data[6]};  // Current X axis
    Vector3 y1 = {current_rot.data[1], current_rot.data[4], current_rot.data[7]};  // Current Y axis
    Vector3 z1 = {current_rot.data[2], current_rot.data[5], current_rot.data[8]};  // Current Z axis
    
    Vector3 x2 = {target_rot.data[0], target_rot.data[3], target_rot.data[6]};    // Target X axis
    Vector3 y2 = {target_rot.data[1], target_rot.data[4], target_rot.data[7]};    // Target Y axis
    Vector3 z2 = {target_rot.data[2], target_rot.data[5], target_rot.data[8]};    // Target Z axis
    
    // Calculate rotation error
    double rot_error = rotation_distance(current_rot, target_rot);
     
    // Calculate axis alignments
    double x_dot = vector3_dot(x1, x2);  // How well X axes align
    double y_dot = vector3_dot(y1, y2);  // How well Y axes align
    double z_dot = vector3_dot(z1, z2);  // How well Z axes align
    
    // Only print if verbose is enabled
    if (solver->verbose) {
        printf("\n Align: x=%.3f y=%.3f z=%.3f ", x_dot, y_dot, z_dot);
        printf("Costs: pos=%.3f rot=%.3f  ", 
               pos_error, rot_error);
    }
    
    return pos_error + 0.5f*rot_error;
}

// Random number utilities
static double random_normal(void) {
    // Box-Muller transform
    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX;
    return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
}

// Different perturbation strategies
static void perturb_uniform(double* angles, const double* base_angles, int num_joints, double std_dev) {
    for (int i = 0; i < num_joints; i++) {
        angles[i] = base_angles[i] + std_dev * random_normal();
    }
}

static void perturb_position_oriented(double* angles, const double* base_angles, int num_joints, double std_dev) {
    for (int i = 0; i < num_joints; i++) {
        // Base joints (0-2) get larger exploration for gross positioning
        // Wrist joints (3-5) get smaller exploration for fine orientation
        double joint_std = (i < 3) ? std_dev : std_dev * 0.3;
        angles[i] = base_angles[i] + joint_std * random_normal();
    }
}

static void perturb_weighted(double* angles, const double* base_angles, int num_joints, double std_dev, const double* weights) {
    for (int i = 0; i < num_joints; i++) {
        angles[i] = base_angles[i] + std_dev * weights[i] * random_normal();
    }
}

static void perturb_endeffector_oriented(double* angles, const double* base_angles, int num_joints, double std_dev) {
    for (int i = 0; i < num_joints; i++) {
        // Base joints (0-2) get smaller exploration for fine positioning
        // Wrist joints (3-5) get larger exploration for orientation search
        double joint_std = (i < 3) ? std_dev * 0.3 : std_dev * 1.5;
        angles[i] = base_angles[i] + joint_std * random_normal();
    }
}

// Modify multiscale perturbation to be simpler - just one scale at a time
static void perturb_multiscale(double* angles, const double* base_angles, int num_joints, double std_dev) {
    for (int i = 0; i < num_joints; i++) {
        angles[i] = base_angles[i] + std_dev * random_normal();
    }
}

// Main perturbation function that selects strategy
static void perturb_angles(double* angles, const double* base_angles, int num_joints, 
                          double std_dev, VegasIK* solver) {
    switch (solver->perturb_strategy) {
        case PERTURB_POSITION:
            perturb_position_oriented(angles, base_angles, num_joints, std_dev);
            break;
        case PERTURB_WEIGHTED:
            perturb_weighted(angles, base_angles, num_joints, std_dev, solver->joint_weights);
            break;
        case PERTURB_ENDEFFECTOR:
            perturb_endeffector_oriented(angles, base_angles, num_joints, std_dev);
            break;
        case PERTURB_MULTISCALE:
            perturb_multiscale(angles, base_angles, num_joints, std_dev);
            break;
        case PERTURB_UNIFORM:
        default:
            perturb_uniform(angles, base_angles, num_joints, std_dev);
            break;
    }
}

// Main solver functions
IKSolution* vegas_ik_get_rollout(VegasIK* solver,
                                const Matrix4* target_pose,
                                const char** joint_names,
                                const double* initial_angles,
                                int num_joints) {
    if (!solver || !target_pose || !joint_names || !initial_angles || num_joints <= 0) {
        return NULL;
    }
    
    // Create solution
    IKSolution* solution = ik_solution_create(num_joints);
    for (int i = 0; i < num_joints; i++) {
        solution->joint_names[i] = strdup_safe(joint_names[i]);
    }
    
    // Generate perturbed angles
    perturb_angles(solution->joint_angles, initial_angles, num_joints, solver->exploration_std, solver);
    
    // Apply angles and compute cost
    urdf_robot_set_joint_angles(solver->robot, joint_names, solution->joint_angles, num_joints);
    solution->cost = calculate_cost(solver->robot, target_pose, solver);
    
    return solution;
}

IKSolution* vegas_ik_solve(VegasIK* solver,
                          const Matrix4* target_pose,
                          const char** joint_names,
                          const double* initial_angles,
                          int num_joints,
                          int num_rollouts,
                          int num_iterations) {
    if (!solver || !target_pose || !joint_names || !initial_angles || 
        num_joints <= 0 || num_rollouts <= 0 || num_iterations <= 0) {
        return NULL;
    }
    
    // Split rollouts between phases
    int samples_per_phase = num_rollouts / 2;
    
    // Create and initialize best solution with initial angles
    IKSolution* best_solution = ik_solution_create(num_joints);
    for (int i = 0; i < num_joints; i++) {
        best_solution->joint_names[i] = strdup_safe(joint_names[i]);
        best_solution->joint_angles[i] = initial_angles[i];
    }
    
    // Calculate initial cost
    urdf_robot_set_joint_angles(solver->robot, joint_names, best_solution->joint_angles, num_joints);
    best_solution->cost = calculate_cost(solver->robot, target_pose, solver);
    
    // Storage for rollout results
    double* current_angles = (double*)malloc_safe(num_joints * sizeof(double));
    
    // For each iteration
    for (int iter = 0; iter < num_iterations; iter++) {
        // Coarse phase
        if (solver->verbose) {
            printf("\nStarting coarse phase with std_dev=%.4f, current best cost=%.4f\n", 
                   solver->coarse_exploration, best_solution->cost);
        }
        
        for (int i = 0; i < samples_per_phase; i++) {
            perturb_multiscale(current_angles, best_solution->joint_angles, num_joints, solver->coarse_exploration);
            urdf_robot_set_joint_angles(solver->robot, joint_names, current_angles, num_joints);
            double cost = calculate_cost(solver->robot, target_pose, solver);
            
            if (cost < best_solution->cost) {
                memcpy(best_solution->joint_angles, current_angles, num_joints * sizeof(double));
                best_solution->cost = cost;
            }
        }
        
        // Fine phase
        if (solver->verbose) {
            printf("Starting fine phase with std_dev=%.4f, current best cost=%.4f\n", 
                   solver->fine_exploration, best_solution->cost);
        }
        
        for (int i = 0; i < samples_per_phase; i++) {
            perturb_multiscale(current_angles, best_solution->joint_angles, num_joints, solver->fine_exploration);
            urdf_robot_set_joint_angles(solver->robot, joint_names, current_angles, num_joints);
            double cost = calculate_cost(solver->robot, target_pose, solver);
            
            if (cost < best_solution->cost) {
                memcpy(best_solution->joint_angles, current_angles, num_joints * sizeof(double));
                best_solution->cost = cost;
            }
        }
        
        if (solver->verbose) {
            printf("Iteration %d completed, best cost=%.4f\n", iter + 1, best_solution->cost);
        }
    }
    
    free(current_angles);
    
    // Store best solution for next time
    if (solver->best_solution) {
        ik_solution_destroy(solver->best_solution);
    }
    solver->best_solution = ik_solution_create(num_joints);
    for (int i = 0; i < num_joints; i++) {
        solver->best_solution->joint_names[i] = strdup_safe(joint_names[i]);
        solver->best_solution->joint_angles[i] = best_solution->joint_angles[i];
    }
    solver->best_solution->cost = best_solution->cost;
    
    return best_solution;
}

void vegas_ik_set_verbose(VegasIK* solver, int verbose) {
    if (solver) {
        solver->verbose = verbose;
    }
}

void vegas_ik_set_strategy(VegasIK* solver, PerturbStrategy strategy) {
    if (solver) {
        solver->perturb_strategy = strategy;
    }
}

void vegas_ik_set_seed(VegasIK* solver, unsigned int seed) {
    if (solver) {
        srand(seed);
    }
}