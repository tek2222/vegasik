// Random number generation using xoshiro256**
struct xoshiro256ss_state {
    ulong s[4];
};

static ulong rotl(const ulong x, int k) {
    return (x << k) | (x >> (64 - k));
}

double xoshiro256ss(struct xoshiro256ss_state* state) {
    const ulong result = rotl(state->s[1] * 5, 7) * 9;
    const ulong t = state->s[1] << 17;
    
    state->s[2] ^= state->s[0];
    state->s[3] ^= state->s[1];
    state->s[1] ^= state->s[2];
    state->s[0] ^= state->s[3];
    
    state->s[2] ^= t;
    state->s[3] = rotl(state->s[3], 45);
    
    return (double)result / (double)0xFFFFFFFFFFFFFFFFUL;
}

// Box-Muller transform for normal distribution
double random_normal(struct xoshiro256ss_state* state) {
    double u1 = xoshiro256ss(state);
    double u2 = xoshiro256ss(state);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

// Matrix operations
void matrix_multiply(const double* a, const double* b, double* result) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) {
                sum += a[i + k*4] * b[k + j*4];
            }
            result[i + j*4] = sum;
        }
    }
}

void create_rotation_matrix(double angle, const double* axis, double* matrix) {
    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    double x = axis[0], y = axis[1], z = axis[2];

    matrix[0] = t*x*x + c;    matrix[4] = t*x*y - s*z;  matrix[8] = t*x*z + s*y;  matrix[12] = 0;
    matrix[1] = t*x*y + s*z;  matrix[5] = t*y*y + c;    matrix[9] = t*y*z - s*x;  matrix[13] = 0;
    matrix[2] = t*x*z - s*y;  matrix[6] = t*y*z + s*x;  matrix[10] = t*z*z + c;   matrix[14] = 0;
    matrix[3] = 0;            matrix[7] = 0;            matrix[11] = 0;           matrix[15] = 1;
}

void create_translation_matrix(const double* trans, double* matrix) {
    matrix[0] = 1;  matrix[4] = 0;  matrix[8] = 0;   matrix[12] = trans[0];
    matrix[1] = 0;  matrix[5] = 1;  matrix[9] = 0;   matrix[13] = trans[1];
    matrix[2] = 0;  matrix[6] = 0;  matrix[10] = 1;  matrix[14] = trans[2];
    matrix[3] = 0;  matrix[7] = 0;  matrix[11] = 0;  matrix[15] = 1;
}

// Forward kinematics calculation
void calculate_fk(
    const double* angles,           // Joint angles for this rollout
    const __global GPUJoint* joints,// Joint data array
    int num_joints,                 // Number of joints
    __global double* joint_poses    // Output joint poses [num_joints * 16]
) {
    double identity[16] = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };
    
    for (int i = 0; i < num_joints; i++) {
        double local_transform[16];
        double rot_matrix[16];
        double trans_matrix[16];
        
        if (joints[i].joint_type == 1) {  // Revolute
            create_rotation_matrix(angles[i], joints[i].axis, rot_matrix);
        } else {
            for (int j = 0; j < 16; j++) rot_matrix[j] = identity[j];
        }
        
        create_translation_matrix(joints[i].origin, trans_matrix);
        matrix_multiply(trans_matrix, rot_matrix, local_transform);
        
        if (joints[i].parent_idx >= 0) {
            matrix_multiply(&joint_poses[joints[i].parent_idx * 16], 
                          local_transform, 
                          &joint_poses[i * 16]);
        } else {
            for (int j = 0; j < 16; j++) {
                joint_poses[i * 16 + j] = local_transform[j];
            }
        }
    }
}

// Cost calculation
double calculate_cost(const double* current_pose, const double* target_pose) {
    // Position error
    double dx = current_pose[12] - target_pose[12];
    double dy = current_pose[13] - target_pose[13];
    double dz = current_pose[14] - target_pose[14];
    double pos_error = sqrt(dx*dx + dy*dy + dz*dz);
    
    // Rotation error (using dot products between axes)
    double x_dot = fabs(current_pose[0]*target_pose[0] + 
                       current_pose[4]*target_pose[4] + 
                       current_pose[8]*target_pose[8]);
    double y_dot = fabs(current_pose[1]*target_pose[1] + 
                       current_pose[5]*target_pose[5] + 
                       current_pose[9]*target_pose[9]);
    double z_dot = fabs(current_pose[2]*target_pose[2] + 
                       current_pose[6]*target_pose[6] + 
                       current_pose[10]*target_pose[10]);
                       
    double rot_error = (3.0 - (x_dot + y_dot + z_dot));
    
    return pos_error + 5.0 * rot_error;
}

__kernel void generate_rollouts(
    __global double* angles,          // [num_rollouts * num_joints]
    __global double* costs,           // [num_rollouts]
    __global const double* base_angles,// [num_joints]
    __global const double* target_pose,// [16]
    __global const GPUJoint* joints,  // [num_joints]
    __global double* joint_poses,     // [num_rollouts * num_joints * 16]
    __global struct xoshiro256ss_state* random_states, // [num_rollouts]
    const int num_joints,
    const double std_dev,
    const int perturb_strategy
) {
    int rollout_id = get_global_id(0);
    struct xoshiro256ss_state* state = &random_states[rollout_id];
    
    // Generate perturbed angles
    for (int j = 0; j < num_joints; j++) {
        double joint_std = std_dev;
        if (perturb_strategy == 1) { // PERTURB_POSITION
            joint_std = (j < 3) ? std_dev : std_dev * 0.3;
        } else if (perturb_strategy == 3) { // PERTURB_ENDEFFECTOR
            joint_std = (j < 3) ? std_dev * 0.3 : std_dev * 1.5;
        }
        
        angles[rollout_id * num_joints + j] = 
            base_angles[j] + joint_std * random_normal(state);
    }
    
    // Calculate FK for this rollout
    calculate_fk(&angles[rollout_id * num_joints],
                joints,
                num_joints,
                &joint_poses[rollout_id * num_joints * 16]);
    
    // End effector pose is the last joint's transform
    const double* current_pose = &joint_poses[rollout_id * num_joints * 16 + (num_joints-1) * 16];
    
    // Calculate cost
    costs[rollout_id] = calculate_cost(current_pose, target_pose);
} 