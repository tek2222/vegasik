#ifndef URDF_IMPORT_H
#define URDF_IMPORT_H

#include "math3d.h"
#include "draw_stl.h"
#include <libxml/parser.h>
#include <libxml/tree.h>

// Forward declarations
struct URDFRobot;
struct URDFJoint;

// Type aliases
typedef struct URDFRobot URDFRobot;
typedef struct URDFJoint URDFJoint;

// Structure definitions
typedef struct {
    char* name;
    char* parent_name;
    Matrix4 transform;
    char** visual_mesh_files;     // Full resolved paths to visual meshes
    int num_visual_meshes;
    char** collision_mesh_files;   // Full resolved paths to collision meshes
    int num_collision_meshes;
} URDFLink;

struct URDFJoint {
    char* name;
    int type;           // 0=fixed, 1=revolute, 2=prismatic
    char* parent_link;  // Name of parent link
    char* child_link;   // Name of child link
    Vector3 origin;     // Position relative to parent
    Vector3 axis;       // Rotation axis
    double angle;       // Current joint angle/position
};

struct URDFRobot {
    URDFLink* links;
    int num_links;
    URDFJoint* joints;
    int num_joints;
};

typedef struct {
    STLMesh** meshes;
    char** names;
    int count;
} URDFMeshes;

// Robot creation/destruction
URDFRobot* urdf_robot_create(const char* filename);
void urdf_robot_destroy(URDFRobot* robot);

// Joint operations
int urdf_robot_get_num_joints(URDFRobot* robot);
const char* urdf_robot_get_joint_name(URDFRobot* robot, int index);
double urdf_robot_get_joint_angle(URDFRobot* robot, const char* joint_name);
void urdf_robot_set_joint_angles(URDFRobot* robot, const char** joint_names, const double* angles, int num_joints);

// Link operations
int urdf_robot_get_num_links(URDFRobot* robot);
const char* urdf_robot_get_link_name(URDFRobot* robot, int index);
const char* urdf_robot_get_link_parent_name(URDFRobot* robot, const char* link_name);
const char* urdf_robot_get_end_effector_link(URDFRobot* robot);

// Kinematics
Matrix4 urdf_robot_get_link_fk(URDFRobot* robot, const char* link_name);
Matrix4 joint_get_transform(URDFJoint* joint);

// Add these with the other function declarations
const char* urdf_robot_get_joint_parent_link(URDFRobot* robot, const char* joint_name);
const char* urdf_robot_get_joint_child_link(URDFRobot* robot, const char* joint_name);
int urdf_robot_get_joint_type(URDFRobot* robot, const char* joint_name);

// Add these function declarations
int urdf_robot_get_link_num_meshes(URDFRobot* robot, const char* link_name);
const char* urdf_robot_get_link_mesh_filename(URDFRobot* robot, const char* link_name, int mesh_index);

// Add these function declarations
void urdf_robot_print_tree(URDFRobot* robot);
void urdf_robot_print_mesh_info(URDFRobot* robot);

// Add these function declarations
const char* urdf_robot_get_link_visual_mesh(URDFRobot* robot, const char* link_name, int mesh_index);
const char* urdf_robot_get_link_collision_mesh(URDFRobot* robot, const char* link_name, int mesh_index);

// Add these function declarations
const char* urdf_robot_get_base_link(URDFRobot* robot);
int urdf_robot_get_link_depth(URDFRobot* robot, const char* link_name);
URDFMeshes* urdf_robot_load_meshes(URDFRobot* robot, float scale);
void urdf_meshes_destroy(URDFMeshes* meshes);

#endif // URDF_IMPORT_H 