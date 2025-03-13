#include "urdf_import.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <urdf_file>\n", argv[0]);
        return 1;
    }
    
    const char* urdf_file = argv[1];
    printf("Loading URDF file: %s\n", urdf_file);
    
    URDFRobot* robot = urdf_robot_create(urdf_file);
    if (!robot) {
        fprintf(stderr, "Failed to load URDF file\n");
        return 1;
    }
    
    // Print robot structure and mesh information
    urdf_robot_print_tree(robot);
    
    urdf_robot_destroy(robot);
    return 0;
} 