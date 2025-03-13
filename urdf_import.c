#include "urdf_import.h"
#include "draw_stl.h"  // Add this for STLMesh functions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  // Add this for access() function
#include <math.h>
#include <libxml/parser.h>
#include <libxml/tree.h>



// Helper functions
static void* malloc_safe(size_t size) {
    void* ptr = malloc(size);
    if (!ptr) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    return ptr;
}

static char* strdup_safe(const char* str) {
    char* dup = strdup(str);
    if (!dup) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    return dup;
}

// Add this helper function
static void print_link_world_coordinates(URDFRobot* robot) {
    printf("\nLink world coordinates after loading:\n");
    printf("=====================================\n");
    
    for (int i = 0; i < robot->num_links; i++) {
        const char* link_name = robot->links[i].name;
        Matrix4 transform = urdf_robot_get_link_fk(robot, link_name);
        
        Vector3 pos = {
            transform.data[3],   // x
            transform.data[7],   // y
            transform.data[11]   // z
        };
        
        printf("Link: %s\n", link_name);
        printf("  Position: (%.3f, %.3f, %.3f)\n", pos.x, pos.y, pos.z);
        printf("  Parent: %s\n", robot->links[i].parent_name ? robot->links[i].parent_name : "none");
        
        // Find and print joint info
        for (int j = 0; j < robot->num_joints; j++) {
            if (robot->joints[j].child_link && !strcmp(robot->joints[j].child_link, link_name)) {
                printf("  Connected by joint: %s (type: %s)\n", 
                       robot->joints[j].name,
                       robot->joints[j].type == 1 ? "revolute" : robot->joints[j].type == 2 ? "prismatic" : "fixed");
                printf("  Joint origin: (%.3f, %.3f, %.3f)\n",
                       robot->joints[j].origin.x,
                       robot->joints[j].origin.y,
                       robot->joints[j].origin.z);
                printf("  Joint axis: (%.3f, %.3f, %.3f)\n",
                       robot->joints[j].axis.x,
                       robot->joints[j].axis.y,
                       robot->joints[j].axis.z);
                break;
            }
        }
        printf("\n");
    }
    printf("=====================================\n\n");
}

// Helper function to get link depth
static int get_link_depth(URDFRobot* robot, const char* link_name) {
    int depth = 0;
    const char* current = link_name;
    
    while (current) {
        current = urdf_robot_get_link_parent_name(robot, current);
        if (current) depth++;
    }
    
    return depth;
}

static xmlNode* find_child_element(xmlNode* parent, const char* name) {
    xmlNode* child = parent->children;
    while (child) {
        if (child->type == XML_ELEMENT_NODE && 
            xmlStrcmp(child->name, (const xmlChar*)name) == 0) {
            return child;
        }
        child = child->next;
    }
    return NULL;
}

// Add this function to resolve package paths
static char* resolve_package_path(const char* package_path, const char* urdf_filepath) {
    if (!package_path || strncmp(package_path, "package://", 10) != 0) {
        return strdup_safe(package_path);
    }
    
    // Skip "package://"
    const char* path = package_path + 10;
    
    // Get package name (everything up to first '/')
    const char* slash = strchr(path, '/');
    if (!slash) return strdup_safe(package_path);
    
    // Get URDF directory
    char* urdf_dir = strdup_safe(urdf_filepath);
    char* last_slash = strrchr(urdf_dir, '/');
    if (last_slash) {
        *last_slash = '\0';
    }
    
    // Try to find package in these locations:
    const char* search_paths[] = {
        urdf_dir,                    // URDF file directory
        ".",                         // Current directory
        "../meshes",                 // Common ROS layout
        "../h1_description/meshes",  // Package-specific path
        "../../h1_description/meshes", // Up one more level
    };
    
    char* resolved_path = NULL;
    for (size_t i = 0; i < sizeof(search_paths)/sizeof(search_paths[0]); i++) {
        if (!search_paths[i]) continue;
        
        // Get the base filename without extension
        const char* mesh_filename = strrchr(path, '/') + 1;
        char* base_name = strdup_safe(mesh_filename);
        char* dot = strrchr(base_name, '.');
        if (dot) *dot = '\0';
        
        // Try different extensions
        const char* extensions[] = {
            ".stl", ".STL",     // STL variants
            ".dae", ".DAE",     // COLLADA variants
            ".obj", ".OBJ"      // OBJ variants
        };
        
        for (size_t j = 0; j < sizeof(extensions)/sizeof(extensions[0]); j++) {
            char test_path[1024];
            snprintf(test_path, sizeof(test_path), "%s/%s%s", 
                    search_paths[i], base_name, extensions[j]);
            
            printf("Trying path: %s\n", test_path);
            if (access(test_path, F_OK) == 0) {
                resolved_path = strdup_safe(test_path);
                free(base_name);
                goto found;  // Break out of both loops
            }
        }
        
        free(base_name);
    }
    
found:
    free(urdf_dir);
    return resolved_path ? resolved_path : strdup_safe(package_path);
}

static char* try_find_stl_file(const char* urdf_path, const char* mesh_filename) {
    // Get URDF directory
    char* urdf_dir = strdup_safe(urdf_path);
    char* last_slash = strrchr(urdf_dir, '/');
    if (last_slash) {
        *last_slash = '\0';
    }
    
    // Get base filename without extension
    char* base_name = strdup_safe(mesh_filename);
    char* dot = strrchr(base_name, '.');
    if (dot) *dot = '\0';  // Remove extension
    
    // Try common mesh locations relative to URDF path
    const char* mesh_dirs[] = {
        "meshes",           // <urdf_dir>/meshes/
        "../meshes",        // <urdf_dir>/../meshes/
        ".",               // <urdf_dir>/
        "..",              // <urdf_dir>/../
        "../h1_description/meshes"  // Common ROS package layout
    };
    
    // Try both .stl and .STL extensions in each location
    char test_path[1024];
    char* found_path = NULL;
    
    printf("\nLooking for STL file for mesh '%s':\n", base_name);
    
    for (size_t i = 0; i < sizeof(mesh_dirs)/sizeof(mesh_dirs[0]); i++) {
        // Try lowercase .stl
        snprintf(test_path, sizeof(test_path), "%s/%s/%s.stl", 
                urdf_dir, mesh_dirs[i], base_name);
        printf("  Trying: %s ... ", test_path);
        if (access(test_path, F_OK) == 0) {
            printf("FOUND!\n");
            found_path = strdup_safe(test_path);
            break;
        } else {
            printf("not found\n");
        }
        
        // Try uppercase .STL
        snprintf(test_path, sizeof(test_path), "%s/%s/%s.STL", 
                urdf_dir, mesh_dirs[i], base_name);
        printf("  Trying: %s ... ", test_path);
        if (access(test_path, F_OK) == 0) {
            printf("FOUND!\n");
            found_path = strdup_safe(test_path);
            break;
        } else {
            printf("not found\n");
        }
    }
    
    if (!found_path) {
        printf("  No STL file found for mesh '%s'\n", base_name);
    }
    
    free(base_name);
    free(urdf_dir);
    return found_path;
}

static const char* extract_base_name(const char* filename) {
    const char* last_slash = strrchr(filename, '/');
    const char* base_name = last_slash ? last_slash + 1 : filename;
    char* dot = strrchr(base_name, '.');
    if (dot) {
        *dot = '\0';  // Remove the extension
    }
    return base_name;
}

static char* load_mesh(const char* urdf_path, const char* mesh_filename, const char* mesh_type) {
    // Extract the base name from the mesh filename
    const char* base_name = extract_base_name(mesh_filename);
    
    // Try to find the STL file
    char* stl_path = try_find_stl_file(urdf_path, base_name);
    if (stl_path) {
        printf("Found %s mesh for link: %s\n", mesh_type, stl_path);
        return stl_path;
    }
    
    printf("No %s mesh found for link: %s\n", mesh_type, base_name);
    return NULL;
}

static void parse_link_element(xmlNode* link_node, URDFLink* link, const char* urdf_path) {
    xmlNode* child = link_node->children;
    
    while (child) {
        if (child->type == XML_ELEMENT_NODE) {
            if (xmlStrcmp(child->name, (const xmlChar*)"visual") == 0) {
                xmlNode* geometry = find_child_element(child, "geometry");
                if (geometry) {
                    xmlNode* mesh = find_child_element(geometry, "mesh");
                    if (mesh) {
                        xmlChar* filename = xmlGetProp(mesh, (const xmlChar*)"filename");
                        if (filename) {
                            char* stl_path = load_mesh(urdf_path, (char*)filename, "visual");
                            if (stl_path) {
                                link->num_visual_meshes++;
                                link->visual_mesh_files = realloc(link->visual_mesh_files, 
                                                                link->num_visual_meshes * sizeof(char*));
                                link->visual_mesh_files[link->num_visual_meshes - 1] = stl_path;
                            }
                            xmlFree(filename);
                        }
                    }
                }
            } else if (xmlStrcmp(child->name, (const xmlChar*)"collision") == 0) {
                xmlNode* geometry = find_child_element(child, "geometry");
                if (geometry) {
                    xmlNode* mesh = find_child_element(geometry, "mesh");
                    if (mesh) {
                        xmlChar* filename = xmlGetProp(mesh, (const xmlChar*)"filename");
                        if (filename) {
                            char* stl_path = load_mesh(urdf_path, (char*)filename, "collision");
                            if (stl_path) {
                                link->num_collision_meshes++;
                                link->collision_mesh_files = realloc(link->collision_mesh_files, 
                                                                   link->num_collision_meshes * sizeof(char*));
                                link->collision_mesh_files[link->num_collision_meshes - 1] = stl_path;
                            }
                            xmlFree(filename);
                        }
                    }
                }
            }
        }
        child = child->next;
    }
}

// Implementation of URDFRobot functions
URDFRobot* urdf_robot_create(const char* filename) {
    printf("Creating robot from file: %s\n", filename);
    
    URDFRobot* robot = (URDFRobot*)malloc_safe(sizeof(URDFRobot));
    memset(robot, 0, sizeof(URDFRobot));  // Initialize all fields to zero
    
    // Parse the XML file
    xmlDoc* doc = xmlReadFile(filename, NULL, 0);
    if (!doc) {
        fprintf(stderr, "Failed to parse URDF file: %s\n", filename);
        free(robot);
        return NULL;
    }
    printf("Successfully parsed XML file\n");
    
    xmlNode* root = xmlDocGetRootElement(doc);
    if (!root || xmlStrcmp(root->name, (const xmlChar*)"robot")) {
        fprintf(stderr, "Root element is not 'robot'\n");
        xmlFreeDoc(doc);
        free(robot);
        return NULL;
    }
    printf("Found robot root element\n");
    
    // First pass: count links and joints
    xmlNode* cur = root->children;
    while (cur) {
        if (cur->type == XML_ELEMENT_NODE) {
            if (!xmlStrcmp(cur->name, (const xmlChar*)"link")) {
                robot->num_links++;
                printf("Found link: %s\n", (char*)xmlGetProp(cur, (const xmlChar*)"name"));
            } else if (!xmlStrcmp(cur->name, (const xmlChar*)"joint")) {
                robot->num_joints++;
                printf("Found joint: %s\n", (char*)xmlGetProp(cur, (const xmlChar*)"name"));
            }
        }
        cur = cur->next;
    }
    printf("First pass complete: found %d links and %d joints\n", 
           robot->num_links, robot->num_joints);
    
    // Allocate arrays
    if (robot->num_links > 0) {
        robot->links = malloc_safe(robot->num_links * sizeof(URDFLink));
        memset(robot->links, 0, robot->num_links * sizeof(URDFLink));
    }
    if (robot->num_joints > 0) {
        robot->joints = malloc_safe(robot->num_joints * sizeof(URDFJoint));
        memset(robot->joints, 0, robot->num_joints * sizeof(URDFJoint));
    }
    
    // Second pass: fill in the data
    int link_idx = 0;
    int joint_idx = 0;
    cur = root->children;
    
    while (cur) {
        if (cur->type == XML_ELEMENT_NODE) {
            if (!xmlStrcmp(cur->name, (const xmlChar*)"link")) {
                xmlChar* name = xmlGetProp(cur, (const xmlChar*)"name");
                if (name) {
                    printf("Processing link: %s\n", (char*)name);
                    robot->links[link_idx].name = strdup_safe((char*)name);
                    robot->links[link_idx].parent_name = NULL;
                    robot->links[link_idx].num_visual_meshes = 0;
                    robot->links[link_idx].num_collision_meshes = 0;
                    robot->links[link_idx].visual_mesh_files = NULL;
                    robot->links[link_idx].collision_mesh_files = NULL;
                    
                    // Parse visual and collision elements
                    parse_link_element(cur, &robot->links[link_idx], filename);
                    
                    xmlFree(name);
                    link_idx++;
                }
            } else if (!xmlStrcmp(cur->name, (const xmlChar*)"joint")) {
                xmlChar* name = xmlGetProp(cur, (const xmlChar*)"name");
                xmlChar* type = xmlGetProp(cur, (const xmlChar*)"type");
                xmlChar* parent = NULL;
                xmlChar* child = NULL;
                Vector3 origin = {0};
                Vector3 axis = {0, 0, 1};  // Default to Z-axis rotation
                
                if (!name) continue;  // Skip if no name
                
                // Find parent, child, and origin
                xmlNode* joint_child = cur->children;
                while (joint_child) {
                    if (joint_child->type == XML_ELEMENT_NODE) {
                        if (!xmlStrcmp(joint_child->name, (const xmlChar*)"origin")) {
                            xmlChar* xyz = xmlGetProp(joint_child, (const xmlChar*)"xyz");
                            if (xyz) {
                                sscanf((char*)xyz, "%lf %lf %lf", 
                                       &origin.x, &origin.y, &origin.z);
                                xmlFree(xyz);
                            }
                        } else if (!xmlStrcmp(joint_child->name, (const xmlChar*)"parent")) {
                            parent = xmlGetProp(joint_child, (const xmlChar*)"link");
                        } else if (!xmlStrcmp(joint_child->name, (const xmlChar*)"child")) {
                            child = xmlGetProp(joint_child, (const xmlChar*)"link");
                        } else if (!xmlStrcmp(joint_child->name, (const xmlChar*)"axis")) {
                            xmlChar* xyz = xmlGetProp(joint_child, (const xmlChar*)"xyz");
                            if (xyz) {
                                sscanf((char*)xyz, "%lf %lf %lf", 
                                       &axis.x, &axis.y, &axis.z);
                                xmlFree(xyz);
                            }
                        }
                    }
                    joint_child = joint_child->next;
                }
                
                // Process joint only if we have all required properties
                if (name && parent && child) {
                    printf("Processing joint: %s (parent: %s, child: %s)\n", 
                           (char*)name, (char*)parent, (char*)child);
                    
                    // Find parent and child links
                    URDFLink* parent_link = NULL;
                    URDFLink* child_link = NULL;
                    
                    // First find both links
                    for (int i = 0; i < robot->num_links; i++) {
                        if (robot->links[i].name && !strcmp(robot->links[i].name, (char*)parent)) {
                            parent_link = &robot->links[i];
                            printf("  Found parent link: %s\n", robot->links[i].name);
                        }
                        if (robot->links[i].name && !strcmp(robot->links[i].name, (char*)child)) {
                            child_link = &robot->links[i];
                            printf("  Found child link: %s\n", robot->links[i].name);
                        }
                    }
                    
                    if (parent_link && child_link) {
                        // Store joint info
                        robot->joints[joint_idx].name = strdup_safe((char*)name);
                        if (xmlStrcmp(type, (const xmlChar*)"revolute") == 0) {
                            robot->joints[joint_idx].type = 1;
                        } else if (xmlStrcmp(type, (const xmlChar*)"prismatic") == 0) {
                            robot->joints[joint_idx].type = 2;
                        } else {
                            robot->joints[joint_idx].type = 0;  // fixed
                        }
                        robot->joints[joint_idx].angle = 0.0;
                        robot->joints[joint_idx].origin = origin;
                        robot->joints[joint_idx].axis = axis;
                        robot->joints[joint_idx].parent_link = strdup_safe((char*)parent);
                        robot->joints[joint_idx].child_link = strdup_safe((char*)child);
                        
                        // Set the parent link directly from the joint definition
                        child_link->parent_name = parent_link->name;
                        
                        printf("  Set %s as parent of %s\n", parent_link->name, child_link->name);
                        joint_idx++;
                    } else {
                        printf("Warning: Could not find both parent and child links\n");
                    }
                } else {
                    printf("Warning: Skipping joint with missing properties\n");
                }
                
                // Clean up XML properties
                if (name) xmlFree(name);
                if (parent) xmlFree(parent);
                if (child) xmlFree(child);
                if (type) xmlFree(type);
            }
        }
        cur = cur->next;
    }
    
    xmlFreeDoc(doc);
    xmlCleanupParser();
    
    printf("Successfully loaded robot with %d links and %d joints\n", 
           robot->num_links, robot->num_joints);
           
    // Print world coordinates of all links
    print_link_world_coordinates(robot);
    
    return robot;
}

void urdf_robot_destroy(URDFRobot* robot) {
    if (!robot) return;
    // Clean up memory
    free(robot->links);
    free(robot->joints);
    free(robot);
}

int urdf_robot_get_num_joints(URDFRobot* robot) {
    return robot ? robot->num_joints : 0;
}

int urdf_robot_get_num_links(URDFRobot* robot) {
    return robot ? robot->num_links : 0;
}

const char* urdf_robot_get_joint_name(URDFRobot* robot, int index) {
    if (!robot || index < 0 || index >= robot->num_joints) return NULL;
    return robot->joints[index].name;
}

const char* urdf_robot_get_link_name(URDFRobot* robot, int index) {
    if (!robot || index < 0 || index >= robot->num_links) return NULL;
    return robot->links[index].name;
}

double urdf_robot_get_joint_angle(URDFRobot* robot, const char* joint_name) {
    if (!robot || !joint_name) return 0.0;
    
    for (int i = 0; i < robot->num_joints; i++) {
        if (strcmp(robot->joints[i].name, joint_name) == 0) {
            return robot->joints[i].angle;
        }
    }
    return 0.0;
}

void urdf_robot_set_joint_angles(URDFRobot* robot, const char** joint_names, 
                                const double* angles, int num_joints) {
    if (!robot || !joint_names || !angles || num_joints <= 0) return;
    
    for (int i = 0; i < num_joints; i++) {
        for (int j = 0; j < robot->num_joints; j++) {
            if (strcmp(robot->joints[j].name, joint_names[i]) == 0) {
                robot->joints[j].angle = angles[i];
                break;
            }
        }
    }
}

Matrix4 urdf_robot_get_link_fk(URDFRobot* robot, const char* link_name) {
    if (!robot || !link_name) return matrix4_identity();
    
    // Find the link
    URDFLink* link = NULL;
    for (int i = 0; i < robot->num_links; i++) {
        if (!strcmp(robot->links[i].name, link_name)) {
            link = &robot->links[i];
            break;
        }
    }
    if (!link) return matrix4_identity();
    
    // Find the joint that has this link as its child
    URDFJoint* joint = NULL;
    for (int i = 0; i < robot->num_joints; i++) {
        if (strcmp(robot->joints[i].child_link, link->name) == 0) {
            joint = &robot->joints[i];
            break;
        }
    }
    
    // If no joint found, this might be the base link
    if (!joint) return matrix4_identity();
    
    // Get parent transform recursively
    Matrix4 parent_transform = urdf_robot_get_link_fk(robot, joint->parent_link);
    
    // Create transform for this joint's origin
    Matrix4 translation = matrix4_translation(
        joint->origin.x,
        joint->origin.y,
        joint->origin.z
    );
    
    // Create rotation for joint angle around joint axis
    Matrix3 rot = matrix3_from_axis_angle(joint->axis, joint->angle);
    Matrix4 rotation = matrix4_from_matrix3_vector3(rot, (Vector3){0,0,0});
    
    // Combine transforms: parent * translation * rotation
    Matrix4 local = matrix4_multiply(translation, rotation);
    return matrix4_multiply(parent_transform, local);
}

const char* urdf_robot_get_end_effector_link(URDFRobot* robot) {
    int max_depth = -1;
    const char* deepest_tool = NULL;
    
    // Find deepest link containing "tool"
    for (int i = 0; i < robot->num_links; i++) {
        const char* link_name = robot->links[i].name;
        if (strstr(link_name, "tool") != NULL) {  // Check if name contains "tool"
            int depth = get_link_depth(robot, link_name);
            if (depth > max_depth) {
                max_depth = depth;
                deepest_tool = link_name;
            }
        }
    }
    
    // If no tool link found, use deepest link
    if (!deepest_tool) {
        max_depth = -1;
        for (int i = 0; i < robot->num_links; i++) {
            const char* link_name = robot->links[i].name;
            int depth = get_link_depth(robot, link_name);
            if (depth > max_depth) {
                max_depth = depth;
                deepest_tool = link_name;
            }
        }
    }
    
    return deepest_tool;
}

// Implementation of the link parent name function
const char* urdf_robot_get_link_parent_name(URDFRobot* robot, const char* link_name) {
    if (!robot || !link_name) return NULL;
    
    // Find the link in the robot's links array
    for (int i = 0; i < robot->num_links; i++) {
        if (strcmp(robot->links[i].name, link_name) == 0) {
            // If link found, return its parent's name
            if (robot->links[i].parent_name) {
                return robot->links[i].parent_name;
            }
            break;
        }
    }
    return NULL;  // Link not found or no parent
}

const char* urdf_robot_get_joint_parent_link(URDFRobot* robot, const char* joint_name) {
    if (!robot || !joint_name) return NULL;
    
    for (int i = 0; i < robot->num_joints; i++) {
        URDFJoint* joint = &robot->joints[i];
        if (joint->name && strcmp(joint->name, joint_name) == 0) {
            // Return parent link name if parent exists
            return joint->parent_link;
        }
    }
    return NULL;
}

const char* urdf_robot_get_joint_child_link(URDFRobot* robot, const char* joint_name) {
    if (!robot || !joint_name) return NULL;
    
    for (int i = 0; i < robot->num_joints; i++) {
        URDFJoint* joint = &robot->joints[i];
        if (joint->name && strcmp(joint->name, joint_name) == 0) {
            // Return child link name if child exists
            return joint->child_link;
        }
    }
    return NULL;
}

int urdf_robot_get_joint_type(URDFRobot* robot, const char* joint_name) {
    if (!robot || !joint_name) return 0;
    
    for (int i = 0; i < robot->num_joints; i++) {
        URDFJoint* joint = &robot->joints[i];
        if (joint->name && strcmp(joint->name, joint_name) == 0) {
            return joint->type;  // Already an int, no need for string comparison
        }
    }
    return 0;
}

int urdf_robot_get_link_num_meshes(URDFRobot* robot, const char* link_name) {
    if (!robot || !link_name) return 0;
    
    // Find the link
    URDFLink* link = NULL;  // Changed type to URDFLink
    for (int i = 0; i < robot->num_links; i++) {
        if (strcmp(robot->links[i].name, link_name) == 0) {
            link = &robot->links[i];
            break;
        }
    }
    
    if (!link) return 0;
    return link->num_visual_meshes + link->num_collision_meshes;
}

const char* urdf_robot_get_link_visual_mesh(URDFRobot* robot, const char* link_name, int mesh_index) {
    if (!robot || !link_name) return NULL;
    
    // Find the link
    for (int i = 0; i < robot->num_links; i++) {
        if (strcmp(robot->links[i].name, link_name) == 0) {
            if (mesh_index >= 0 && mesh_index < robot->links[i].num_visual_meshes) {
                return robot->links[i].visual_mesh_files[mesh_index];
            }
            break;
        }
    }
    return NULL;
}

const char* urdf_robot_get_link_collision_mesh(URDFRobot* robot, const char* link_name, int mesh_index) {
    if (!robot || !link_name) return NULL;
    
    // Find the link
    for (int i = 0; i < robot->num_links; i++) {
        if (strcmp(robot->links[i].name, link_name) == 0) {
            if (mesh_index >= 0 && mesh_index < robot->links[i].num_collision_meshes) {
                return robot->links[i].collision_mesh_files[mesh_index];
            }
            break;
        }
    }
    return NULL;
}

// Update urdf_robot_get_link_mesh_filename to use these functions
const char* urdf_robot_get_link_mesh_filename(URDFRobot* robot, const char* link_name, int mesh_index) {
    // First try visual meshes
    const char* mesh = urdf_robot_get_link_visual_mesh(robot, link_name, mesh_index);
    if (mesh) return mesh;
    
    // Then try collision meshes
    return urdf_robot_get_link_collision_mesh(robot, link_name, mesh_index);
}

// Add these implementations
static void print_mesh_info_for_link(URDFRobot* robot, const char* link_name) {
    int num_visual = 0;
    int num_collision = 0;
    
    // Count visual and collision meshes
    for (int i = 0; i < robot->num_links; i++) {
        if (strcmp(robot->links[i].name, link_name) == 0) {
            num_visual = robot->links[i].num_visual_meshes;
            num_collision = robot->links[i].num_collision_meshes;
            break;
        }
    }
    
    // Print mesh information with proper indentation
    printf("    Link '%s':\n", link_name);
    
    if (num_visual > 0) {
        printf("      Visual meshes:\n");
        for (int i = 0; i < num_visual; i++) {
            const char* mesh_file = urdf_robot_get_link_mesh_filename(robot, link_name, i);
            if (mesh_file) {
                printf("        - %s", mesh_file);
                if (access(mesh_file, F_OK) == 0) {
                    printf(" (Found)\n");
                } else {
                    printf(" (Not found)\n");
                }
            }
        }
    } else {
        printf("      No visual meshes\n");
    }
    
    if (num_collision > 0) {
        printf("      Collision meshes:\n");
        for (int i = 0; i < num_collision; i++) {
            const char* mesh_file = urdf_robot_get_link_mesh_filename(robot, link_name, i + num_visual);
            if (mesh_file) {
                printf("        - %s", mesh_file);
                if (access(mesh_file, F_OK) == 0) {
                    printf(" (Found)\n");
                } else {
                    printf(" (Not found)\n");
                }
            }
        }
    } else {
        printf("      No collision meshes\n");
    }
    
    // Print collision primitives if any
    // TODO: Add support for collision primitives (sphere, cylinder, etc.)
}

static void print_joint_tree_recursive(URDFRobot* robot, const char* current_link, int depth) {
    // Print indentation
    for (int i = 0; i < depth; i++) {
        printf("  ");  // Two spaces per level
    }
    
    // Get joint info for this link (by finding joint where this link is the child)
    const char* joint_name = NULL;
    int joint_type = 0;
    
    // For non-base links, find their connecting joint
    if (depth > 0) {
        // Get parent link
        const char* parent = urdf_robot_get_link_parent_name(robot, current_link);
        if (parent) {
            // Find joint connecting parent to current
            for (int i = 0; i < urdf_robot_get_num_joints(robot); i++) {
                const char* name = urdf_robot_get_joint_name(robot, i);
                const char* parent_link = urdf_robot_get_joint_parent_link(robot, name);
                const char* child_link = urdf_robot_get_joint_child_link(robot, name);
                
                if (strcmp(parent_link, parent) == 0 && strcmp(child_link, current_link) == 0) {
                    joint_name = name;
                    joint_type = urdf_robot_get_joint_type(robot, name);
                    break;
                }
            }
        }
    }
    
    // Print current link and its joint info
    if (depth == 0) {
        printf("Base: %s\n", current_link);
    } else {
        const char* type_str = "fixed";
        if (joint_type == 1) type_str = "revolute";
        else if (joint_type == 2) type_str = "prismatic";
        
        printf("└─ [%s] Joint '%s' -> Link '%s'\n", 
               type_str, 
               joint_name ? joint_name : "unknown",
               current_link);
    }

    // Find the link structure
    URDFLink* link = NULL;
    for (int i = 0; i < robot->num_links; i++) {
        if (strcmp(robot->links[i].name, current_link) == 0) {
            link = &robot->links[i];
            break;
        }
    }

    if (link) {
        // Print visual meshes
        if (link->num_visual_meshes > 0) {
            for (int i = 0; i < depth + 1; i++) printf("  ");
            printf("Visual meshes:\n");
            for (int i = 0; i < link->num_visual_meshes; i++) {
                for (int j = 0; j < depth + 2; j++) printf("  ");
                printf("- %s\n", link->visual_mesh_files[i]);
            }
        }

        // Print collision meshes
        if (link->num_collision_meshes > 0) {
            for (int i = 0; i < depth + 1; i++) printf("  ");
            printf("Collision meshes:\n");
            for (int i = 0; i < link->num_collision_meshes; i++) {
                for (int j = 0; j < depth + 2; j++) printf("  ");
                printf("- %s\n", link->collision_mesh_files[i]);
            }
        }
    }
    
    // Find children by scanning all links and checking their parents
    for (int i = 0; i < urdf_robot_get_num_links(robot); i++) {
        const char* link = urdf_robot_get_link_name(robot, i);
        const char* parent = urdf_robot_get_link_parent_name(robot, link);
        
        if (parent && strcmp(parent, current_link) == 0) {
            print_joint_tree_recursive(robot, link, depth + 1);
        }
    }
}

void urdf_robot_print_tree(URDFRobot* robot) {
    if (!robot) return;
    
    printf("\nRobot Structure:\n");
    printf("=====================================\n");
    
    // Find base link (the one without a parent)
    const char* base_link = NULL;
    for (int i = 0; i < urdf_robot_get_num_links(robot); i++) {
        const char* link = urdf_robot_get_link_name(robot, i);
        if (!urdf_robot_get_link_parent_name(robot, link)) {
            base_link = link;
            break;
        }
    }
    
    if (base_link) {
        print_joint_tree_recursive(robot, base_link, 0);
    }
    printf("\n");
}

void urdf_robot_print_mesh_info(URDFRobot* robot) {
    if (!robot) return;
    
    printf("\nMesh Information:\n");
    printf("=====================================\n");
    
    for (int i = 0; i < urdf_robot_get_num_links(robot); i++) {
        const char* link_name = urdf_robot_get_link_name(robot, i);
        print_mesh_info_for_link(robot, link_name);
    }
    printf("\n");
}

// Add implementations of other functions declared in the header
const char* urdf_robot_get_base_link(URDFRobot* robot) {
    for (int i = 0; i < robot->num_links; i++) {
        const char* link_name = robot->links[i].name;
        if (!urdf_robot_get_link_parent_name(robot, link_name)) {
            return link_name;
        }
    }
    return NULL;
}

int urdf_robot_get_link_depth(URDFRobot* robot, const char* link_name) {
    int depth = 0;
    const char* current = link_name;
    
    while (current) {
        current = urdf_robot_get_link_parent_name(robot, current);
        if (current) depth++;
    }
    
    return depth;
}

URDFMeshes* urdf_robot_load_meshes(URDFRobot* robot, float scale) {
    URDFMeshes* meshes = (URDFMeshes*)malloc(sizeof(URDFMeshes));
    meshes->count = robot->num_links;
    meshes->meshes = (STLMesh**)malloc(meshes->count * sizeof(STLMesh*));
    meshes->names = (char**)malloc(meshes->count * sizeof(char*));
    memset(meshes->meshes, 0, meshes->count * sizeof(STLMesh*));
    memset(meshes->names, 0, meshes->count * sizeof(char*));

    for (int i = 0; i < meshes->count; i++) {
        const char* link_name = robot->links[i].name;
        
        // Try to load visual mesh first
        const char* mesh_file = urdf_robot_get_link_visual_mesh(robot, link_name, 0);
        if (!mesh_file) {
            // If no visual mesh, try collision mesh
            mesh_file = urdf_robot_get_link_collision_mesh(robot, link_name, 0);
        }
        
        if (mesh_file) {
            meshes->meshes[i] = stl_mesh_create();
            if (stl_mesh_load(meshes->meshes[i], mesh_file)) {
                printf("Loaded mesh for link %s: %s\n", link_name, mesh_file);
                meshes->names[i] = strdup(link_name);
                
                // Scale the mesh vertices
                for (int j = 0; j < meshes->meshes[i]->num_vertices / 3; j++) {
                    Triangle* tri = &meshes->meshes[i]->triangles[j];
                    for (int k = 0; k < 3; k++) {
                        tri->vertices[k].x *= scale;
                        tri->vertices[k].y *= scale;
                        tri->vertices[k].z *= scale;
                    }
                }
                
                stl_mesh_compute_buffers(meshes->meshes[i]);
            } else {
                printf("Failed to load mesh for link %s: %s\n", link_name, mesh_file);
                stl_mesh_destroy(meshes->meshes[i]);
                meshes->meshes[i] = NULL;
            }
        } else {
            printf("No mesh found for link %s\n", link_name);
            meshes->meshes[i] = NULL;
        }
    }
    
    return meshes;
}

void urdf_meshes_destroy(URDFMeshes* meshes) {
    if (!meshes) return;
    
    for (int i = 0; i < meshes->count; i++) {
        if (meshes->meshes[i]) {
            stl_mesh_destroy(meshes->meshes[i]);
        }
        free(meshes->names[i]);
    }
    
    free(meshes->meshes);
    free(meshes->names);
    free(meshes);
}

// ... 