#include <cstdio>
#include <cstring>
#include <iostream>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "rclcpp/rclcpp.hpp"

class HumanoidSimulNode : public rclcpp::Node
{
public:
    HumanoidSimulNode()	: Node("humanoid_simul"){
	this->declare_parameter<std::string>("model", "model.xml");
	this->get_parameter("model", this->model_file);
    }

    void set_joint_names(std::vector<std::string> names){
    }

    void publish_joint_angles(std::vector<double> angles){
    }

    void goal_angle_callback(){
    }

    std::string model_file;
};

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
	mj_resetData(m, d);
	mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
    
    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
	return;
    }
    
    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    
    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    
    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
    
    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
	action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
	action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
	action = mjMOUSE_ZOOM;
    }
    
    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
    //std::cout << yoffset << std::endl;
}

void print_joint_list(const mjModel* m) {
    std::cout << "Model has " << m->njnt << " joints." << std::endl;

    std::string joint_types[] = {"Free", "Ball", "Slide", "Hinge"};
    for (int i = 0; i < m->njnt; ++i) {
        // Get the joint name from the name array using the address offset
        const char* joint_name = mj_id2name(m, mjOBJ_JOINT, i);
        if (joint_name) {
            std::cout << "Joint " << i << ": " << joint_name << "  Type: " << joint_types[m->jnt_type[i]] << std::endl;
        } else {
            std::cout << "Joint " << i << ": (unnamed)" << "  Type: " << joint_types[m->jnt_type[i]] << std::endl;
        }
        // // Joint position in the body frame
        // std::cout << "  Position (pos): " << m->jnt_pos[3*i] << ", "
        //           << m->jnt_pos[3*i+1] << ", " << m->jnt_pos[3*i+2] << std::endl;
    }
}

void print_qpos(const mjModel* m, const mjData* d) {
    std::cout << "Joint positions (qpos):" << std::endl;
    // Iterate over all position coordinates (nq)
    for (int i = 0; i < m->nq; ++i) {
        std::cout << "qpos[" << i << "]: " << d->qpos[i] << std::endl;
    }
}

// main function
int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    HumanoidSimulNode n;
    RCLCPP_INFO_STREAM(n.get_logger(), "Model file: " << n.model_file);
    rclcpp::Rate rate(std::chrono::milliseconds(33));
    
    // load and compile model
    char error[1000] = "Could not load binary model";
    if(n.model_file.compare(n.model_file.size() - 4, 4, ".mjb") == 0){
	//if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
	m = mj_loadModel(n.model_file.c_str(), 0);
    } else {
	m = mj_loadXML(n.model_file.c_str(), 0, error, 1000);
    }
    if (!m) {
	mju_error("Load model error: %s", error);
	return -1;
    }
    print_joint_list(m);
    
    // make data
    d = mj_makeData(m);
    
    // init GLFW
    if (!glfwInit()) {
	mju_error("Could not initialize GLFW");
    }
    
    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
  
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // run main loop, target real-time simulation and 60 fps rendering
    int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, "AAHead_yaw");
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*5, &scn, &cam);
    while (!glfwWindowShouldClose(window) && rclcpp::ok()) {
	// advance interactive simulation for 1/60 sec
	//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
	//  this loop will finish on time for the next frame to be rendered at 60 fps.
	//  Otherwise add a cpu timer and exit this loop when it is time to render.
	mjtNum simstart = d->time;
	while (d->time - simstart < 1.0/30.0) {
	    mj_step(m, d);
	}
	
	// get framebuffer viewport
	mjrRect viewport = {0, 0, 0, 0};
	glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
	
	// update scene and render
	mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
	mjr_render(viewport, &scn, &con);
	
	// swap OpenGL buffers (blocking call due to v-sync)
	glfwSwapBuffers(window);
	
	// process pending GUI events, call GLFW callbacks
	glfwPollEvents();

	rclcpp::spin_some(n.get_node_base_interface());
	rate.sleep();
	//print_qpos(m,d);
	d->ctrl[actuator_id] = sin(6.28*d->time);
    }
    
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    
    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
