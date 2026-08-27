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
    if (m) {
        mjv_moveCamera(m, action, dx/height, dy/height, &cam);
    }
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    if (m) {
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &cam);
    }
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
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    HumanoidSimulNode n;
    RCLCPP_INFO_STREAM(n.get_logger(), "Model file: " << n.model_file);
    rclcpp::Rate rate(std::chrono::milliseconds(33));
    
    // Cargar y compilar modelo
    char error[1000] = "Could not load binary model";
    if (n.model_file.size() >= 4 && n.model_file.compare(n.model_file.size() - 4, 4, ".mjb") == 0) {
        m = mj_loadModel(n.model_file.c_str(), 0);
    } else {
        m = mj_loadXML(n.model_file.c_str(), 0, error, 1000);

        if (!m) {
            RCLCPP_ERROR(n.get_logger(), "Error al cargar el archivo XML: %s", error);
            rclcpp::shutdown();
            return -1;
        }
    }

    // Validar que el modelo haya cargado correctamente
    if (!m) {
        RCLCPP_ERROR(n.get_logger(), "Error al cargar el modelo MuJoCo: %s", error);
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    
    print_joint_list(m);
    d = mj_makeData(m);
    
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }
    
        // 1. Verificar carga del modelo
    if (!m) {
        RCLCPP_ERROR(n.get_logger(), "Error al cargar modelo XML (%s): %s", n.model_file.c_str(), error);
        rclcpp::shutdown();
        return 1;
    }

    // 2. Verificar inicialización de GLFW
    if (!glfwInit()) {
        RCLCPP_ERROR(n.get_logger(), "Error: No se pudo inicializar GLFW");
        rclcpp::shutdown();
        return 1;
    }

    // 3. Verificar creación de la ventana gráfica
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    if (!window) {
        RCLCPP_ERROR(n.get_logger(), "Error: GLFW no pudo crear la ventana. Revisa los drivers de pantalla o la variable $DISPLAY.");
        glfwTerminate();
        rclcpp::shutdown();
        return 1;
}
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
  
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, "AAHead_yaw");
    if (actuator_id < 0) {
        RCLCPP_WARN(n.get_logger(), "El actuador 'AAHead_yaw' no fue encontrado en el modelo.");
    }

    if (m) {
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*5, &cam);
    }

    while (!glfwWindowShouldClose(window) && rclcpp::ok()) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/30.0) {
            mj_step(m, d);
        }
        
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        
        glfwSwapBuffers(window);
        glfwPollEvents();

        rclcpp::spin_some(n.get_node_base_interface());
        rate.sleep();

        // Control seguro del actuador
        if (actuator_id >= 0) {
            d->ctrl[actuator_id] = sin(6.28 * d->time);
        }
    }
    
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
