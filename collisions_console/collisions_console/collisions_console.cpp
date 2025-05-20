#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <ctime>
#include <math.h>
#include <string>
#include "shader_m.h"
#include "detection.h"
#include "generation.h"
#include "pair_maker.h"
#include "sweep_and_prune.h"
#include "collision_resolution.h"
#include "write_fluid_positions.h"

const int SCR_WIDTH = 800;
const int SCR_HEIGHT = 600;

const bool REAL_TIME = false;
const bool WRITE_TIMESTEPS = true;
const double FIXED_TIMESTEP = .01;
const int FRAME_MAX = 200;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

void  processInput(GLFWwindow* window, float& camera_azimuth, float& camera_elevation, double time_step);

int main()
{
    srand(time(0));
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Simulation", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
    glEnable(GL_DEPTH_TEST);

    //Make Shaders
    Shader shader("coordinate_systems.vs", "coordinate_systems.fs");

    /*Polyhedron::external_force = [](Polyhedron* ptr) {
        if (typeid(*ptr) == typeid(Sphere)) 
            return glm::vec3(0.0f, -9.8, 0.0f) * ptr->mass; 
        else 
            return glm::vec3(0.0f, 0.0f, 0.0f);
    };*/

    Polyhedron::external_force = [](Polyhedron* ptr) {
        return glm::vec3(0.0f, -9.8, 0.0f) * ptr->mass;
    };

    int num_fluid_particles = 400;
    float fluid_radius = .0625;
    float fluid_density = 1;

    std::vector<Polyhedron*> polyhedra = std::vector<Polyhedron*>();
    std::vector<Polyhedron*> colliding_polyhedra = std::vector<Polyhedron*>();

    for (int p = 0; p < num_fluid_particles; p++) {
        Sphere* fluid_particle = new Sphere(fluid_radius, fluid_density);
        polyhedra.push_back(fluid_particle);
        colliding_polyhedra.push_back(fluid_particle);
        fluid_particle->displacement = glm::vec3(.875 * rand_float(), .5 * rand_float() -.5, .875 * rand_float());
    }

    std::vector<std::string> dragon_files = std::vector<std::string>();
    
    for (int i = 0; i < 81; i++) {
        dragon_files.emplace_back("data/Dragon_50k_" + std::to_string(i) + ".obj");
    }

    
    ConcavePolyhedron* dragon = new ConcavePolyhedron(.5, dragon_files);
    polyhedra.push_back(dragon);
    colliding_polyhedra.push_back(dragon);
    dragon->displacement = glm::vec3(0.0f, 0.75f, 0.0);
    dragon->color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

    //Generate Bounding Box Object
    glm::vec3 bb_min = glm::vec3(-1.0f, -1.0f, -1.0f);
    glm::vec3 bb_max = glm::vec3(1.0f, 1.0f, 1.0f);

    ConvexPolyhedron* bounding_box = generate_cube(0, bb_min, bb_max);
    polyhedra.push_back(bounding_box);

    PairMaker* sweep_and_prune_manager;

    sweep_and_prune_manager = new SweepAndPrune(colliding_polyhedra, bb_min, bb_max);

    std::vector<std::pair<Polyhedron*, glm::vec3>> edge_striking = std::vector<std::pair<Polyhedron*, glm::vec3>>();
    std::vector<std::pair<Polyhedron*, Polyhedron*>> potentially_colliding_pairs = std::vector<std::pair<Polyhedron*, Polyhedron*>>();


    //Produce data required by OpenGL for rendering polyhedra
    std::vector<float> vertices_flat = std::vector<float>();
    std::vector<unsigned int> indices_flat = std::vector<unsigned int>();

    int polyhedron_offset = 0;

    for (Polyhedron* polyhedron : polyhedra) {
        polyhedron->insert_vertices(vertices_flat);
        polyhedron->insert_indices(indices_flat, polyhedron_offset);
    }

    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices_flat.size() * sizeof(float), vertices_flat.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_flat.size() * sizeof(unsigned int), indices_flat.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glBindVertexArray(VAO);

    int frames_written = 0;

    float camera_distance = 2.0;
    float camera_azimuth = 0.0;
    float camera_elevation = 0.0;

    double start_time = glfwGetTime();
    double time_since_last_printout = 0.0;
    double current_time = glfwGetTime();
    double next_time = 0.0;
    double time_step = 0.0;
    int elapsed_frames = 0;
    int frames_since_last_printout = 0;

    while (!glfwWindowShouldClose(window))
    {
        if (FRAME_MAX != -1 && frames_written == FRAME_MAX)
            break;

        next_time = glfwGetTime();
        time_step = next_time - current_time;
        time_since_last_printout += time_step;
        current_time = next_time;
        elapsed_frames++;
        frames_since_last_printout++;

        if (time_since_last_printout > 10.0) {
            std::cout << "FPS: " << frames_since_last_printout / time_since_last_printout << std::endl;
            time_since_last_printout = 0.0;
            frames_since_last_printout = 0;
        }

        //Get potentially colliding pairs
        sweep_and_prune_manager->update();
        sweep_and_prune_manager->find_edge_striking_polys(edge_striking);
        sweep_and_prune_manager->find_potential_colliding_pairs(potentially_colliding_pairs);

        //Compute edge collisions 
        for (std::pair<Polyhedron *, glm::vec3>& pair : edge_striking) {
            pair.first->displace(pair.second);
            glm::vec3 norm_second = glm::normalize(pair.second);
            pair.first->velocity -= norm_second * glm::dot(pair.first->velocity, norm_second);
        }

        //Compute collisions for potentially colliding pairs
        for (std::pair<Polyhedron*, Polyhedron*>& pair : potentially_colliding_pairs) {
            detection det = intersects(pair.first, pair.second);

            if (glm::isnan(det.penetration[0]), glm::isnan(det.penetration[1]), glm::isnan(det.penetration[2]))
                det.penetration = random_unit_surface_point() * fluid_radius / 25.0f;

            if (det.detected) {
                resolve_collision(pair.first, pair.second, det.penetration);
            }
        }

        //Update polyhedra displacements
        for (Polyhedron* polyhedron : polyhedra) {
            polyhedron->time_step(REAL_TIME ? time_step : FIXED_TIMESTEP);
        }

        if (WRITE_TIMESTEPS)
            writeParticlePositions("output/particles_step_" + std::to_string(frames_written++) + ".json", polyhedra);
        

        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

        processInput(window, camera_azimuth, camera_elevation, time_step);

        glClearColor(0.2f, 0.7f, 0.7f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader.use();

        // create spatial transformation
        
        glm::vec3 camera_position = camera_distance * glm::vec3(glm::cos(camera_elevation) * glm::cos(camera_azimuth), 
                                                                glm::sin(camera_elevation),
                                                                glm::cos(camera_elevation)* glm::sin(camera_azimuth));

        glm::mat4 view = glm::lookAt(camera_position,
                                     glm::vec3(0.0f, 0.0f, 0.0f),
                                     glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 projection = glm::mat4(1.0f);
        //projection = glm::ortho(0.0f, 800.0f, 0.0f, 600.0f, 0.1f, 100.0f);
        projection = glm::perspective(glm::radians(90.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

        shader.setMat4("projection", projection); 
        shader.setMat4("view", view);

        int polyhedron_offset = 0;
        for (int p_index = 0; p_index < polyhedra.size(); p_index++) {
            glm::mat4 model = glm::mat4(1.0);
            model = glm::translate(model, polyhedra[p_index]->displacement);
            model = model * glm::mat4(polyhedra[p_index]->rotation);
            shader.setMat4("model", model);
            shader.setVec4("color", polyhedra[p_index]->color);
            
            glDrawElements(GL_TRIANGLES, 3 * polyhedra[p_index]->num_triangles, GL_UNSIGNED_INT, (void*)(3 * polyhedron_offset * sizeof(unsigned int)));
            polyhedron_offset += polyhedra[p_index]->num_triangles;
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    delete sweep_and_prune_manager;
    for (Polyhedron* poly : polyhedra) {
        delete poly;
    }
    glfwTerminate();

    std::cout << "Average FPS: " << elapsed_frames / (current_time - start_time) << std::endl;
    system("pause");
    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window, float &camera_azimuth, float &camera_elevation, double time_step)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);


    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera_elevation = std::min(glm::radians(90.0f), (float)(camera_elevation + glm::radians(60.0f) * time_step));
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera_elevation = std::max(glm::radians(-90.0f), (float)(camera_elevation - glm::radians(60.0f) * time_step));
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera_azimuth = std::fmod((float)(camera_azimuth - glm::radians(60.0f) * time_step), glm::radians(360.0f));
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera_azimuth = std::fmod((float)(camera_azimuth + glm::radians(60.0f) * time_step), glm::radians(360.0f));
}