#include <viewer.h>
#include <cstddef>
#include <algorithm>
#include <iostream>

#include "world.h"
#include "scene.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_transform_2d.hpp>

float checkRelativeVelocity(RigidBody& a, RigidBody& b, const glm::vec2& normal);
float calculateImpulseMagnitude(RigidBody& a, RigidBody& b, const glm::vec2& normal);
float getEpsilon();
float getRelativeVelocity(RigidBody& a, RigidBody& b, const glm::vec2& normal);

struct ExerciseBroadPhase : public BroadPhaseBase
{
    /*
     * Task 3:
     *  -> implement a broad-phase algorithm of your choice to speed-up the collision detection
     *  -> you might not need to implement all methods (depends on the algorithm you want to implement)
     */

    void onCreate(RigidBody& body) override final
    {
        /* called when a rigid body is created (e.g. insert into hierarchy) */
    }

    void onUpdate(RigidBody& body)  override final
    {
        /*
         * called when the position and rotation of a rigid body changes
         * e.g. update the axis aligned bounding box
         *
         * You will likely handle the body differently depending on the shape,
         * to access the specific data check the type and cast.
         */

        if (body.shape.type == Shape::eType::CIRCLE)
        {
            auto& circle = static_cast<Circle&>(body.shape);
            /* circle.radius, body.position */
        }
        else if (body.shape.type == Shape::eType::BOX)
        {
            auto& box = static_cast<Box&>(body.shape);
            /*  box.world_vertices (corner points of the rectangle) */
        }
    }

    void query(std::vector<RigidBody>& bodies, std::vector<Contact>& collisionContacts) override final
    {
        /* perform broad-phase collision test and only call the narrow-phase ( collision(a, b) ) if there is a potential contact */

        /* brute force test all bodies against each other for possible narrow-phase collision */
        for (auto& a : bodies)
        {
            for (auto& b : bodies)
            {
                /* are the same body or both are static bodies */
                if (&a == &b) continue;
                if (a.type == RigidBody::eType::STATIC && b.type == RigidBody::eType::STATIC) continue;

                /* check narrow-phase collision and add contact information to collisionContacts if the bodies are overlaping */
                if (auto result = collision(a, b)) { collisionContacts.emplace_back(std::move(*result)); }
            }
        }
    }

    void clear() override final
    {
        /* all rigid bodies are deleted */
    }
};



void contact_response(RigidBody& a, RigidBody& b, const glm::vec2& point, const glm::vec2& normal, float distance, float dt)
{
    /* Task 1 and 2:
     *  -> compute the impulse and update the velocities to keep the rigid bodies a and b from colliding further (see lecture slides "Rigid Bodies 2" 14-27)
     *  Note: skip the computation of an impulse if the bodies are already seperating (check relative velocity)
     *
     * RigidBody contains the state variables of the rigid body (e.g. a.position, a.velocity, a.angle, a.angularVelocity, a.coeffRest).
     * The mass and inertia of a body are stored in the "shape" of a body (a.shape.mass, a.shape.inertia)
     *
     * Note, that a rigid body can be a static body and as such should not be subject to motion.
     *  if(a.type == RigidBody::eType::STATIC) { ... }
     *  if(a.type == RigidBody::eType::DYNAMIC) { ... }
     */

 

     // calculate the relative velocity in the direction of the contact normal
    float relativeVelocity = getRelativeVelocity(a, b, normal);
    float impulse = calculateImpulseMagnitude(a, b, normal, point);
    glm::vec2 r_a = point - a.position;
    glm::vec2 r_b = point - b.position;
    glm::vec2 impulse_normal = impulse * normal;
    float inverseInertiaB = 1 / b.shape.inertia;
    float inverseInertiaA = 1 / a.shape.inertia;
    
    // if the bodies are already separating, no impulse is needed
    if (relativeVelocity <= 0.0f) {
        return;
    }else {

        
        
        if (a.type == RigidBody::eType::STATIC) {
            b.velocity -= impulse * normal / b.shape.mass;
            
            b.angularVelocity += inverseInertiaB * (r_b.x * impulse_normal.y - r_b.y * impulse_normal.x);
        }
        else if (b.type == RigidBody::eType::STATIC) {
            a.velocity -= impulse * normal/ a.shape.mass;
            a.angularVelocity -= inverseInertiaA * (r_a.x * impulse_normal.y - r_a.y * impulse_normal.x);
        }
        else {
            
            a.velocity += impulse * normal/ a.shape.mass;
            b.velocity -= impulse * normal/ b.shape.mass;
            a.angularVelocity += inverseInertiaA * (r_a.x * impulse_normal.y - r_a.y * impulse_normal.x);
            b.angularVelocity -= inverseInertiaB * (r_b.x * impulse_normal.y - r_b.y * impulse_normal.x);
        }
        return;
    }
}



float calculateImpulseMagnitude(RigidBody& a, RigidBody& b, const glm::vec2& normal, const glm::vec2& contactPoint) {
    float inv_mass_a = 1 / a.shape.mass;
    float inv_mass_b = 1 / b.shape.mass;

    // Calculate moment of inertia for each body
    float inv_inertia_a = 1 / a.shape.inertia;
    float inv_inertia_b = 1 / b.shape.inertia;

    // Calculate perpendicular distance from contact point to center of mass for each body
    glm::vec2 r_a = contactPoint - a.position;
    glm::vec2 r_b = contactPoint - b.position;
    float r_perp_a = glm::dot(r_a, glm::vec2(-normal.y, normal.x));
    float r_perp_b = glm::dot(r_b, glm::vec2(-normal.y, normal.x));

    // Calculate relative velocity at contact point
    glm::vec2 v_rel = a.velocity + r_perp_a * a.angularVelocity - b.velocity - r_perp_b * b.angularVelocity;

    // Calculate impulse magnitude
    float j = -(1 + b.coeffRest) * glm::dot(normal, v_rel) / (glm::dot(normal, normal) * (inv_mass_a + inv_mass_b + pow(r_perp_a, 2) * inv_inertia_a + pow(r_perp_b, 2) * inv_inertia_b));

    return j;
}


float getRelativeVelocity(RigidBody& a, RigidBody& b, const glm::vec2& normal) {
    return glm::dot(normal, (a.velocity - b.velocity));
}
//returns relative velocity of bodies a and b in normal direction of impact


int main(int argc, char** argv)
{
    Viewer viewer;
    viewer.mWindow.title = "02 Rigid Bodies";
    viewer.mWindow.width = 1280;
    viewer.mWindow.height = 720;
    viewer.mWindow.vsync = true;  /* Note: call to onUpdate function depends on display refresh rate */
    viewer.mWindow.mHDPI = false; /* = true for 4k (highres) displays */
    viewer.mCamera.size /= 50.0f;
    viewer.mRender.pointRadius = 5.0f;
    viewer.mRender.lineWidth = 2.0f;
    viewer.mRender.wireframe = true;

    struct
    {
        eScene scene = eScene::CIRCLE;
        bool renderWireframe = true;
        bool renderContactPoints = false;
        bool renderContactNormals = false;
        bool renderBodySate = false;
        int broad_phase_algo = 0;
    } helper;

    World world;
    setup_scene(world, helper.scene);


    viewer.onUpdate([&](Window& window, double dt)
    {
        world.update();
    });


    viewer.onDraw([&](Window& window, double dt)
    {
        viewer.mRender.wireframe = helper.renderWireframe;

        /* render rigid bodies */
        const auto& bodies = world.bodies();
        for(const auto& body : bodies)
        {
            glm::vec4 color = (body.type == RigidBody::eType::DYNAMIC)
                    ? glm::vec4{0.0f, 0.0f, 0.0f, 1.0f} : glm::vec4{0.0, 0.0, 1.0, 1.0};

            const auto& shape = body.shape;
            if(shape.type == Shape::eType::CIRCLE)
            {
                const auto& circle = static_cast<const Circle&>(shape);
                viewer.drawCircleOutline(body.position, circle.radius, color);
            }
            else if(shape.type == Shape::eType::BOX)
            {
                const auto& box = static_cast<const Box&>(shape);
                viewer.drawQuad(box.world_vertices[0], box.world_vertices[1], box.world_vertices[2], box.world_vertices[3], color);
            }
        }

        /* render rigid bodie state (position, rotation and velocity) */
        if(helper.renderBodySate)
        {
            viewer.drawPoints(bodies.begin(), bodies.end(), [](const auto& body, glm::vec2& coord, glm::vec4& color)
            {
                coord = body.position;
                color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
                return true;
            });

            viewer.drawLines(bodies.begin(), bodies.end(), [](const auto& body, glm::vec2& start, glm::vec2& end, glm::vec4& color)
            {
                glm::vec2 dir(glm::cos(body.angle), glm::sin(body.angle));

                start = body.position - 0.2f *dir;
                end = body.position + 0.2f *dir;
                color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
                return true;
            });

            viewer.drawLines(bodies.begin(), bodies.end(), [](const auto& body, glm::vec2& start, glm::vec2& end, glm::vec4& color)
            {
                start = body.position;
                end = body.position + 0.2f * body.velocity;
                color = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
                return true;
            });
        }

        /* render collision information (contact points) */
        if(helper.renderContactPoints)
        {
            const auto& contacts = world.contacts();
            std::vector<glm::vec2> points;
            for(auto& contact : contacts)
            {
                for(int i = 0; i < contact.num_contacts; i++)
                {
                    points.emplace_back(contact.contact[i].point);
                }
            }

            viewer.drawPoints(points.begin(), points.end(), [](const auto& contact, glm::vec2& coord, glm::vec4& color)
            {
                coord = contact;
                color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                return true;
            });
        }

         /* render collision information (contact normals) */
        if(helper.renderContactNormals)
        {
            const auto& contacts = world.contacts();
            viewer.drawLines(contacts.begin(), contacts.end(), [](const auto& contact, glm::vec2& start, glm::vec2& end, glm::vec4& color)
            {
                start = contact.contact[0].point;
                end = contact.contact[0].point + 0.2f * contact.contact[0].normal;
                color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                return true;
            });
        }
    });


    viewer.onGui([&](Window& window, double dt)
    {
        ImGui::SetNextWindowPos(ImVec2{16, 16});
        ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings);
        {
            const auto& times = world.performanceMeasure();
            ImGui::TextColored(ImVec4(0.9, 0.6, 0.6, 1.0), "FPS: %4.2f", viewer.fps());
            ImGui::TextColored(ImVec4(1.0, 0.4, 0.4, 1.0), "World Update: %4.3f ms", times.at("WorldUpdate"));
            ImGui::TextColored(ImVec4(0.8, 0.4, 1.0, 1.0), "Integrate Forces: %4.3f ms", times.at("WorldIntegrateForces"));
            ImGui::TextColored(ImVec4(0.8, 0.4, 1.0, 1.0), "Collision detection: %4.3f ms", times.at("CollisionDetection"));
            ImGui::TextColored(ImVec4(0.8, 0.4, 1.0, 1.0), "Collision response: %4.3f ms", times.at("CollisionResponse"));
            ImGui::TextColored(ImVec4(0.8, 0.4, 1.0, 1.0), "Integrate Velocities: %4.3f ms", times.at("WorldIntegrateVelocities"));

            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.6, 0.8, 0.6, 1.0), "Physic Engine Info: ");
            {
                ImGui::TextColored(ImVec4(1.0, 1.0, 1.0, 1.0), "Body count: %zu", world.bodies().size());
                ImGui::TextColored(ImVec4(1.0, 1.0, 1.0, 1.0), "Collisions: %zu", world.contacts().size());
            }

            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.6, 0.8, 0.6, 1.0), "Physic Engine Parameter: ");
            {
                ImGui::SliderFloat("timestep", &world.timestep, 0.0, 0.032f);
                ImGui::SliderInt("substeps", &world.substeps, 1, 100);
                ImGui::SliderInt("solver iterations",  &world.solverIterations, 0, 200);

                const char* algos_string[] = { "brute force", "exercise" };
                if(ImGui::Combo("broadphase algo", &helper.broad_phase_algo, algos_string, 2))
                {
                    if(helper.broad_phase_algo == 0) world.broadPhase<BruteForceBroadPhase>();
                    else if(helper.broad_phase_algo == 1) world.broadPhase<ExerciseBroadPhase>();
                }
            }

            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.6, 0.8, 0.6, 1.0), "Debug: ");
            {
                ImGui::Checkbox("draw body state", &helper.renderBodySate);
                ImGui::Checkbox("contact point", &helper.renderContactPoints);
                ImGui::Checkbox("contact normal", &helper.renderContactNormals);
            }

            ImGui::Separator();

            const char* scene_strings[] = { "circles", "boxes", "restitution", "slope", "broad phase", "angry ball" };
            int _scene = static_cast<int>(helper.scene);
            if(ImGui::Combo("scene", &_scene, scene_strings, 6))
            {
                helper.scene = static_cast<eScene>(_scene);
                setup_scene(world, helper.scene);
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos( ImVec2(16, window.size().y - 112 * (viewer.mWindow.mHDPI ? 1.5f : 1.0f)) );
        ImGui::Begin("##controls", nullptr, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar );
        {
            ImGui::TextColored({1.0, 1.0, 0.0, 1.0},   "[w/s keys]   ");    ImGui::SameLine(); ImGui::TextColored({1.0, 1.0, 1.0, 0.9}, " control zoom level");
            ImGui::TextColored({1.0, 1.0, 0.0, 1.0},   "[f key]      ");    ImGui::SameLine(); ImGui::TextColored({1.0, 1.0, 1.0, 0.9}, " toggle fullscreen");
            ImGui::TextColored({1.0, 1.0, 0.0, 1.0},   "[r key]      ");    ImGui::SameLine(); ImGui::TextColored({1.0, 1.0, 1.0, 0.9}, " reload scene");
            ImGui::TextColored({1.0, 1.0, 0.0, 1.0},   "[mouse left] ");    ImGui::SameLine(); ImGui::TextColored({1.0, 1.0, 1.0, 0.9}, " spawn circle body");
            ImGui::TextColored({1.0, 1.0, 0.0, 1.0},   "[mouse right]");    ImGui::SameLine(); ImGui::TextColored({1.0, 1.0, 1.0, 0.9}, " spawn box body");
        }
        ImGui::End();
    });


    viewer.onMouseButton([&](Window& window, Mouse& mouse, int button, int mod, bool press)
    {
        if(viewer.mUI.mouseCaptured) return;

        if(press && button == GLFW_MOUSE_BUTTON_1)
        {
            world.body(viewer.worldSpacePosition(mouse.position()), Circle{0.5, 0.2});
        }

        if(press && button == GLFW_MOUSE_BUTTON_2)
        {
            world.body(viewer.worldSpacePosition(mouse.position()), Box{0.5, 0.2, 0.2});
        }
    });


    viewer.onKey([&](Window& window, Keyboard& keyboard, int key, int mod, bool press)
    {
        if(!press) return;

        /* toggle broadphase algorithm */
        if(key == GLFW_KEY_SPACE)
        {
            helper.broad_phase_algo = !helper.broad_phase_algo;
            if(helper.broad_phase_algo == 0) world.broadPhase<BruteForceBroadPhase>();
            else if(helper.broad_phase_algo == 1) world.broadPhase<ExerciseBroadPhase>();
        }

        /* close application */
        if(key == GLFW_KEY_ESCAPE) { window.close(true); }

        /* reload scene */
        if(key == GLFW_KEY_R) { setup_scene(world, helper.scene); }

        /* fullscreen toggle */
        if(key == GLFW_KEY_F) { window.fullscreen( !window.fullscreen() ); }

        /* zoom in */
        if(key == GLFW_KEY_W) { viewer.mCamera.zoom -= 0.25f; }

        /* zoom out */
        if(key == GLFW_KEY_S) { viewer.mCamera.zoom += 0.25f; }
    });

    viewer.run();

    return EXIT_SUCCESS;
}
