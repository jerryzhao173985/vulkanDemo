// main.cpp

#include <vsg/all.h>
#include <ode/ode.h>
#include "primitive.h"
#include "base.h"
#include "osghandle.h"
#include "odehandle.h"
#include "pos.h"
#include "pose.h"
#include "globaldata.h"

using namespace lpzrobots;

int main(int argc, char** argv)
{
    // Initialize ODE
    dInitODE2(0);

    // Create ODE world and space
    dWorldID world = dWorldCreate();
    dSpaceID space = dHashSpaceCreate(0);

    // Set gravity
    dWorldSetGravity(world, 0, 0, -9.81);

    // Create an OdeHandle
    OdeHandle odeHandle;
    odeHandle.world = world;
    odeHandle.space = space;

    // Create an OsgHandle (for compatibility; adjust as needed)
    OsgHandle osgHandle;
    osgHandle.cfg = new OsgConfig();

    // Create a VSG viewer
    auto viewer = vsg::Viewer::create();

    // Set up window traits
    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "VSG Primitive Test";
    windowTraits->width = 800;
    windowTraits->height = 600;
    auto window = vsg::Window::create(windowTraits);
    if (!window)
    {
        std::cerr << "Could not create window." << std::endl;
        return 1;
    }
    viewer->addWindow(window);

    // Create the root node
    auto root = vsg::Group::create();

    // Set up the camera
    auto lookAt = vsg::LookAt::create(
        {5.0, 5.0, 5.0},  // eye position
        {0.0, 0.0, 0.0},  // center position
        {0.0, 0.0, 1.0}   // up vector
    );

    auto perspective = vsg::Perspective::create(
        60.0,                                                 // field of view
        static_cast<double>(window->extent2D().width) /
            static_cast<double>(window->extent2D().height),   // aspect ratio
        0.1,                                                  // near clip
        1000.0                                                // far clip
    );

    auto viewportState = vsg::ViewportState::create(window->extent2D());
    auto camera = vsg::Camera::create(perspective, lookAt, viewportState);

    // Create a command graph
    auto commandGraph = vsg::CommandGraph::create(window);
    commandGraph->addChild(camera);
    commandGraph->addChild(root);

    // Assign tasks to viewer
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    // Create a Base instance
    Base base("VSG Primitive Test");

    // Initialize the physics scene
    base.makePhysicsScene();

    // Create ground plane primitive
    auto ground = new Plane();
    ground->init(odeHandle, 0.0, osgHandle, Primitive::Geom | Primitive::Draw);
    ground->setPose(Pose::translate(0.0, 0.0, 0.0));

    // Add ground visual node to the scene
    auto groundNode = ground->getVSGNode();
    if (groundNode)
    {
        root->addChild(groundNode);
    }

    // Create a box primitive
    auto box = new Box(1.0, 1.0, 1.0);
    box->init(odeHandle, 1.0, osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
    box->setPose(Pose::translate(0.0, 0.0, 5.0)); // Start higher to see it fall

    // Add box visual node to the scene
    auto boxNode = box->getVSGNode();
    if (boxNode)
    {
        root->addChild(boxNode);
    }

    // Create a sphere primitive
    auto sphere = new Sphere(0.5);
    sphere->init(odeHandle, 1.0, osgHandle, Primitive::Body | Primitive::Geom | Primitive::Draw);
    sphere->setPose(Pose::translate(2.0, 0.0, 5.0)); // Start higher to see it fall

    // Add sphere visual node to the scene
    auto sphereNode = sphere->getVSGNode();
    if (sphereNode)
    {
        root->addChild(sphereNode);
    }

    // Add lights to the scene
    base.makeLights(root, *osgHandle.cfg);

    // Compile the Vulkan objects
    viewer->compile();

    // Simulation parameters
    double simulationTime = 0.0;
    double timeStep = 0.01;

    // Main rendering loop
    while (viewer->advanceToNextFrame())
    {
        viewer->handleEvents();

        // Step the physics simulation
        dSpaceCollide(odeHandle.space, nullptr, nullptr);
        dWorldQuickStep(odeHandle.world, timeStep);
        dJointGroupEmpty(0);  // Assuming no joint group used

        simulationTime += timeStep;

        // Update the primitives
        box->update();
        sphere->update();
        ground->update();

        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
    }

    // Cleanup
    delete box;
    delete sphere;
    delete ground;
    delete osgHandle.cfg;

    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}