// base.cpp - Enhanced VSG Version

#include "base.h"
#include "primitive.h"
#include "pos.h"
#include "odehandle.h"
#include "globaldata.h"
#include <iostream>

namespace lpzrobots {

Base::Base(const std::string& caption)
    : caption(caption), title(""), groundTexture("Images/whiteground.jpg"),
      hUDStatisticsManager(nullptr), shadowTexSize(1024), useNVidia(true),
      plane(nullptr), rootNode(nullptr), hudNode(nullptr) {
}

Base::~Base() {
    base_close();
}

void Base::base_close() {
    // Cleanup resources
    if (plane)
        delete plane;
    if (hUDStatisticsManager)
        delete hUDStatisticsManager;
    // Additional cleanup as needed
}

void Base::makePhysicsScene() {
    // Create ODE ground plane (physical plane)
    ground = dCreatePlane(odeHandle.space, 0, 0, 1, 0);
    dGeomSetCategoryBits(ground, Primitive::Stat);
    dGeomSetCollideBits(ground, ~Primitive::Stat);
    // Assign a primitive to the ground plane to have substance (material) support
    plane = new Plane();
    dGeomSetData(ground, (void*)plane);
}

Substance Base::getGroundSubstance() {
    if (plane)
        return plane->substance;
    else
        return Substance();
}

void Base::setGroundSubstance(const Substance& substance) {
    if (plane)
        plane->setSubstance(substance);
}

void Base::makeScene(OsgScene* scene, const OsgConfig& config) {
    // Create the root node
    rootNode = vsg::Group::create();

    // Create the world group
    auto worldGroup = vsg::Group::create();

    // Create the scene group
    auto sceneGroup = vsg::Group::create();

    // Create ground
    auto groundNode = makeGround(config);

    // Add ground to the world
    worldGroup->addChild(groundNode);

    // Add scene to the world
    worldGroup->addChild(sceneGroup);

    // Add world to root
    rootNode->addChild(worldGroup);

    // Assign the groups to the scene structure
    scene->root = rootNode;
    scene->world = worldGroup;
    scene->scene = sceneGroup;
    scene->groundScene = groundNode;
}

vsg::ref_ptr<vsg::Node> Base::makeSky(const OsgConfig& config) {
    // Create a skybox or skydome using a large sphere or cube with a sky texture

    // Create a large sphere
    double radius = 5000.0;
    auto skySphere = vsg::Geometry::create();

    // Create geometry for the sphere
    // (You need to generate sphere vertices, normals, and texture coordinates)

    // Load sky texture
    auto skyTexture = vsg::read_cast<vsg::Data>("Images/sky.jpg");
    if (!skyTexture) {
        std::cerr << "Failed to load sky texture." << std::endl;
        return nullptr;
    }

    // Create texture sampler and descriptor
    auto sampler = vsg::Sampler::create();
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    auto descriptorImage = vsg::DescriptorImage::create(sampler, skyTexture, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    // Set up shaders, pipeline, etc.
    // (You need to create shader stages for the sky and configure the pipeline)

    // Return the sky node
    return skySphere;
}

vsg::ref_ptr<vsg::Node> Base::makeGround(const OsgConfig& config) {
    // Create a ground plane

    double size = 1000.0;
    auto groundGeometry = vsg::Geometry::create();

    // Define vertices
    auto vertices = vsg::vec3Array::create({
        {-size, -size, 0.0},
        { size, -size, 0.0},
        { size,  size, 0.0},
        {-size,  size, 0.0}
    });

    // Define texture coordinates
    double texScale = 0.1;
    auto texCoords = vsg::vec2Array::create({
        {0.0, 0.0},
        {texScale * size, 0.0},
        {texScale * size, texScale * size},
        {0.0, texScale * size}
    });

    // Define indices
    auto indices = vsg::ushortArray::create({0, 1, 2, 2, 3, 0});

    // Assign arrays to geometry
    groundGeometry->arrays = vsg::DataList{vertices, texCoords};
    groundGeometry->indices = indices;

    // Load ground texture
    auto groundTexture = vsg::read_cast<vsg::Data>(this->groundTexture);
    if (!groundTexture) {
        std::cerr << "Failed to load ground texture." << std::endl;
        return nullptr;
    }

    // Create texture sampler and descriptor
    auto sampler = vsg::Sampler::create();
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    auto descriptorImage = vsg::DescriptorImage::create(sampler, groundTexture, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    // Create the graphics pipeline for the ground
    // (You need to set up shaders, pipeline layout, descriptor sets, etc.)

    // Create the state group or bind descriptor sets
    // (Attach the pipeline and descriptors to the ground geometry)

    // Return the ground node
    return groundGeometry;
}

vsg::ref_ptr<vsg::Node> Base::createHUD(OsgScene* scene, const OsgConfig& config) {
    // Implement HUD creation using VSG
    // Since VSG does not have built-in text rendering, you may need to use a text rendering library
    // Alternatively, you can render text using textures or custom shaders

    // Placeholder: Return an empty group for HUD
    hudNode = vsg::Group::create();
    return hudNode;
}

void Base::makeLights(vsg::ref_ptr<vsg::Group> node, const OsgConfig& config) {
    // Create a directional light
    auto light = vsg::Light::create();
    light->name = "MainLight";
    light->position = {0.0, 0.0, 1.0, 0.0}; // Directional light pointing down
    light->diffuse = {1.0, 1.0, 1.0, 1.0};
    light->specular = {1.0, 1.0, 1.0, 1.0};
    light->ambient = {0.5, 0.5, 0.5, 1.0};

    // Create a state group or bind the light to the scene
    // In VSG, you need to manage lighting through shaders and descriptor sets

    // Placeholder: Add the light as a child node (for scene graph completeness)
    node->addChild(light);
}

vsg::ref_ptr<vsg::Node> Base::createShadowedScene(vsg::ref_ptr<vsg::Node> sceneToShadow,
                                                  vsg::ref_ptr<vsg::Light> lightSource,
                                                  int shadowType) {
    // Implement shadow mapping techniques using shaders and render passes
    // This is an advanced topic and requires setting up depth maps, framebuffers, and appropriate shaders

    // For this example, we'll return the scene without shadows
    return sceneToShadow;
}

void Base::setCaption(const std::string& caption) {
    this->caption = caption;
    // Update HUD if necessary
}

void Base::setTitle(const std::string& title) {
    this->title = title;
    // Update HUD if necessary
}

HUDStatisticsManager* Base::getHUDSM() {
    if (!hUDStatisticsManager) {
        // Create HUDStatisticsManager
        hUDStatisticsManager = new HUDStatisticsManager();
        // Register callbacks or perform initialization
    }
    return hUDStatisticsManager;
}

void Base::setTimeStats(double time, double realtimefactor,
                        double truerealtimefactor, bool pause) {
    // Update time stats on HUD
    // Implement text rendering in VSG
}

void Base::changeShadowTechnique() {
    // Implement shadow technique switching if shadows are implemented
}

int Base::contains(char** list, int len, const char* str) {
    for (int i = 0; i < len; i++) {
        if (strcmp(list[i], str) == 0)
            return i + 1;
    }
    return 0;
}

} // namespace lpzrobots