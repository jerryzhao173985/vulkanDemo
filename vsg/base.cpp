// base.cpp - VSG Version

#include "base.h"
#include "primitive.h"
#include "pos.h"
#include "odehandle.h"
#include "globaldata.h"
#include <iostream>

namespace lpzrobots {

Base::Base(const std::string& caption)
    : caption(caption), title(""), groundTexture("Images/whiteground.jpg"),
      hUDStatisticsManager(nullptr), shadowTexSize(1024), useNVidia(true) {
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
    // Assign a dummy primitive to the ground plane to have substance (material) support
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
    scene->root = vsg::Group::create();

    // Create the world group
    scene->world = vsg::Group::create();

    // Create the scene group
    scene->scene = vsg::Group::create();

    // Create ground
    scene->groundScene = makeGround(config);

    // Add ground to the world
    scene->world->addChild(scene->groundScene);

    // Add scene to the world
    scene->world->addChild(scene->scene);

    // Add world to root
    scene->root->addChild(scene->world);

    // Create a dummy group to prevent nodes from being deleted
    dummy = vsg::Group::create();
    dummy->addChild(scene->world);
}

vsg::ref_ptr<vsg::Node> Base::makeSky(const OsgConfig& config) {
    // Implement sky creation if needed
    return nullptr;
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
    double texScale = 0.2;
    auto texCoords = vsg::vec2Array::create({
        {-texScale * size, -texScale * size},
        { texScale * size, -texScale * size},
        { texScale * size,  texScale * size},
        {-texScale * size,  texScale * size}
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

    // Create texture image
    auto sampler = vsg::Sampler::create();
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    auto descriptorImage = vsg::DescriptorImage::create(sampler, groundTexture, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    // Set up shaders, pipeline, etc.

    // Return the ground node
    return groundGeometry;
}

void Base::makeLights(vsg::ref_ptr<vsg::Group> node, const OsgConfig& config) {
    // Create a directional light
    auto light = vsg::Light::create();
    light->name = "MainLight";
    light->position = {1.0, 1.0, 1.0, 0.0}; // Directional light
    light->diffuse = {1.0, 1.0, 1.0, 1.0};
    light->specular = {1.0, 1.0, 1.0, 1.0};
    light->ambient = {0.3, 0.3, 0.3, 1.0};

    // Add the light to the scene graph
    node->addChild(light);
}

vsg::ref_ptr<vsg::Node> Base::createShadowedScene(vsg::ref_ptr<vsg::Node> sceneToShadow,
                                                  vsg::ref_ptr<vsg::Light> lightSource,
                                                  int shadowType) {
    // Implement shadow mapping techniques
    // VSG does not have built-in shadow support like OSG's osgShadow
    // You need to implement shadow mapping using shaders and render passes
    // For the purpose of this example, we'll return the scene without shadows

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