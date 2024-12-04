// base.h - Enhanced VSG Version

#ifndef __BASE_H
#define __BASE_H

#include <ode/ode.h>
#include <vsg/all.h>

#include "osghandle.h"
#include "odehandle.h"
#include "hudstatistics.h"
#include <selforg/configurable.h>

namespace lpzrobots {

    struct StatLineProperties {
        StatLineProperties(int fontSizeTime, int fontSizeText, const std::string& fontColor)
            : fontSizeTime(fontSizeTime), fontSizeText(fontSizeText), fontColor(fontColor) {
        }
        int fontSizeTime;
        int fontSizeText;
        std::string fontColor;
    };

    /**
     * Base class responsible for creating and managing the scene, physics, and visual elements.
     * Integrates ODE physics simulation with VSG rendering.
     */
    class Base : public Configurable
    {
    public:
        Base(const std::string& caption = "LpzRobots Simulator (Martius et al)");

        static const int PHYSICS_CALLBACKABLE = 1; //!< called each ode/physics step
        static const int GRAPHICS_CALLBACKABLE = 2; //!< called each vsg/draw step

        /// Create the ground plane in the physics simulation
        virtual void makePhysicsScene();

        /**
         * Creates the base scene graph with world, sky, ground, shadows, and HUD.
         * Stores it in the provided scene.
         * @param scene The VSG scene structure to populate
         * @param config The configuration for the OsgHandle (adjusted for VSG)
         */
        virtual void makeScene(OsgScene* scene, const OsgConfig& config);

        /**
         * Creates the sky dome or background for the scene.
         * Since VSG does not have a built-in sky dome, we can create a skybox or large sphere with a sky texture.
         * @param config The configuration for the OsgHandle (adjusted for VSG)
         * @return A VSG node representing the sky
         */
        virtual vsg::ref_ptr<vsg::Node> makeSky(const OsgConfig& config);

        /**
         * Creates the ground plane for the scene.
         * @param config The configuration for the OsgHandle (adjusted for VSG)
         * @return A VSG node representing the ground
         */
        virtual vsg::ref_ptr<vsg::Node> makeGround(const OsgConfig& config);

        /**
         * Creates HUD (Heads-Up Display) elements and adds them to the scene.
         * In VSG, we need to implement text rendering, possibly using external libraries or custom shaders.
         * @param scene The VSG scene structure to populate
         * @param config The configuration for the OsgHandle (adjusted for VSG)
         * @return A VSG node representing the HUD
         */
        virtual vsg::ref_ptr<vsg::Node> createHUD(OsgScene* scene, const OsgConfig& config);

        /**
         * Adds light to the scene.
         * @param node The node to which the light will be added
         * @param config The configuration for the OsgHandle (adjusted for VSG)
         */
        virtual void makeLights(vsg::ref_ptr<vsg::Group> node, const OsgConfig& config);

        /**
         * Creates a shadowed scene.
         * VSG does not have built-in shadow support, so we need to implement shadow mapping techniques using shaders.
         * @param sceneToShadow The scene node to apply shadows to
         * @param lightSource The light source casting the shadows
         * @param shadowType The type of shadow technique to use
         * @return A VSG node with shadows applied
         */
        virtual vsg::ref_ptr<vsg::Node> createShadowedScene(vsg::ref_ptr<vsg::Node> sceneToShadow,
                                                            vsg::ref_ptr<vsg::Light> lightSource,
                                                            int shadowType);

        /// Sets the texture file for the ground plane
        virtual void setGroundTexture(const std::string& filename) {
            this->groundTexture = filename;
        }

        /// Gets the substance properties of the ground plane
        virtual Substance getGroundSubstance();

        /// Sets the substance properties of the ground plane
        virtual void setGroundSubstance(const Substance& substance);

        /// Sets the caption displayed on the status line
        virtual void setCaption(const std::string& caption);

        /// Sets the title displayed on the status line
        virtual void setTitle(const std::string& title);

        /// Gets the properties of the status line
        virtual StatLineProperties getStatLineProperties() { return statlineprop; }

        /// Sets the properties of the status line; should be called before the scene is initialized
        virtual void setStatLineProperties(const StatLineProperties& statlineprop) {
            this->statlineprop = statlineprop;
        }

        /**
         * Creates HUDStatisticsManager and registers it for callbacks every step.
         * Does not display if the system is initialized with -nographics.
         * @return The HUDStatisticsManager instance
         */
        virtual HUDStatisticsManager* getHUDSM();

        virtual ~Base();

    protected:
        /**
         * Updates the time statistics displayed on the HUD.
         * @param time The current simulation time
         * @param realtimefactor The real-time factor of the simulation
         * @param truerealtimefactor The true real-time factor
         * @param pause Whether the simulation is paused
         */
        virtual void setTimeStats(double time, double realtimefactor,
                                  double truerealtimefactor, bool pause);

        /**
         * Changes the shadow technique used in the scene.
         * Since VSG does not have built-in shadow support, this function may involve changing shader parameters.
         */
        virtual void changeShadowTechnique();

        /// Cleans up resources created in makeScene and related functions
        virtual void base_close();

        // ODE ground plane
        dGeomID ground;

        // Handles for ODE and VSG configurations
        OsgHandle osgHandle;
        OdeHandle odeHandle;

        // Caption and title for the HUD/status line
        std::string caption;
        std::string title;

        // Ground texture file
        std::string groundTexture;

        // Root node for the scene
        vsg::ref_ptr<vsg::Group> rootNode;

        // Node for the HUD elements
        vsg::ref_ptr<vsg::Node> hudNode;

        // Properties of the status line
        StatLineProperties statlineprop;

        // Ground plane primitive
        Plane* plane;

        // HUD statistics manager
        HUDStatisticsManager* hUDStatisticsManager;

        // Shadow traversal masks (unused in VSG but kept for compatibility)
        int ReceivesShadowTraversalMask;
        int CastsShadowTraversalMask;

        // Configurable parameters
        int shadowTexSize;  // Shadow texture size (unused in VSG)
        bool useNVidia;     // Unused: if false, use ATI Radeon!

    public:
        // Helper function
        /// Returns the index+1 if the list contains the given string, or 0 if not
        static int contains(char** list, int len, const char* str);

    };

}

#endif // __BASE_H