// base.h - VSG Version

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

    class Base : public Configurable
    {
    public:
        Base(const std::string& caption = "LpzRobots Simulator (Martius et al)");

        static const int PHYSICS_CALLBACKABLE = 1; //!< called each ode/physics step
        static const int GRAPHICS_CALLBACKABLE = 2; //!< called each vsg/draw step

        /// Create the ground plane
        virtual void makePhysicsScene();

        /** Creates the base scene graph with world, sky, floor, shadows, and HUD
            and stores it in the scene
         */
        virtual void makeScene(OsgScene* scene, const OsgConfig& config);

        virtual vsg::ref_ptr<vsg::Node> makeSky(const OsgConfig& config);
        virtual vsg::ref_ptr<vsg::Node> makeGround(const OsgConfig& config);

        /** Creates HUD and returns the camera for it; adds the HUD geode to the scene */
        virtual vsg::ref_ptr<vsg::Node> createHUD(OsgScene* scene, const OsgConfig& config);
        virtual void createHUDManager(vsg::ref_ptr<vsg::Node> geode);

        /** Adds light to the node */
        virtual void makeLights(vsg::ref_ptr<vsg::Group> node, const OsgConfig& config);

        /** Shadow types:
         * 1 - ShadowVolume
         * 2 - ShadowTexture
         * 3 - ParallelSplitShadowMap
         * 4 - SoftShadowMap
         * 5 - ShadowMap
         */
        virtual vsg::ref_ptr<vsg::Node> createShadowedScene(vsg::ref_ptr<vsg::Node> sceneToShadow,
                                                            vsg::ref_ptr<vsg::Light> lightSource,
                                                            int shadowType);

        virtual void setGroundTexture(const std::string& filename) {
            this->groundTexture = filename;
        }

        virtual Substance getGroundSubstance();
        virtual void setGroundSubstance(const Substance& substance);

        /// Sets the caption that is displayed at the right of the status line
        virtual void setCaption(const std::string& caption);

        /// Sets the title that is displayed in the center of the status line
        virtual void setTitle(const std::string& title);

        virtual StatLineProperties getStatLineProperties() { return statlineprop; }
        /// Sets the properties of the status line; do it before the scene is initialized
        virtual void setStatLineProperties(const StatLineProperties& statlineprop) {
            this->statlineprop = statlineprop;
        }

        /**
         * Creates HUDStatisticsManager and registers it for being called back every step.
         * Does not display if the system is initialized with -nographics.
         * @return the actual HUDStatisticsManager
         */
        virtual HUDStatisticsManager* getHUDSM();

        virtual ~Base();

    protected:
        virtual void setTimeStats(double time, double realtimefactor,
                                  double truerealtimefactor, bool pause);

        /**
         * Changes the currently used shadow technique.
         * The switch is realized between:
         * 0 - NoShadow
         * 3 - ParallelSplitShadowMap
         * 4 - SoftShadowMap
         * 5 - ShadowMap (simple)
         * Currently not supported by this function:
         * 1 - ShadowVolume
         * 2 - ShadowTexture
         */
        virtual void changeShadowTechnique();

        /// Deletes the stuff that is created in makeScene and similar functions
        virtual void base_close();

        dGeomID ground;

        OsgHandle osgHandle;
        // ODE globals
        OdeHandle odeHandle;
        std::string caption;
        std::string title;
        std::string groundTexture;

        vsg::ref_ptr<vsg::Group> dummy;

        vsg::ref_ptr<vsg::Node> hud;
        // Implement text rendering in VSG (e.g., using a text rendering library or custom shaders)
        StatLineProperties statlineprop;

        Primitive* plane;

        /// This manager provides methods for displaying statistics on the graphical window
        HUDStatisticsManager* hUDStatisticsManager;

        int ReceivesShadowTraversalMask;
        int CastsShadowTraversalMask;

        // Configurable parameters
        int shadowTexSize;  // Shadow texture size
        bool useNVidia;     // Unused: if false, use ATI Radeon!

    public:
        // Helper
        /// Returns the index+1 if the list contains the given string or 0 if not
        static int contains(char** list, int len, const char* str);

    };

}

#endif // __BASE_H