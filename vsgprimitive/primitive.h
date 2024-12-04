// primitive.h - Revised VSG Version

#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#include <ode/ode.h>
#include <vsg/all.h>

#include <vector>
#include "pos.h"
#include "pose.h"
#include "substance.h"
#include "vsgprimitive.h"

namespace lpzrobots {

    // Helper functions
    Pose vsgPose(dGeomID geom);
    Pose vsgPose(dBodyID body);
    Pose vsgPose(const double* position, const double* rotation);
    void odeRotation(const Pose& pose, dMatrix3& odematrix);

    extern int globalNumVelocityViolations;

    /**
       Interface class for primitives represented in the physical and graphical world.
       This class integrates VSG and ODE, hiding most implementation details.
    */
    class Primitive : public vsg::Inherit<vsg::Object, Primitive> {
    public:
        /** Modes for initializing primitives */
        enum Modes {
            Body = 1,
            Geom = 2,
            Draw = 4,
            Density = 8,
            _Child = 16,
            _Transform = 32
        };
        enum Category { Dyn = 1, Stat = 2 };

        Primitive();
        virtual ~Primitive();

        /** Registers primitive in ODE and VSG.
            @param odeHandle Struct with ODE variables (world, space, etc.)
            @param mass Mass of the object in ODE (if withBody = true)
            @param osgHandle Struct with VSG variables (scene node, color, etc.)
            @param mode Combination of Modes flags.
         */
        virtual void init(const OdeHandle& odeHandle, double mass,
                          const OsgHandle& osgHandle,
                          char mode = Body | Geom | Draw) = 0;

        /** Updates the VSG nodes with ODE coordinates.
            Must be implemented by derived classes.
         */
        virtual void update() = 0;

        /// Returns the associated VSG node, or nullptr if none
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode() = 0;

        /// Sets the color of the primitive
        virtual void setColor(const Color& color);

        /// Sets the color using the color schema of osgHandle
        virtual void setColor(const std::string& color);

        /// Assigns a texture to the primitive
        virtual void setTexture(const std::string& filename);
        virtual void setTexture(const TextureDescr& texture);
        virtual void setTexture(int surface, const TextureDescr& texture);
        virtual void setTextures(const std::vector<TextureDescr>& textures);

        /// Sets the position of the primitive (orientation is preserved)
        virtual void setPosition(const Pos& pos);

        /// Sets the pose of the primitive
        virtual void setPose(const Pose& pose);

        /// Returns the position of the primitive
        virtual Pos getPosition() const;

        /// Returns the pose of the primitive
        virtual Pose getPose() const;

        /// Returns the linear velocity of the primitive
        virtual Pos getVel() const;

        /// Returns the angular velocity of the primitive
        virtual Pos getAngularVel() const;

        /** Applies a force to the primitive (in world coordinates)
            Returns true if successful.
        */
        virtual bool applyForce(vsg::dvec3 force);
        virtual bool applyForce(double x, double y, double z);

        /** Applies a torque to the primitive (in world coordinates)
            Returns true if successful.
        */
        virtual bool applyTorque(vsg::dvec3 torque);
        virtual bool applyTorque(double x, double y, double z);

        /** Sets the mass of the body (uniform)
            If density==true then mass is interpreted as a density
        */
        virtual void setMass(double mass, bool density = false) = 0;

        /** Sets full mass specification
            @param mass Total mass
            @param cgx, cgy, cgz Center of gravity coordinates
            @param I11, I22, I33, I12, I13, I23 Inertia tensor components
        */
        void setMass(double mass, double cgx, double cgy, double cgz,
                     double I11, double I22, double I33,
                     double I12, double I13, double I23);

        /// Returns the ODE geomID if available
        dGeomID getGeom() const { return geom; }

        /// Returns the ODE bodyID if available
        dBodyID getBody() const { return body; }

        /// Limits the linear velocity to maxVel
        bool limitLinearVel(double maxVel);

        /// Limits the angular velocity to maxVel
        bool limitAngularVel(double maxVel);

        /** Decelerates the primitive by applying a force and torque
            proportional to the velocities and given factors.
        */
        void decelerate(double factorLin, double factorAng);

        /// Transforms a point to local coordinates
        vsg::dvec3 toLocal(const vsg::dvec3& pos) const;

        /// Transforms a vector or axis to local coordinates
        vsg::dvec4 toLocal(const vsg::dvec4& axis) const;

        /// Transforms a point to global coordinates
        vsg::dvec3 toGlobal(const vsg::dvec3& pos) const;

        /// Transforms a vector or axis to global coordinates
        vsg::dvec4 toGlobal(const vsg::dvec4& axis) const;

        /** Sets whether to destroy the geom when the primitive is destroyed
            @param _destroyGeom If false, geom will not be destroyed
        */
        static void setDestroyGeomFlag(bool _destroyGeom) {
            destroyGeom = _destroyGeom;
        }

        /// Returns the number of velocity violations
        int getNumVelocityViolations() { return numVelocityViolations; }

        /// Sets the substance properties of the primitive
        void setSubstance(const Substance& substance);

        /* **** Storeable interface *******/
        virtual bool store(FILE* f) const;

        virtual bool restore(FILE* f);

    protected:
        /** Attaches geom to body (if any) and sets collision categories
            Assumes mode & Geom != 0
        */
        virtual void attachGeomAndSetColliderFlags();

    public:
        Substance substance; // Substance description

    protected:
        dGeomID geom;
        dBodyID body;
        char mode;
        bool substanceManuallySet;
        int numVelocityViolations; ///< Number of times the maximal velocity was exceeded

        static bool destroyGeom;

        // VSGPrimitive for graphical representation
        vsg::ref_ptr<VSGPrimitive> vsgPrimitive;
        vsg::ref_ptr<vsg::MatrixTransform> transformNode; // For convenience
    };

    /** Plane primitive */
    class Plane : public Primitive {
    public:
        Plane();
        virtual ~Plane();

        virtual void init(const OdeHandle& odeHandle, double mass,
                          const OsgHandle& osgHandle,
                          char mode = Geom | Draw);

        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();

        virtual void setMass(double mass, bool density = false);

    };

    /** Box primitive */
    class Box : public Primitive {
    public:
        Box(double lengthX, double lengthY, double lengthZ);
        Box(const vsg::dvec3& dim);

        virtual ~Box();

        virtual void init(const OdeHandle& odeHandle, double mass,
                          const OsgHandle& osgHandle,
                          char mode = Body | Geom | Draw);

        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();

        virtual void setMass(double mass, bool density = false);

    protected:
        vsg::dvec3 dimensions;
    };

    /** Sphere primitive */
    class Sphere : public Primitive {
    public:
        Sphere(double radius);
        virtual ~Sphere();

        virtual void init(const OdeHandle& odeHandle, double mass,
                          const OsgHandle& osgHandle,
                          char mode = Body | Geom | Draw);

        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();

        virtual void setMass(double mass, bool density = false);

    protected:
        double radius;
    };

    /** Additional primitives (Capsule, Cylinder, Ray, Mesh, Transform) would be defined similarly */

}
#endif // __PRIMITIVE_H