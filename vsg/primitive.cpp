// primitive.cpp - VSG Version

#include "primitive.h"
#include "pos.h"
#include "pose.h"
#include "substance.h"
#include "odehandle.h"
#include "globaldata.h"
#include <iostream>

namespace lpzrobots {

int globalNumVelocityViolations = 0;
bool Primitive::destroyGeom = true;

// Helper functions implementation
Pose vsgPose(dGeomID geom) {
    const dReal* pos = dGeomGetPosition(geom);
    const dReal* rot = dGeomGetRotation(geom);
    return vsgPose(pos, rot);
}

Pose vsgPose(dBodyID body) {
    const dReal* pos = dBodyGetPosition(body);
    const dReal* rot = dBodyGetRotation(body);
    return vsgPose(pos, rot);
}

Pose vsgPose(const double* position, const double* rotation) {
    vsg::dmat4 mat(
        rotation[0], rotation[1], rotation[2], 0.0,
        rotation[4], rotation[5], rotation[6], 0.0,
        rotation[8], rotation[9], rotation[10], 0.0,
        position[0], position[1], position[2], 1.0
    );
    return Pose(mat);
}

void odeRotation(const Pose& pose, dMatrix3& odematrix) {
    vsg::dmat4 mat = pose.toMatrix();
    // Extract rotation components
    odematrix[0] = mat[0][0];
    odematrix[1] = mat[1][0];
    odematrix[2] = mat[2][0];
    odematrix[3] = 0.0;
    odematrix[4] = mat[0][1];
    odematrix[5] = mat[1][1];
    odematrix[6] = mat[2][1];
    odematrix[7] = 0.0;
    odematrix[8] = mat[0][2];
    odematrix[9] = mat[1][2];
    odematrix[10] = mat[2][2];
    odematrix[11] = 0.0;
}

// Primitive class implementation

Primitive::Primitive()
    : geom(0), body(0), mode(0), substanceManuallySet(false), numVelocityViolations(0) {
}

Primitive::~Primitive() {
    if (destroyGeom && geom)
        dGeomDestroy(geom);
    if (body && ((mode & _Transform) == 0))
        dBodyDestroy(body);
}

void Primitive::attachGeomAndSetColliderFlags() {
    if (mode & Body) {
        dGeomSetBody(geom, body);
        dGeomSetCategoryBits(geom, Dyn);
        dGeomSetCollideBits(geom, ~0x0); // Collides with everything
    } else {
        dGeomSetCategoryBits(geom, Stat);
        dGeomSetCollideBits(geom, ~Stat);
    }
    if (mode & _Child) {
        dGeomSetCategoryBits(geom, Dyn);
        dGeomSetCollideBits(geom, ~0x0); // Collides with everything
    }
    dGeomSetData(geom, (void*)this); // Set primitive as geom data
}

void Primitive::setColor(const Color& color) {
    // Implement color setting for VSG node if necessary
}

void Primitive::setColor(const std::string& color) {
    // Implement color lookup and setting
}

void Primitive::setTexture(const std::string& filename) {
    // Implement texture setting
}

void Primitive::setTexture(const TextureDescr& texture) {
    // Implement texture setting with descriptor
}

void Primitive::setTexture(int surface, const TextureDescr& texture) {
    // Implement texture setting for specific surface
}

void Primitive::setTextures(const std::vector<TextureDescr>& textures) {
    // Implement setting multiple textures
}

void Primitive::setPosition(const Pos& pos) {
    if (body) {
        dBodySetPosition(body, pos.x(), pos.y(), pos.z());
    } else if (geom) {
        dGeomSetPosition(geom, pos.x(), pos.y(), pos.z());
    }
    update();
}

void Primitive::setPose(const Pose& pose) {
    if (body) {
        vsg::dvec3 pos = pose.getTrans();
        dBodySetPosition(body, pos.x, pos.y, pos.z);
        vsg::dquat q = pose.getRotate();
        dQuaternion quat = { q.w, q.x, q.y, q.z };
        dBodySetQuaternion(body, quat);
    } else if (geom) {
        vsg::dvec3 pos = pose.getTrans();
        dGeomSetPosition(geom, pos.x, pos.y, pos.z);
        vsg::dquat q = pose.getRotate();
        dQuaternion quat = { q.w, q.x, q.y, q.z };
        dGeomSetQuaternion(geom, quat);
    } else {
        assert(false && "Call setPose only after initialization");
    }
    update();
}

Pos Primitive::getPosition() const {
    if (geom) {
        const dReal* pos = dGeomGetPosition(geom);
        return Pos(pos[0], pos[1], pos[2]);
    } else if (body) {
        const dReal* pos = dBodyGetPosition(body);
        return Pos(pos[0], pos[1], pos[2]);
    } else {
        return Pos(0, 0, 0);
    }
}

Pose Primitive::getPose() const {
    if (geom) {
        const dReal* pos = dGeomGetPosition(geom);
        const dReal* rot = dGeomGetRotation(geom);
        return vsgPose(pos, rot);
    } else if (body) {
        const dReal* pos = dBodyGetPosition(body);
        const dReal* rot = dBodyGetRotation(body);
        return vsgPose(pos, rot);
    } else {
        return Pose::identity();
    }
}

Pos Primitive::getVel() const {
    if (body) {
        const dReal* vel = dBodyGetLinearVel(body);
        return Pos(vel[0], vel[1], vel[2]);
    } else {
        return Pos(0, 0, 0);
    }
}

Pos Primitive::getAngularVel() const {
    if (body) {
        const dReal* vel = dBodyGetAngularVel(body);
        return Pos(vel[0], vel[1], vel[2]);
    } else {
        return Pos(0, 0, 0);
    }
}

bool Primitive::applyForce(vsg::dvec3 force) {
    return applyForce(force.x, force.y, force.z);
}

bool Primitive::applyForce(double x, double y, double z) {
    if (body) {
        dBodyAddForce(body, x, y, z);
        return true;
    } else {
        return false;
    }
}

bool Primitive::applyTorque(vsg::dvec3 torque) {
    return applyTorque(torque.x, torque.y, torque.z);
}

bool Primitive::applyTorque(double x, double y, double z) {
    if (body) {
        dBodyAddTorque(body, x, y, z);
        return true;
    } else {
        return false;
    }
}

void Primitive::setMass(double mass, double cgx, double cgy, double cgz,
                        double I11, double I22, double I33,
                        double I12, double I13, double I23) {
    dMass m;
    dMassSetParameters(&m, mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23);
    dBodySetMass(body, &m);
}

bool Primitive::limitLinearVel(double maxVel) {
    if (!body) return false;
    const dReal* vel = dBodyGetLinearVel(body);
    double vellen = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    if (vellen > maxVel*maxVel) {
        numVelocityViolations++;
        globalNumVelocityViolations++;
        double scaling = sqrt(vellen) / maxVel;
        dBodySetLinearVel(body, vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
        return true;
    } else {
        return false;
    }
}

bool Primitive::limitAngularVel(double maxVel) {
    if (!body) return false;
    const dReal* vel = dBodyGetAngularVel(body);
    double vellen = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    if (vellen > maxVel*maxVel) {
        numVelocityViolations++;
        globalNumVelocityViolations++;
        double scaling = sqrt(vellen) / maxVel;
        dBodySetAngularVel(body, vel[0]/scaling, vel[1]/scaling, vel[2]/scaling);
        return true;
    } else {
        return false;
    }
}

void Primitive::decelerate(double factorLin, double factorAng) {
    if (!body) return;
    Pos vel;
    if (factorLin != 0) {
        vel = getVel();
        applyForce(vel * (-factorLin));
    }
    if (factorAng != 0) {
        vel = getAngularVel();
        applyTorque(vel * (-factorAng));
    }
}

vsg::dvec3 Primitive::toLocal(const vsg::dvec3& pos) const {
    Pose invPose = getPose().inverse();
    return invPose.transformPoint(pos);
}

vsg::dvec4 Primitive::toLocal(const vsg::dvec4& axis) const {
    Pose invPose = getPose().inverse();
    return invPose * axis;
}

vsg::dvec3 Primitive::toGlobal(const vsg::dvec3& pos) const {
    return getPose().transformPoint(pos);
}

vsg::dvec4 Primitive::toGlobal(const vsg::dvec4& axis) const {
    return getPose() * axis;
}

void Primitive::setSubstance(const Substance& substance) {
    this->substance = substance;
    substanceManuallySet = true;
}

bool Primitive::store(FILE* f) const {
    Pose pose = getPose();
    Pos vel = getVel();
    Pos avel = getAngularVel();

    if (fwrite(pose.data(), sizeof(Pose::value_type), 16, f) == 16)
        if (fwrite(vel.data(), sizeof(Pos::value_type), 3, f) == 3)
            if (fwrite(avel.data(), sizeof(Pos::value_type), 3, f) == 3)
                return true;
    return false;
}

bool Primitive::restore(FILE* f) {
    Pose pose;
    Pos vel;
    Pos avel;

    if (fread(pose.data(), sizeof(Pose::value_type), 16, f) == 16)
        if (fread(vel.data(), sizeof(Pos::value_type), 3, f) == 3)
            if (fread(avel.data(), sizeof(Pos::value_type), 3, f) == 3) {
                setPose(pose);
                if (body) {
                    dBodySetLinearVel(body, vel.x(), vel.y(), vel.z());
                    dBodySetAngularVel(body, avel.x(), avel.y(), avel.z());
                }
                return true;
            }
    std::cerr << "Primitive::restore: cannot read primitive from data" << std::endl;
    return false;
}

// Implementation of Plane

Plane::Plane() {
    vsgPlane = nullptr;
}

Plane::~Plane() {
    // Cleanup if necessary
}

void Plane::init(const OdeHandle& odeHandle, double mass,
                 const OsgHandle& osgHandle,
                 char mode) {
    assert(mode & Body || mode & Geom);
    this->mode = mode;
    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    if (mode & Geom) {
        geom = dCreatePlane(odeHandle.space, 0, 0, 1, 0);
        attachGeomAndSetColliderFlags();
    }
    if (mode & Draw) {
        // Create a VSG node representing an infinite plane
        // Since VSG does not support infinite planes, we can create a large quad
        double size = 1000.0;
        auto planeGeometry = vsg::Geometry::create();

        // Define vertices
        auto vertices = vsg::vec3Array::create({
            {-size, -size, 0.0},
            { size, -size, 0.0},
            { size,  size, 0.0},
            {-size,  size, 0.0}
        });

        // Define indices
        auto indices = vsg::ushortArray::create({0, 1, 2, 2, 3, 0});

        // Assign arrays to geometry
        planeGeometry->arrays = vsg::DataList{vertices};
        planeGeometry->indices = indices;

        // Set up shaders, pipeline, etc.
        // For simplicity, we can create a basic pipeline with a solid color

        // Store the plane node
        vsgPlane = planeGeometry;
    }
}

void Plane::update() {
    // Planes in ODE do not have a position or orientation
    // If needed, update the VSG node's transformation
}

vsg::ref_ptr<vsg::Node> Plane::getVSGNode() {
    return vsgPlane;
}

void Plane::setMass(double mass, bool density) {
    // Planes in ODE are static and do not have mass
}

// Implementation of Box

Box::Box(double lengthX, double lengthY, double lengthZ)
    : dimensions(lengthX, lengthY, lengthZ) {
    vsgBox = nullptr;
}

Box::Box(const vsg::dvec3& dim)
    : dimensions(dim) {
    vsgBox = nullptr;
}

Box::~Box() {
    // Cleanup if necessary
}

void Box::init(const OdeHandle& odeHandle, double mass,
               const OsgHandle& osgHandle,
               char mode) {
    assert((mode & Body) || (mode & Geom));
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;

    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    if (mode & Geom) {
        geom = dCreateBox(odeHandle.space, dimensions.x, dimensions.y, dimensions.z);
        attachGeomAndSetColliderFlags();
    }
    if (mode & Draw) {
        // Create a VSG node representing the box
        auto boxGeometry = vsg::Geometry::create();

        // Define vertices
        double hx = dimensions.x / 2.0;
        double hy = dimensions.y / 2.0;
        double hz = dimensions.z / 2.0;

        auto vertices = vsg::vec3Array::create({
            {-hx, -hy, -hz},
            { hx, -hy, -hz},
            { hx,  hy, -hz},
            {-hx,  hy, -hz},
            {-hx, -hy,  hz},
            { hx, -hy,  hz},
            { hx,  hy,  hz},
            {-hx,  hy,  hz}
        });

        // Define indices for the faces
        auto indices = vsg::ushortArray::create({
            // Bottom face
            0, 1, 2, 2, 3, 0,
            // Top face
            4, 5, 6, 6, 7, 4,
            // Front face
            0, 1, 5, 5, 4, 0,
            // Back face
            2, 3, 7, 7, 6, 2,
            // Left face
            0, 3, 7, 7, 4, 0,
            // Right face
            1, 2, 6, 6, 5, 1
        });

        // Assign arrays to geometry
        boxGeometry->arrays = vsg::DataList{vertices};
        boxGeometry->indices = indices;

        // Set up shaders, pipeline, etc.

        // Store the box node
        vsgBox = boxGeometry;
    }
}

void Box::update() {
    if (mode & Draw) {
        Pose pose = getPose();
        if (!vsgBox) return;

        // Update the transformation
        auto transform = vsg::MatrixTransform::create();
        transform->matrix = pose.toMatrix();
        transform->addChild(vsgBox);

        vsgBox = transform;
    }
}

vsg::ref_ptr<vsg::Node> Box::getVSGNode() {
    return vsgBox;
}

void Box::setMass(double mass, bool density) {
    if (body) {
        dMass m;
        if (density) {
            dMassSetBox(&m, mass, dimensions.x, dimensions.y, dimensions.z);
        } else {
            dMassSetBoxTotal(&m, mass, dimensions.x, dimensions.y, dimensions.z);
        }
        dBodySetMass(body, &m);
    }
}

// Implementation of Sphere

Sphere::Sphere(double radius)
    : radius(radius) {
    vsgSphere = nullptr;
}

Sphere::~Sphere() {
    // Cleanup if necessary
}

void Sphere::init(const OdeHandle& odeHandle, double mass,
                  const OsgHandle& osgHandle,
                  char mode) {
    assert(mode & Body || mode & Geom);
    if (!substanceManuallySet)
        substance = odeHandle.substance;
    this->mode = mode;

    if (mode & Body) {
        body = dBodyCreate(odeHandle.world);
        setMass(mass, mode & Density);
    }
    if (mode & Geom) {
        geom = dCreateSphere(odeHandle.space, radius);
        attachGeomAndSetColliderFlags();
    }
    if (mode & Draw) {
        // Create a VSG node representing the sphere
        auto sphereGeometry = vsg::Sphere::create(radius);

        // Set up shaders, pipeline, etc.

        // Store the sphere node
        vsgSphere = sphereGeometry;
    }
}

void Sphere::update() {
    if (mode & Draw) {
        Pose pose = getPose();
        if (!vsgSphere) return;

        // Update the transformation
        auto transform = vsg::MatrixTransform::create();
        transform->matrix = pose.toMatrix();
        transform->addChild(vsgSphere);

        vsgSphere = transform;
    }
}

vsg::ref_ptr<vsg::Node> Sphere::getVSGNode() {
    return vsgSphere;
}

void Sphere::setMass(double mass, bool density) {
    if (body) {
        dMass m;
        if (density) {
            dMassSetSphere(&m, mass, radius);
        } else {
            dMassSetSphereTotal(&m, mass, radius);
        }
        dBodySetMass(body, &m);
    }
}

} // namespace lpzrobots