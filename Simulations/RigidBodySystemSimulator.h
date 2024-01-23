#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2

struct Spring {

    Spring(int masspoint1, int masspoint2, float initialLength)
        : _masspoint1(masspoint1), _masspoint2(masspoint2),
        _initialLength(initialLength) {}

    int _masspoint1;
    int _masspoint2;
    float _initialLength;
};

struct RigidBody {
    float width;
    float height;
    float depth;
    float mass;
    Vec3 position;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Vec3 angularMomentum;
    Vec3 force;
    Vec3 torque;
    Mat4 InvIntertiaTensor;
    Quat orientation;
    Mat4 worldMatrix;
    bool fixed;
    bool isSphere = false;
};

class RigidBodySystemSimulator : public Simulator {
public:
    // Construtors
    RigidBodySystemSimulator();

    // Functions
    const char* getTestCasesStr();
    void initUI(DrawingUtilitiesClass* DUC);
    void reset();
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
    void notifyCaseChanged(int testCase);
    void externalForcesCalculations(float timeElapsed);
    void simulateTimestep(float timeStep);
    void onClick(int x, int y);
    void onMouse(int x, int y);

    void setGravity(float g);

    // rigibody stuff
    void setMass(float mass);
    int getNumberOfRigidBodies();
    Vec3 getPositionOfRigidBody(int i);
    Vec3 getLinearVelocityOfRigidBody(int i);
    Vec3 getAngularVelocityOfRigidBody(int i);
    void applyForceOnBody(int i, Vec3 loc, Vec3 force);
    size_t addRigidBody(Vec3 position, Vec3 size, int mass, bool fixed = false, bool isSphere = false);
    size_t addRigidBodyInternal(Vec3 position, Vec3 size, int mass,
        std::vector<RigidBody>& storage, bool fixed = false, bool isSphere = false);
    void setOrientationOf(int i, Quat orientation);
    void setVelocityOf(int i, Vec3 velocity);
    Mat4 calcWorldMatrix(RigidBody& rb);
    void calculateImpulse(CollisionInfo& colInfo, RigidBody& rbA, RigidBody& rbB);

    // mass spring system stuff
    void setStiffness(float stiffness);
    void setDampingFactor(float damping);
    void setBounceFactor(float bounceFactor);

    void addSpring(int masspoint1, int masspoint2, float initialLength);
    int getNumberOfSprings();
    void computeForces(std::vector<RigidBody>& rigidBodies);
    void applyDamping(std::vector<RigidBody>& rigidBodies);

    // helpers
    float degToRad(float degree);
    void createSpringMesh(size_t m, size_t n, double spacing, Vec3 pos,
        double scale);
    void createGrid(size_t rows, size_t columns);

private:
    // UI Attributes
    Point2D _mouse;
    Point2D _trackmouse;
    Point2D _oldtrackmouse;

    // Attributes
    Vec3 _externalForce;
    float _gravity = 0;

    // Rigidbody parameters
    float _c = 1.;

    // Rigid Bodies
    std::vector<RigidBody> _rigidBodies;
    std::vector<RigidBody> _constRigidBodies;

    // Spring parameters
    float _mass;
    float _stiffness;
    float _damping;
    float _bounceFactor{ 0.0f };

    // Springs
    std::vector<Spring> _springs;
};
#endif