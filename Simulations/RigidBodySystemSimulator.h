#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2
#define SPHERERADIUS 0.01f

struct Spring {

    Spring(int masspoint1, int masspoint2, float initialLength,
        std::array<Vec3, 2>& relativeApplicationOffsets)
        : _masspoint1(masspoint1), _masspoint2(masspoint2),
        _initialLength(initialLength),
        _relativeApplicationOffsets(relativeApplicationOffsets) {}

    int _masspoint1;
    int _masspoint2;
    float _initialLength;
    std::array<Vec3, 2> _relativeApplicationOffsets;
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
    void onKey(UINT nChar);

    void setGravity(float g);

    // rigibody stuff
    void setMass(float mass);
    int getNumberOfRigidBodies();
    Vec3 getPositionOfRigidBody(int i);
    Vec3 getLinearVelocityOfRigidBody(int i);
    Vec3 getAngularVelocityOfRigidBody(int i);
    void applyForceOnBody(int i, Vec3 loc, Vec3 force);
    size_t addRigidBody(Vec3 position, Vec3 size, float mass, bool fixed = false,
        bool isSphere = false);
    size_t addRigidBodyInternal(Vec3 position, Vec3 size, float mass,
        std::vector<RigidBody>& storage,
        bool fixed = false, bool isSphere = false);
    void setOrientationOf(int i, Quat orientation);
    void setVelocityOf(int i, Vec3 velocity);
    Mat4 calcWorldMatrix(RigidBody& rb);
    void calculateImpulse(CollisionInfo& colInfo, RigidBody& rbA, RigidBody& rbB);
    bool calcRayAABBIntersection(RigidBody& rb, Vec3 rayOrigin, Vec3 rayDirection,
        float& out);
    bool calcRaySphereIntersection(RigidBody& rb, Vec3 rayOrigin,
        Vec3 rayDirection, float& out);

    // mass spring system stuff
    void setStiffness(float stiffness);
    void setDampingFactor(float damping);

    void addSpring(int masspoint1, int masspoint2, float initialLength,
        std::array<Vec3, 2>& relativeApplicationOffsets);
    int getNumberOfSprings();
    void computeForces(std::vector<RigidBody>& rigidBodies);
    void applyDamping(std::vector<RigidBody>& rigidBodies);

    // helpers
    float degToRad(float degree);
    void createSpringMesh(size_t m, size_t n, double spacing, Vec3 pos,
        double scale, double mass);
    void createGrid(size_t rows, size_t columns);
    CollisionInfo checkCollisionSphere(RigidBody& rbA, RigidBody& rbB);
    CollisionInfo checkCollisionSphereBox(RigidBody& rbA, RigidBody& rbB);
    Vec3 mulMat4Vec3(Mat4& mat, Vec3& vec);

private:
    // UI Attributes
    Point2D _mouse;
    Point2D _trackmouse;
    Point2D _oldtrackmouse;
    bool _visualizeSpringForces = false;
    float _springForceScale = 1.5f;

    // Demo 4 game
    size_t _demo4plane = 0;
    bool _demo4isRunning = false;
    float _demo4minVelocity = std::numeric_limits<float>::infinity();
    float _demo4currentVelocity = std::numeric_limits<float>::infinity();

    // Attributes
    float _gravity = 0;

    // Rigidbody parameters
    float _mass;
    float _massGridSpheres = 0.1;
    float _c = 1.;
    float _forceFactor = 30.;
    float _sphereRadius = 0.1;
    float _rbSize = 0.1;

    // Rigid Bodies
    std::vector<RigidBody> _rigidBodies;

    // Spring parameters
    float _stiffness;
    float _damping;

    // Springs
    std::vector<Spring> _springs;

    // tmp
    Vec3 _rayOrigin;
    Vec3 _rayDirection;
    Vec3 _intersectionWithGroundPlane;
};
#endif