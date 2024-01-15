#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2

struct MassPoint {

    MassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
        m_position = position;
        m_velocity = velocity;
        m_isFixed = isFixed;
        m_force = Vec3(0, 0, 0);
    }

    void clearForce() { m_force = Vec3(0, 0, 0); }

    Vec3 m_position;
    Vec3 m_velocity;
    Vec3 m_force;
    bool m_isFixed;
};

struct Spring {

    Spring(int masspoint1, int masspoint2, float initialLength) {
        m_iMasspoint1 = masspoint1;
        m_iMasspoint2 = masspoint2;
        m_fInitialLength = initialLength;
    }

    int m_iMasspoint1;
    int m_iMasspoint2;
    float m_fInitialLength;
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

    // ExtraFunctions
    int getNumberOfRigidBodies();
    Vec3 getPositionOfRigidBody(int i);
    Vec3 getLinearVelocityOfRigidBody(int i);
    Vec3 getAngularVelocityOfRigidBody(int i);
    void applyForceOnBody(int i, Vec3 loc, Vec3 force);
    void addRigidBody(Vec3 position, Vec3 size, int mass);
    void addRigidBodyInternal(Vec3 position, Vec3 size, int mass,
        std::vector<RigidBody>& storage);
    void setOrientationOf(int i, Quat orientation);
    void setVelocityOf(int i, Vec3 velocity);

    // mass spring system stuff
    void setMass(float mass);
    void setStiffness(float stiffness);
    void setDampingFactor(float damping);
    void setBounceFactor(float bounceFactor);
    int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
    void addSpring(int masspoint1, int masspoint2, float initialLength);
    int getNumberOfMassPoints();
    int getNumberOfSprings();
    Vec3 getPositionOfMassPoint(int index);
    Vec3 getVelocityOfMassPoint(int index);
    void applyExternalForce(Vec3 force);
    void computeForces(std::vector<MassPoint>& massPoints);
    void applyDamping(std::vector<MassPoint>& massPoints);

    Mat4 calcWorldMatrix(RigidBody& rb);
    void calculateImpulse(CollisionInfo& colInfo, RigidBody& rbA, RigidBody& rbB);
    void setGravity(float g);
    float degToRad(float degree);

private:
    // Attributes
    // add your RigidBodySystem data members, for e.g.,
    // RigidBodySystem * m_pRigidBodySystem;
    Vec3 m_externalForce;

    // UI Attributes
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;

    std::vector<RigidBody> rigidBodies;

    bool timestepOverwrite{ false };
    float ownTimestep;
    float c = 1.;
    float gravity = 0;
    bool gravitySet = false;

    // constant rigidbodies
    std::vector<RigidBody> constRigidBodies;

    float m_fMass;
    float m_fStiffness;
    float m_fDamping;
    float m_fBounceFactor{ 0.0f };

    // Mass Points
    std::vector<MassPoint> m_massPoints;

    // Springs
    std::vector<Spring> m_springs;
};
#endif