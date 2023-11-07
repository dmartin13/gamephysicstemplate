#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class MassSpringSystemSimulator : public Simulator {
public:
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

    // Construtors
    MassSpringSystemSimulator();

    // UI Functions
    const char* getTestCasesStr();
    const char* getIntegratorStr();
    void initUI(DrawingUtilitiesClass* DUC);
    void reset();
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
    void notifyCaseChanged(int testCase);
    void externalForcesCalculations(float timeElapsed);
    void simulateTimestep(float timeStep);
    void onClick(int x, int y);
    void onMouse(int x, int y);

    // Specific Functions
    void setMass(float mass);
    void setStiffness(float stiffness);
    void setDampingFactor(float damping);
    void setGravity(float gravity);
    void setBounceFactor(float bounceFactor);
    int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
    void addSpring(int masspoint1, int masspoint2, float initialLength);
    int getNumberOfMassPoints();
    int getNumberOfSprings();
    Vec3 getPositionOfMassPoint(int index);
    Vec3 getVelocityOfMassPoint(int index);
    void applyExternalForce(Vec3 force);
    void integrateEuler(float timeStep);
    void integrateMidpoint(float timeStep);
    void integrateLeapfrog(float timeStep);
    void integrate(float timeStep);
    void computeForces(std::vector<MassPoint>& massPoints);
    void applyDamping(std::vector<MassPoint>& massPoints);

    void computeDemoOne();
    void setupDemoTwo();
    void setupDemoThree();
    void setupDemoFour();

    // Do Not Change
    void setIntegrator(int integrator) { m_iIntegrator = integrator; }

private:
    // Data Attributes
    float m_fMass;
    float m_fStiffness;
    float m_fDamping;
    int m_iIntegrator;
    float m_iTimestep;
    bool timestepOverwrite{ false };
    bool useGroundCollision{ false };
    float m_fGravity{ 0.0f };
    float m_fBounceFactor{ 0.0f };

    // Mass Points
    std::vector<MassPoint> m_massPoints;

    // Springs
    std::vector<Spring> m_springs;

    // UI Attributes
    Vec3 m_externalForce;
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;
};
#endif