#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
    m_iTestCase = 0;

    m_fMass = 1.f;
    m_fStiffness = 1.f;
    m_fDamping = 0.f;
    m_iIntegrator = EULER;
    m_externalForce = Vec3(0.f, 0.f, 0.f);

    reset();
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
    return "Demo1,Demo2,Demo3,Demo4,Demo5";
}

const char* MassSpringSystemSimulator::getIntegratorStr() {
    return "Euler,Leapfrog,Midpoint";
}

void MassSpringSystemSimulator::reset() {
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
    m_massPoints.clear();
    m_springs.clear();
    timestepOverwrite = false;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    TwType TW_TYPE_INTEGRATOR =
        TwDefineEnumFromString("Integrator", getIntegratorStr());
    switch (m_iTestCase) {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR,
            &m_iIntegrator, "");
    case 4:
    default:

        TwAddVarRW(DUC->g_pTweakBar, "Ground Collision", TW_TYPE_BOOLCPP,
            &useGroundCollision, "");
        TwAddVarRW(DUC->g_pTweakBar, "Bounce Factor", TW_TYPE_FLOAT,
            &m_fBounceFactor, "step=0.1 min=0.0");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping,
            "step=0.1 min=0.0");
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness,
            "step=0.1 min=0.01");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01");
        TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity,
            "min=0.0");
        break;
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        cout << "Demo1!\n";
        reset();
        computeDemoOne();
        break;
    case 1:
        cout << "Demo2!\n";
        reset();
        setupDemoTwo();
        break;
    case 2:
        cout << "Demo3!\n";
        reset();
        setupDemoThree();
        break;
    case 3:
        cout << "Demo4!\n";
        reset();
        setupDemoFour();
        break;
    case 4:
        cout << "Demo5!\n";
        reset();
        setupDemoFive();
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void MassSpringSystemSimulator::computeDemoOne() {
    timestepOverwrite = true;
    m_iTimestep = 0.1f;

    setMass(10.f);
    setStiffness(40.f);
    setDampingFactor(0.0f);
    useGroundCollision = false;
    setGravity(0.0f);
    setBounceFactor(0.0f);

    int mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    int mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);

    for (auto& p : m_massPoints) {
        p.clearForce();
    }

    computeForces(m_massPoints);
    integrateEuler(0.1);

    std::cout << "Results after one step of Euler Integration with deltaT=0.1:"
        << std::endl;
    std::cout << "Position of MP1: " << m_massPoints[mp1].m_position << std::endl;
    std::cout << "Position of MP2: " << m_massPoints[mp2].m_position << std::endl;
    std::cout << "Velocity of MP1: " << m_massPoints[mp1].m_velocity << std::endl;
    std::cout << "Velocity of MP2: " << m_massPoints[mp2].m_velocity << std::endl;

    reset();

    timestepOverwrite = true;
    m_iTimestep = 0.1f;

    setMass(10.f);
    setStiffness(40.f);
    setDampingFactor(0.0f);
    useGroundCollision = false;
    setGravity(0.0f);
    setBounceFactor(0.0f);

    mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);

    for (auto& p : m_massPoints) {
        p.clearForce();
    }

    computeForces(m_massPoints);

    integrateMidpoint(0.1);

    std::cout << "Results after one step of Midpoint Integration with deltaT=0.1:"
        << std::endl;
    std::cout << "Position of MP1: " << m_massPoints[mp1].m_position << std::endl;
    std::cout << "Position of MP2: " << m_massPoints[mp2].m_position << std::endl;
    std::cout << "Velocity of MP1: " << m_massPoints[mp1].m_velocity << std::endl;
    std::cout << "Velocity of MP2: " << m_massPoints[mp2].m_velocity << std::endl;

    reset();
}

void MassSpringSystemSimulator::setupDemoTwo() {
    timestepOverwrite = true;
    m_iTimestep = 0.005f;
    m_iIntegrator = EULER;

    setMass(10.f);
    setStiffness(40.f);
    setDampingFactor(0.0f);
    useGroundCollision = false;
    setGravity(0.0f);
    setBounceFactor(0.0f);

    int mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    int mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);
}

void MassSpringSystemSimulator::setupDemoThree() {
    timestepOverwrite = true;
    m_iTimestep = 0.005f;
    m_iIntegrator = MIDPOINT;

    setMass(10.f);
    setStiffness(40.f);
    setDampingFactor(0.0f);
    useGroundCollision = false;
    setGravity(0.0f);
    setBounceFactor(0.0f);

    int mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    int mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);
}

void MassSpringSystemSimulator::setupDemoFour() {

    if (!m_valuesForComplexDemoSet) {
        setMass(10.f);
        setStiffness(200.f);
        setDampingFactor(0.4f);
        useGroundCollision = true;
        setGravity(2.f);
        setBounceFactor(0.5f);
        m_valuesForComplexDemoSet = true;
        m_iIntegrator = EULER;
    }

    // Object 1 (arm with box)
    int p00 = addMassPoint(Vec3(-4.f, 2.f, 0.f), Vec3(0.f, 0.f, 0.f), true);
    int p01 = addMassPoint(Vec3(-5.f, 4.f, 2.f), Vec3(0.f, -0.1f, 0.f), false);
    int p02 = addMassPoint(Vec3(-4.f, 5.f, 2.f), Vec3(0.f, 0.f, -0.2f), false);
    int p03 = addMassPoint(Vec3(-6.f, 7.f, 2.f), Vec3(0.f, 0.f, 0.2f), false);
    int p04 = addMassPoint(Vec3(-4.f, 7.f, 2.f), Vec3(0.f, 0.f, 0.f), false);
    int p05 = addMassPoint(Vec3(-6.f, 5.f, 2.f), Vec3(0.f, 0.f, 0.f), false);
    addSpring(p00, p01, .5f);
    addSpring(p01, p02, 1.f);
    addSpring(p02, p05, 1.f);
    addSpring(p05, p03, 1.f);
    addSpring(p03, p04, 1.f);
    addSpring(p04, p02, 1.f);

    // Object 3
    int p20 = addMassPoint(Vec3(-1.f, 1.f, -1.f), Vec3(0.f, 0.f, 0.f), false);
    int p21 = addMassPoint(Vec3(-1.f, 1.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p22 = addMassPoint(Vec3(-1.f, 1.f, 1.f), Vec3(0.f, 0.f, 0.f), false);
    int p23 = addMassPoint(Vec3(0.f, 1.f, -1.f), Vec3(0.f, 0.f, 0.f), false);
    int p24 = addMassPoint(Vec3(0.f, 1.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p25 = addMassPoint(Vec3(0.f, 1.f, 1.f), Vec3(0.f, 0.f, 0.f), false);
    int p26 = addMassPoint(Vec3(1.f, 1.f, -1.f), Vec3(0.f, 0.f, 0.f), false);
    int p27 = addMassPoint(Vec3(1.f, 1.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p28 = addMassPoint(Vec3(1.f, 1.f, 1.f), Vec3(0.f, 0.f, 0.f), false);
    int p29 = addMassPoint(Vec3(-0.5f, 2.f, -0.5f), Vec3(0.f, 0.f, 0.f), false);
    int p210 = addMassPoint(Vec3(-0.5f, 2.f, 0.5f), Vec3(0.f, 0.f, 0.f), false);
    int p211 = addMassPoint(Vec3(0.5f, 2.f, -0.5f), Vec3(0.f, 0.f, 0.f), false);
    int p212 = addMassPoint(Vec3(0.5f, 2.f, 0.5f), Vec3(0.f, 0.f, 0.f), false);
    int p213 = addMassPoint(Vec3(0.f, 3.f, 0.f), Vec3(0.f, 0.f, 0.f), false);

    addSpring(p20, p21,
        sqrt(m_massPoints[p20].m_position.squaredDistanceTo(
            m_massPoints[p21].m_position)));
    addSpring(p20, p23,
        sqrt(m_massPoints[p20].m_position.squaredDistanceTo(
            m_massPoints[p23].m_position)));
    addSpring(p20, p24,
        sqrt(m_massPoints[p20].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p20, p29,
        sqrt(m_massPoints[p20].m_position.squaredDistanceTo(
            m_massPoints[p29].m_position)));

    addSpring(p21, p22,
        sqrt(m_massPoints[p21].m_position.squaredDistanceTo(
            m_massPoints[p22].m_position)));
    addSpring(p21, p23,
        sqrt(m_massPoints[p21].m_position.squaredDistanceTo(
            m_massPoints[p23].m_position)));
    addSpring(p21, p24,
        sqrt(m_massPoints[p21].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p21, p25,
        sqrt(m_massPoints[p21].m_position.squaredDistanceTo(
            m_massPoints[p25].m_position)));
    addSpring(p21, p29,
        sqrt(m_massPoints[p21].m_position.squaredDistanceTo(
            m_massPoints[p29].m_position)));
    addSpring(p21, p210,
        sqrt(m_massPoints[p21].m_position.squaredDistanceTo(
            m_massPoints[p210].m_position)));

    addSpring(p22, p25,
        sqrt(m_massPoints[p22].m_position.squaredDistanceTo(
            m_massPoints[p25].m_position)));
    addSpring(p22, p24,
        sqrt(m_massPoints[p22].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p22, p210,
        sqrt(m_massPoints[p22].m_position.squaredDistanceTo(
            m_massPoints[p210].m_position)));

    addSpring(p23, p24,
        sqrt(m_massPoints[p23].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p23, p26,
        sqrt(m_massPoints[p23].m_position.squaredDistanceTo(
            m_massPoints[p26].m_position)));
    addSpring(p23, p29,
        sqrt(m_massPoints[p23].m_position.squaredDistanceTo(
            m_massPoints[p29].m_position)));
    addSpring(p23, p211,
        sqrt(m_massPoints[p23].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));
    addSpring(p23, p27,
        sqrt(m_massPoints[p23].m_position.squaredDistanceTo(
            m_massPoints[p27].m_position)));

    addSpring(p25, p24,
        sqrt(m_massPoints[p25].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p25, p28,
        sqrt(m_massPoints[p25].m_position.squaredDistanceTo(
            m_massPoints[p28].m_position)));
    addSpring(p25, p27,
        sqrt(m_massPoints[p25].m_position.squaredDistanceTo(
            m_massPoints[p27].m_position)));
    addSpring(p25, p210,
        sqrt(m_massPoints[p25].m_position.squaredDistanceTo(
            m_massPoints[p210].m_position)));
    addSpring(p25, p212,
        sqrt(m_massPoints[p25].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));

    addSpring(p26, p24,
        sqrt(m_massPoints[p26].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p26, p27,
        sqrt(m_massPoints[p26].m_position.squaredDistanceTo(
            m_massPoints[p27].m_position)));
    addSpring(p26, p211,
        sqrt(m_massPoints[p26].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));

    addSpring(p27, p24,
        sqrt(m_massPoints[p27].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p27, p28,
        sqrt(m_massPoints[p27].m_position.squaredDistanceTo(
            m_massPoints[p28].m_position)));
    addSpring(p27, p211,
        sqrt(m_massPoints[p27].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));
    addSpring(p27, p212,
        sqrt(m_massPoints[p27].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));

    addSpring(p28, p24,
        sqrt(m_massPoints[p28].m_position.squaredDistanceTo(
            m_massPoints[p24].m_position)));
    addSpring(p28, p212,
        sqrt(m_massPoints[p28].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));

    addSpring(p29, p210,
        sqrt(m_massPoints[p29].m_position.squaredDistanceTo(
            m_massPoints[p210].m_position)));
    addSpring(p29, p211,
        sqrt(m_massPoints[p29].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));
    addSpring(p210, p212,
        sqrt(m_massPoints[p210].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));
    addSpring(p211, p212,
        sqrt(m_massPoints[p211].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));
    addSpring(p210, p211,
        sqrt(m_massPoints[p210].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));
    addSpring(p29, p212,
        sqrt(m_massPoints[p29].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));

    addSpring(p213, p29,
        sqrt(m_massPoints[p213].m_position.squaredDistanceTo(
            m_massPoints[p29].m_position)));
    addSpring(p213, p210,
        sqrt(m_massPoints[p213].m_position.squaredDistanceTo(
            m_massPoints[p210].m_position)));
    addSpring(p213, p211,
        sqrt(m_massPoints[p213].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));
    addSpring(p213, p212,
        sqrt(m_massPoints[p213].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));

    addSpring(p24, p29,
        sqrt(m_massPoints[p24].m_position.squaredDistanceTo(
            m_massPoints[p29].m_position)));
    addSpring(p24, p210,
        sqrt(m_massPoints[p24].m_position.squaredDistanceTo(
            m_massPoints[p210].m_position)));
    addSpring(p24, p211,
        sqrt(m_massPoints[p24].m_position.squaredDistanceTo(
            m_massPoints[p211].m_position)));
    addSpring(p24, p212,
        sqrt(m_massPoints[p24].m_position.squaredDistanceTo(
            m_massPoints[p212].m_position)));

    addSpring(p24, p213,
        sqrt(m_massPoints[p24].m_position.squaredDistanceTo(
            m_massPoints[p213].m_position)));
}

void MassSpringSystemSimulator::setupDemoFive() {
    m_valuesForComplexDemoSet = false;
    m_iIntegrator = LEAPFROG;
    setupDemoFour();
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
    // Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view
    // plane)
    if (m_iTestCase == 3 || m_iTestCase == 4) {
        // apply mouse input to top point of the pyramid
        Point2D mouseDiff;
        mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
        mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
        if (mouseDiff.x != 0 || mouseDiff.y != 0) {
            Mat4 worldViewInv =
                Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
            worldViewInv = worldViewInv.inverse();
            Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
            Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
            // find a proper scale!
            float inputScale = 0.001f;
            inputWorld = inputWorld * inputScale;
            m_massPoints[m_massPoints.size() - 1].m_position =
                m_vfMovableObjectFinalPos + inputWorld;
        }
        else {
            m_vfMovableObjectFinalPos =
                m_massPoints[m_massPoints.size() - 1].m_position;
        }
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
    // first check if we have overwritten the timestep. If so, the use the
    // internal
    if (timestepOverwrite) {
        timeStep = m_iTimestep;
    }
    // clear forces
    for (auto& p : m_massPoints) {
        p.clearForce();
    }
    // add gravity
    addGravity(m_massPoints);
    // force computation
    computeForces(m_massPoints);
    // damping
    applyDamping(m_massPoints);

    // integrate the timestep
    integrate(timeStep);

    // collision on floor
    if (useGroundCollision) {
        for (auto& p : m_massPoints) {
            if (p.m_position.y < -1.f) {
                // reset position from floor
                p.m_position.y = -0.9999f;
                // bounce back from floor
                p.m_velocity.y *= -m_fBounceFactor;
            }
        }
    }
}

void MassSpringSystemSimulator::drawFrame(
    ID3D11DeviceContext* pd3dImmediateContext) {
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,
        0.6 * Vec3(1.f, 1.f, 1.f));
    for (const auto& s : m_springs) {
        DUC->beginLine();
        DUC->drawLine(m_massPoints[s.m_iMasspoint1].m_position, Vec3(1.f, 0.f, 0.f),
            m_massPoints[s.m_iMasspoint2].m_position,
            Vec3(1.f, 0.f, 0.f));
        DUC->endLine();
    }
    for (const auto& p : m_massPoints) {
        DUC->drawSphere(p.m_position, Vec3(.1f, .1f, .1f));
    }
}

void MassSpringSystemSimulator::onClick(int x, int y) {
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) { m_fMass = mass; }

void MassSpringSystemSimulator::setStiffness(float stiffness) {
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
    m_fDamping = damping;
}

void MassSpringSystemSimulator::setGravity(float gravity) {
    m_fGravity = gravity;
}

void MassSpringSystemSimulator::setBounceFactor(float bounceFactor) {
    m_fBounceFactor = bounceFactor;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity,
    bool isFixed) {
    m_massPoints.push_back(MassPoint(position, velocity, isFixed));
    return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2,
    float initialLength) {
    m_springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
    return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() { return m_springs.size(); }

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
    return m_massPoints[index].m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
    return m_massPoints[index].m_velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
    for (auto& p : m_massPoints) {
        p.m_force += force;
    }
}

void MassSpringSystemSimulator::computeForces(
    std::vector<MassPoint>& massPoints) {
    for (auto& s : m_springs) {
        Vec3 d = massPoints[s.m_iMasspoint1].m_position -
            massPoints[s.m_iMasspoint2].m_position;
        float dist = sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z);
        Vec3 f1 = -m_fStiffness * (dist - s.m_fInitialLength) * (d / dist);
        Vec3 f2 = -f1;

        massPoints[s.m_iMasspoint1].m_force += f1;
        massPoints[s.m_iMasspoint2].m_force += f2;
    }
}

void MassSpringSystemSimulator::applyDamping(
    std::vector<MassPoint>& massPoints) {
    for (auto& p : massPoints) {
        p.m_force -= m_fDamping * p.m_velocity;
    }
}

void MassSpringSystemSimulator::addGravity(std::vector<MassPoint>& massPoints) {
    for (auto& p : massPoints) {
        p.m_force += m_fMass * Vec3(0.0f, -m_fGravity, 0.0f);
    }
}

void MassSpringSystemSimulator::integrate(float timeStep) {
    switch (m_iIntegrator) {
    case EULER:
        integrateEuler(timeStep);
        break;
    case LEAPFROG:
        integrateLeapfrog(timeStep);
        break;
    case MIDPOINT:
        integrateMidpoint(timeStep);
        break;
    default:
        break;
    }
}

void MassSpringSystemSimulator::integrateEuler(float timeStep) {
    for (auto& p : m_massPoints) {
        if (!p.m_isFixed) {
            // newPos = oldPos + deltaT * oldVel
            p.m_position += timeStep * p.m_velocity;
            // newVel = oldVel + deltaT * (acceleration(=force/mass))
            p.m_velocity += timeStep * ((p.m_force / m_fMass));
        }
    }
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep) {
    // first we create a copy of our masspoints using the assignment operator
    // and integrate a half-timestep for this copied mass points. We need to
    // create a copy since we need the initial positions and velocities in the
    // next step
    auto massPointsTmp = m_massPoints;
    for (auto& p : massPointsTmp) {
        if (!p.m_isFixed) {
            p.m_position += (0.5f * timeStep) * p.m_velocity;
            p.m_velocity += (0.5f * timeStep) * ((p.m_force / m_fMass));
        }
    }

    // update forces of points after integration with half of a timestep (note:
    // more costly as euler, since we need to update the forces here again)
    // first we have to clear the forces of our tmp array
    for (auto& p : massPointsTmp) {
        p.clearForce();
    }
    computeForces(massPointsTmp);
    applyDamping(massPointsTmp);

    // now do the integration for the full timestep
    for (auto i = 0; i < m_massPoints.size(); ++i) {
        if (!m_massPoints[i].m_isFixed) {
            m_massPoints[i].m_position += timeStep * massPointsTmp[i].m_velocity;
            m_massPoints[i].m_velocity +=
                timeStep * ((massPointsTmp[i].m_force / m_fMass));
        }
    }
}

void MassSpringSystemSimulator::integrateLeapfrog(float timeStep) {
    for (auto& p : m_massPoints) {
        if (!p.m_isFixed) {
            // newVel = oldVel + deltaT * (acceleration(=force/mass))
            p.m_velocity += timeStep * ((p.m_force / m_fMass));
            // newPos = oldPos + deltaT * oldVel
            p.m_position += timeStep * p.m_velocity;
        }
    }
}