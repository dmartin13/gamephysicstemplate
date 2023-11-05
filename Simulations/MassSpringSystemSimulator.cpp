#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
    m_iTestCase = 0;

    m_fMass = 1.f;
    m_fStiffness = 1.f;
    m_fDamping = 0.f;
    m_iIntegrator = 0;

    m_externalForce = Vec3();

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
        TwAddVarRW(DUC->g_pTweakBar, "Ground Collision", TW_TYPE_BOOLCPP,
            &useGroundCollision, "");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping,
            "min=0.0");
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness,
            "min=0.01");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01");
        TwAddVarRW(DUC->g_pTweakBar, "Gravitation", TW_TYPE_BOOLCPP,
            &useGraviatation, "");
        break;
    case 4:
        break;
    default:
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
    int mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    int mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);

    for (auto& p : m_massPoints) {
        p.clearForce();
    }

    for (auto& s : m_springs) {
        s.computeForces(m_massPoints);
    }

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
    mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);

    for (auto& p : m_massPoints) {
        p.clearForce();
    }

    for (auto& s : m_springs) {
        s.computeForces(m_massPoints);
    }

    integrateMidpoint(0.1);

    std::cout << "Results after one step of Midpoint Integration with deltaT=0.1:"
        << std::endl;
    std::cout << "Position of MP1: " << m_massPoints[mp1].m_position << std::endl;
    std::cout << "Position of MP2: " << m_massPoints[mp2].m_position << std::endl;
    std::cout << "Velocity of MP1: " << m_massPoints[mp1].m_velocity << std::endl;
    std::cout << "Velocity of MP2: " << m_massPoints[mp2].m_velocity << std::endl;
}

void MassSpringSystemSimulator::setupDemoTwo() {
    timestepOverwrite = true;
    m_iTimestep = 0.005f;
    m_iIntegrator = EULER;

    setMass(10.f);
    setStiffness(40.f);
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
    int mp1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
    int mp2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
    addSpring(mp1, mp2, 1.f);
}

void MassSpringSystemSimulator::setupDemoFour() {

    setMass(10.f);
    setStiffness(40.f);

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

    // Object 2 (star)
    int p10 = addMassPoint(Vec3(0.f, 4.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p11 = addMassPoint(Vec3(0.f, 6.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p12 = addMassPoint(Vec3(-sqrt(2.f), 4.f + sqrt(2.f), 0.f),
        Vec3(0.f, 0.f, 0.f), false);
    int p13 = addMassPoint(Vec3(-2.f, 4.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p14 = addMassPoint(Vec3(-sqrt(2.f), 4.f - sqrt(2.f), 0.f),
        Vec3(0.f, 0.f, 0.f), false);
    int p15 = addMassPoint(Vec3(0.f, 2.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p16 = addMassPoint(Vec3(sqrt(2.f), 4.f - sqrt(2.f), 0.f),
        Vec3(0.f, 0.f, 0.f), false);
    int p17 = addMassPoint(Vec3(2.f, 4.f, 0.f), Vec3(0.f, 0.f, 0.f), false);
    int p18 = addMassPoint(Vec3(sqrt(2.f), 4.f + sqrt(2.f), 0.f),
        Vec3(0.f, 0.f, 0.f), false);
    addSpring(p10, p11, 1.f);
    addSpring(p10, p12, 1.f);
    addSpring(p10, p13, 1.f);
    addSpring(p10, p14, 1.f);
    addSpring(p10, p15, 1.f);
    addSpring(p10, p16, 1.f);
    addSpring(p10, p17, 1.f);
    addSpring(p10, p18, 1.f);
    addSpring(p11, p12, 1.f);
    addSpring(p12, p13, 1.f);
    addSpring(p13, p14, 1.f);
    addSpring(p14, p15, 1.f);
    addSpring(p15, p16, 1.f);
    addSpring(p16, p17, 1.f);
    addSpring(p17, p18, 1.f);
    addSpring(p18, p11, 1.f);

    // Object 3 TBD
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
    // apply the gravity here?
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
    // first check if we have overwritten the timestep. If so, the use the
    // internal
    if (timestepOverwrite) {
        timeStep = m_iTimestep;
    }
    // update current setup for each frame
    switch (m_iTestCase) { // handling different cases
    case 0:
        // in Demo 1 we only print out the values to the console
        break;
    default:
        for (auto& p : m_massPoints) {
            p.clearForce();
            if (useGraviatation) {
                p.addGravity(Vec3(0.f, -9.81f * timeStep, 0.f));
            }
        }

        for (auto& s : m_springs) {
            s.computeForces(m_massPoints);
        }
        applyDamping(m_massPoints);

        integrate(timeStep);

        if (useGroundCollision) {
            for (auto& p : m_massPoints) {
                if (p.m_position.y < -1.f) {
                    p.m_position.y = -1.0001f;
                    // bounce back from ground
                    p.m_velocity.y *= -1.f;
                }
            }
        }

        break;
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity,
    bool isFixed) {
    m_massPoints.push_back(MassPoint(position, velocity, isFixed));
    return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2,
    float initialLength) {
    m_springs.push_back(
        Spring(masspoint1, masspoint2, initialLength, m_fStiffness));
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

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {}

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
            p.m_position += timeStep * p.m_velocity;
            p.m_velocity += timeStep * (p.m_force / m_fMass);
        }
    }
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep) {
    // first we create a copy of our masspoints using the assignment operator and
    // integrate a half-timestep for this copied mass points. We need to create a
    // copy since we need the initial positions and velocities in the next step
    auto massPointsTmp = m_massPoints;
    for (auto& p : massPointsTmp) {
        if (!p.m_isFixed) {
            p.m_position += (0.5f * timeStep) * p.m_velocity;
            p.m_velocity += (0.5f * timeStep) * (p.m_force / m_fMass);
        }
    }

    // update forces of points after integration with half of a timestep (note:
    // more costly as euler, since we need to update the forces here again)
    for (auto& s : m_springs) {
        s.computeForces(massPointsTmp);
    }
    applyDamping(massPointsTmp);

    // now do the integration for the full timestep
    for (auto i = 0; i < m_massPoints.size(); ++i) {
        if (!m_massPoints[i].m_isFixed) {
            m_massPoints[i].m_position += timeStep * massPointsTmp[i].m_velocity;
            m_massPoints[i].m_velocity +=
                timeStep * (massPointsTmp[i].m_force / m_fMass);
        }
    }
}

void MassSpringSystemSimulator::integrateLeapfrog(float timeStep) {}

void MassSpringSystemSimulator::applyDamping(
    std::vector<MassPoint>& massPoints) {
    for (auto& p : massPoints) {
        p.m_force -= m_fDamping * p.m_velocity;
    }
}