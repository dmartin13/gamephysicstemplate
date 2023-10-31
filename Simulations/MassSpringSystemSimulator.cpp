#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
    m_iTestCase = 0;

    m_fMass = 1.f;
    m_fStiffness = 1.f;
    m_fDamping = 0.f;
    m_iIntegrator = 0;

    m_externalForce = Vec3();
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
    return "Euler,Leapfrog,Midpoint";
}

void MassSpringSystemSimulator::reset() {
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    switch (m_iTestCase) {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    default:
        break;
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        cout << "Euler!\n";
        break;
    case 1:
        cout << "Leapfrog!\n";
        break;
    case 2:
        cout << "Midpoint!\n";
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
    // Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view
    // plane)
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0) {
        // Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() *
        // DUC->g_camera.GetViewMatrix()); worldViewInv = worldViewInv.inverse();
        // Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        // Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        // // find a proper scale!
        // float inputScale = 0.001f;
        // inputWorld = inputWorld * inputScale;
        // m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
    }
    else {
        // m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
    // update current setup for each frame
    switch (m_iTestCase) { // handling different cases
    case 0:
        // // rotate the teapot
        // m_vfRotate.x += timeStep;
        // if (m_vfRotate.x > 2 * M_PI) m_vfRotate.x -= 2.0f * (float)M_PI;
        // m_vfRotate.y += timeStep;
        // if (m_vfRotate.y > 2 * M_PI) m_vfRotate.y -= 2.0f * (float)M_PI;
        // m_vfRotate.z += timeStep;
        // if (m_vfRotate.z > 2 * M_PI) m_vfRotate.z -= 2.0f * (float)M_PI;

        break;
    default:
        break;
    }
}

void MassSpringSystemSimulator::drawSomeRandomObjects() {
    std::mt19937 eng;
    std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
    std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    for (int i = 0; i < 20; i++) {
        DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,
            0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
        DUC->drawSphere(Vec3(randPos(eng), randPos(eng), randPos(eng)),
            Vec3(0.05f, 0.05f, 0.05f));
    }
}

void MassSpringSystemSimulator::drawFrame(
    ID3D11DeviceContext* pd3dImmediateContext) {
    switch (m_iTestCase) {
    case 0:
        break;
    case 1:
        drawSomeRandomObjects();
        break;
    case 2:
        break;
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

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {}