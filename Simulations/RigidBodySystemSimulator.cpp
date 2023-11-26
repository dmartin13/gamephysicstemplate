#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
    m_iTestCase = 0;
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}
const char* RigidBodySystemSimulator::getTestCasesStr() {
    return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    switch (m_iTestCase) {
    case 0:
        break;
    case 1:
        // TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32,
        // &m_iNumSpheres,
        //            "min=1");
        // TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT,
        // &m_fSphereSize,
        //            "min=0.01 step=0.01");
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        break;
    }
}

void RigidBodySystemSimulator::reset() {
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

    rigidBodies.clear();
}

void RigidBodySystemSimulator::drawFrame(
    ID3D11DeviceContext* pd3dImmediateContext) {}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        cout << "Demo1!\n";
        addRigidBody(Vec3(0., 0., 0.), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(0, Quat(0., 0., 90.));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1., 1., 0.));
        simulateTimestep(2.);
        std::cout << "linearVelocity: " << getLinearVelocityOfRigidBody(0) << std::endl;
        std::cout << "angularVelocity: " << getAngularVelocityOfRigidBody(0) << std::endl;
        break;
    case 1:
        cout << "Demo2!\n";
        break;
    case 2:
        cout << "Demo3!\n";
        break;
    case 3:
        cout << "Demo4!\n";
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    // Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view
    // plane) Point2D mouseDiff; mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    // mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    // if (mouseDiff.x != 0 || mouseDiff.y != 0)
    // {
    // 	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() *
    // DUC->g_camera.GetViewMatrix()); 	worldViewInv = worldViewInv.inverse();
    // Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
    // Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
    // 	// find a proper scale!
    // 	float inputScale = 0.001f;
    // 	inputWorld = inputWorld * inputScale;
    // 	m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
    // }
    // else {
    // 	m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
    // }
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
    // position and velocity update (Euler Step)
    for (auto& rb : rigidBodies) {
        rb.position += timeStep * rb.linearVelocity;
        rb.linearVelocity += timeStep * (rb.force / rb.mass);

        //rotation update
        rb.orientation += (Quat(rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z, 0) * rb.orientation) * (0.5 * timeStep);
        // bring back to unit length as mentioned in exercise sheet
        rb.orientation = rb.orientation.unit();

        //angular momentum
        rb.angularMomentum += timeStep * rb.torque;

        //angular velcocity
        // first calculate I^-1 = Rot_r * I_0^-1 * Ro_r^T
        Mat4 rot = rb.orientation.getRotMat();
        Mat4 rotT = rot;
        rotT.transpose();
        Mat4 InewInv = rot * rb.InvIntertiaTensor * rotT;

        rb.angularVelocity = InewInv * rb.angularMomentum;

        //reset forces and torque
        rb.force = 0;
        rb.torque = 0;
    }

}

void RigidBodySystemSimulator::onClick(int x, int y) {
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
    return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
    return rigidBodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
    return rigidBodies[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
    return rigidBodies[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
    // calculate and accumulate the torque
    rigidBodies[i].torque += cross((loc - rigidBodies[i].position), force);
    // accumulate the force
    rigidBodies[i].force += force;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size,
    int mass) {
    RigidBody rb;

    // init values from arguments
    rb.position = position;
    rb.mass = mass;
    rb.width = size.x;
    rb.height = size.y;
    rb.depth = size.z;

    // default inits
    rb.linearVelocity = Vec3();
    rb.angularVelocity = Vec3();
    rb.angularMomentum = Vec3();
    rb.force = Vec3();
    rb.torque = Vec3();
    rb.orientation = Quat(0., 0., 0., 1.);

    // calculate InVIntertaTensor (use version for rectengular boxes)
    // Note: Y and Z axis are flipped in contrast to the version from wikipedia
    float iit0 =
        1.f / (float(mass) * (1.f / 12.f) * (size.y * size.y + size.z * size.z));
    float iit1 =
        1.f / (float(mass) * (1.f / 12.f) * (size.x * size.x + size.z * size.z));
    float iit2 =
        1.f / (float(mass) * (1.f / 12.f) * (size.x * size.x + size.y * size.y));

    rb.InvIntertiaTensor =
        Mat4(iit0, 0., 0., 0., 0, iit1, 0., 0., 0., 0., iit2, 0., 0., 0., 0., 0.);

    rigidBodies.push_back(rb);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
    // make sure we use unit length orientation
    rigidBodies[i].orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
    rigidBodies[i].linearVelocity = velocity;
}