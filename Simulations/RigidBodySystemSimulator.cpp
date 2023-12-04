#include "RigidBodySystemSimulator.h"


RigidBodySystemSimulator::RigidBodySystemSimulator() {
    m_iTestCase = 0;
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


const char* RigidBodySystemSimulator::getTestCasesStr() {

    return "Demo 1,Demo 2,Demo 3,Demo 4";
}


void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {

    this->DUC = DUC;

    switch (m_iTestCase) {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        TwAddVarRW(DUC->g_pTweakBar, "Linear Damping", TW_TYPE_FLOAT, &dampingLinear, "min=0.0 step=0.1");
        TwAddVarRW(DUC->g_pTweakBar, "Angular Damping", TW_TYPE_FLOAT, &dampingAngular, "min=0.0 step=0.1");
        break;
    default:
        break;
    }
}



void RigidBodySystemSimulator::reset() {

    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

    // Clearing the rigidbodies std::vector
    rigidBodies.clear();
    fixedRigidBodies.clear();
    isFirstTime = true;
}



void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    std::mt19937 eng;
    std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
    switch (m_iTestCase)
    {
    case 0:
        //break;
    case 1:
        //break;
    case 2:
        //break;
    case 3:
        for (int i{ 0 }; i < rigidBodies.size(); i++)
        {
            DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
            DUC->drawRigidBody(rigidBodies.at(i).objToWorldMat);
        }
        for (int i{ 0 }; i < fixedRigidBodies.size(); i++)
        {
            DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
            DUC->drawRigidBody(fixedRigidBodies.at(i).objToWorldMat);
        }
        break;
    default:
        break;
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        cout << "Demo 1\n";
        reset();
        //spawnWalls();

        addRigidBody(Vec3(), Vec3(1.0f, 0.6f, 0.5f), 2);
        setOrientationOf(0, Quat(0.0f, 0.0f, degToRad(90.0f)));
        applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
        break;
    case 1:
        cout << "Demo 2\n";
        reset();
        //spawnWalls();

        addRigidBody(Vec3(), Vec3(1.0f, 0.6f, 0.5f), 2);
        setOrientationOf(0, Quat(0.0f, 0.0f, degToRad(90.0f)));
        applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));

        break;
    case 2:
        cout << "Demo 3\n";
        reset();
        //spawnWalls();

        addRigidBody(Vec3(), Vec3(1.0f, 0.6f, 0.5f), 2);
        setOrientationOf(0, Quat(0.0f, 0.0f, degToRad(90.0f)));

        addRigidBody(Vec3(-1.5f, 1.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2);
        setOrientationOf(1, Quat(0.0f, 0.0f, degToRad(45.0f)));
        setVelocityOf(1, Vec3(0.1f, -0.1f, 0.0f));
        break;
    case 3:
        cout << "Demo 4\n";
        reset();
        spawnWalls();

        addRigidBody(Vec3(), Vec3(1.0f, 0.6f, 0.5f), 2);
        setOrientationOf(0, Quat(0.0f, 0.0f, degToRad(90.0f)));

        addRigidBody(Vec3(-1.5f, 1.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2);
        setOrientationOf(1, Quat(0.0f, 0.0f, degToRad(45.0f)));
        setVelocityOf(1, Vec3(0.1f, -0.1f, 0.0f));

        addRigidBody(Vec3(0.5f, 0.5f, 0.0f), Vec3(0.25f), 1);
        addRigidBody(Vec3(0.5f, -0.5f, 0.0f), Vec3(0.25f), 1);
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}



void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    if (m_iTestCase >= 1)
    {
        // Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
        Point2D mouseDiff;
        mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
        mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
        if (mouseDiff.x != 0 || mouseDiff.y != 0)
        {
            Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
            worldViewInv = worldViewInv.inverse();
            Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
            Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
            // find a proper scale!
            float inputScale = 0.001f;
            inputWorld = inputWorld * inputScale;

            rigidBodies.at(0).force = m_vfMovableObjectFinalPos + inputWorld;
        }
        else {
            m_vfMovableObjectFinalPos = rigidBodies.at(0).force;
        }
    }
}



void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
    switch (m_iTestCase)
    {
    case 0:
        if (isFirstTime)
        {
            integrateVelPos(timeStep);
            std::cout << "v = " << getLinearVelocityOfRigidBody(0) << std::endl;
            std::cout << "w = " << getAngularVelocityOfRigidBody(0) << std::endl;
            isFirstTime = false;
        }
        break;
    case 1:
        //break;
    case 2:
        //break;
    case 3:
        integrateVelPos(timeStep);
        break;
    default:
        break;
    }

}

void RigidBodySystemSimulator::integrateVelPos(float timeStep) {
    for (auto& rb : rigidBodies)
    {
        if (!rb.isFixed)
        {
            // Apply damping
            if (m_iTestCase == 3)
                rb.lVelocity *= (1.0 - dampingLinear * timeStep);
            // Euler for linear velocity & position
            rb.position += timeStep * rb.lVelocity;
            rb.lVelocity += timeStep * (rb.force / rb.mass);

            
            // Rotation update
            rb.orientation += (Quat(rb.aVelocity.x, rb.aVelocity.y, rb.aVelocity.z, 0) * rb.orientation) * (0.5 * timeStep);
            rb.orientation = rb.orientation.unit();
            // Apply angular damping
            if (m_iTestCase == 3)
                rb.aMomentum *= (1.0 - dampingAngular * timeStep);
            // Angular momentum
            rb.aMomentum += timeStep * rb.torque;


            Mat4 rot = rb.orientation.getRotMat();
            Mat4 rotT = rot;
            rotT.transpose();
            Mat4 newInertiaTensorInv = rot * rb.inertiaTensorInv * rotT;

            rb.aVelocity = newInertiaTensorInv * rb.aMomentum;


            rb.force = 0;
            rb.torque = 0;

            rb.objToWorldMat = computeWorldMat(rb);
        }
    }

    if (m_iTestCase >= 2) {
        // Check collisions between dynamic rigid bodies
        for (int i = 0; i < rigidBodies.size(); i++)
            for (int j = i + 1; j < rigidBodies.size(); j++)
                checkAndApplyCollision(rigidBodies.at(i), rigidBodies.at(j));
    
        // Check collisions between dynamic and fixed rigid bodies
        for (int i = 0; i < rigidBodies.size(); i++)
            for (int j = 0; j < fixedRigidBodies.size(); j++)
                checkAndApplyCollision(rigidBodies.at(i), fixedRigidBodies.at(j));
    }   
}

void RigidBodySystemSimulator::checkAndApplyCollision(RigidBody& rbA, RigidBody& rbB) {
    CollisionInfo collisionInfo = checkCollisionSAT(rbA.objToWorldMat, rbB.objToWorldMat);
    if (collisionInfo.isValid) {
        applyImpulse(collisionInfo, rbA, rbB);
    }
}


void RigidBodySystemSimulator::applyImpulse(CollisionInfo& collisionInfo, RigidBody& rbA, RigidBody& rbB)
{
    const Vec3 normal = collisionInfo.normalWorld;
    const Vec3 collisionPoint = collisionInfo.collisionPointWorld;

    // Calculate relative positions
    const Vec3 relativePosA = collisionPoint - rbA.position;
    const Vec3 relativePosB = collisionPoint - rbB.position;

    // Calculate relative velocities
    const Vec3 relativeVelA = rbA.lVelocity + cross(rbA.aVelocity, relativePosA);
    const Vec3 relativeVelB = rbB.lVelocity + cross(rbB.aVelocity, relativePosB);
    const Vec3 relativeVelocity = relativeVelA - relativeVelB;

    // Check if objects are moving towards each other
    if (dot(relativeVelocity, normal) <= 0)
    {
        // Calculate impulse
        const double restitution = 0.7; // Adjust this value as needed
        const double numerator = -(1.0 + restitution) * dot(relativeVelocity, normal);
        const double invMassA = rbA.isFixed ? 0 : (1.0 / rbA.mass);
        const double invMassB = rbB.isFixed ? 0 : (1.0 / rbB.mass);

        const Vec3 angularImpulseA = cross(relativePosA, normal);
        const Vec3 angularImpulseB = cross(relativePosB, normal);

        const double denominator = invMassA + invMassB + dot(rbA.inertiaTensorInv * angularImpulseA, normal) + dot(rbB.inertiaTensorInv * angularImpulseB, normal);

        double impulseMagnitude = numerator / denominator;

        // Introduce an impulse threshold to avoid excessive forces
        const double impulseThreshold = 100.0; // Adjust this threshold to ensure stability
        if (impulseMagnitude > impulseThreshold)
        {
            // Clamp impulse to the threshold value
            impulseMagnitude = impulseThreshold;
        }

        // Apply linear impulses
        if (!rbA.isFixed)
        {
            rbA.lVelocity += (impulseMagnitude * normal) / rbA.mass;
            rbA.lVelocity = Vec3(min(static_cast<int>(rbA.lVelocity.X), MAX_LINEAR_VELOCITY),
                min(static_cast<int>(rbA.lVelocity.Y), MAX_LINEAR_VELOCITY), 
                min(static_cast<int>(rbA.lVelocity.Z), MAX_LINEAR_VELOCITY));
        }

        if (!rbB.isFixed)
        {
            rbB.lVelocity -= (impulseMagnitude * normal) / rbB.mass;
            rbB.lVelocity = Vec3(min(static_cast<int>(rbB.lVelocity.X), MAX_LINEAR_VELOCITY),
                min(static_cast<int>(rbB.lVelocity.Y), MAX_LINEAR_VELOCITY),
                min(static_cast<int>(rbB.lVelocity.Z), MAX_LINEAR_VELOCITY));
        }
        
        // Apply angular impulses
        if (!rbA.isFixed)
            rbA.aMomentum += cross(relativePosA, impulseMagnitude * normal);
        if (!rbB.isFixed)
            rbB.aMomentum -= cross(relativePosB, impulseMagnitude * normal);
    }
}




void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
    rigidBodies[i].torque += cross((loc - rigidBodies[i].position), force);
    rigidBodies[i].force += force;
}


void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    RigidBody rb;
    rb.position = position;
    rb.size = size;
    rb.mass = mass;

    float i1j1 = 1.0f / (static_cast<float>(mass) * (1.0f / 12.0f) * (size.y * size.y + size.z * size.z));
    float i2j2 = 1.0f / (static_cast<float>(mass) * (1.0f / 12.0f) * (size.x * size.x + size.z * size.z));
    float i3j3 = 1.0f / (static_cast<float>(mass) * (1.0f / 12.0f) * (size.x * size.x + size.y * size.y));

    rb.inertiaTensorInv = Mat4(i1j1, 0.0f, 0.0f, 0.0f, 0, i2j2, 0.0f, 0.0f, 0.0f, 0.0f, i3j3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    rb.objToWorldMat = computeWorldMat(rb);
    rigidBodies.push_back(rb);
}

void RigidBodySystemSimulator::addFixedRigidBody(Vec3 position, Vec3 size, int mass)
{
    RigidBody rb;
    rb.isFixed = true;
    rb.position = position;
    rb.size = size;
    rb.mass = mass;
    rb.inertiaTensorInv = Mat4(0);

    rb.objToWorldMat = computeWorldMat(rb);
    fixedRigidBodies.push_back(rb);
}

Mat4 RigidBodySystemSimulator::computeWorldMat(RigidBody& rb) {
    const Mat4 scale = Mat4(rb.size.X, 0, 0, 0, 0, rb.size.Y, 0, 0, 0, 0, rb.size.Z, 0, 0, 0, 0, 1);
    const Mat4 rotate = rb.orientation.getRotMat();
    const Mat4 translate = Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, rb.position.x, rb.position.y, rb.position.z, 1);
    return scale * rotate * translate;
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


int RigidBodySystemSimulator::getNumberOfRigidBodies() { return rigidBodies.size(); }
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { return rigidBodies.at(i).position; }
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) { return rigidBodies.at(i).lVelocity; }
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) { return rigidBodies.at(i).aVelocity; }
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) { rigidBodies.at(i).orientation = orientation.unit(); }
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) { rigidBodies.at(i).lVelocity = velocity; }
float RigidBodySystemSimulator::degToRad(float degree) { return degree * (XM_PI / 180); }

void RigidBodySystemSimulator::spawnWalls() {
    addFixedRigidBody(Vec3(-2.2f, 0.5f, 0.0f), Vec3(0.1f, 3.0f, 3.0f), 1); // Left Wall
    addFixedRigidBody(Vec3(2.2f, 0.5f, 0.0f), Vec3(0.1f, 3.0f, 3.0f), 1); // Right Wall

    addFixedRigidBody(Vec3(0.0f, 2.0f, 0.0f), Vec3(4.0f, 0.1f, 3.0f), 1); // Top Wall
    addFixedRigidBody(Vec3(0.0f, -1.0f, 0.0f), Vec3(4.0f, 0.1f, 3.0f), 1); // Bottom Wall
}



