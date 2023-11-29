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
        //break;
    case 4:
        for (int i{ 0 }; i < rigidBodies.size(); i++)
        {
            DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
            DUC->drawRigidBody(rigidBodies.at(i).objToWorldMat);
        }
        break;
    defualt:
        break;
    }
}


void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        cout << "Demo 1\n";

        addRigidBody(Vec3(0., 0., 0.), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(0, Quat(0., 0., 90.));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1., 1., 0.));
        simulateTimestep(2.);
        std::cout << "v = " << getLinearVelocityOfRigidBody(0) << std::endl;
        std::cout << "w = " << getAngularVelocityOfRigidBody(0) << std::endl;
        break;
    case 1:
        cout << "Demo 2\n";
        break;
    case 2:
        cout << "Demo 3\n";
        break;
    case 3:
        cout << "Demo 4\n";
        addRigidBody(Vec3(-0.5, 0, 0), Vec3(0.25), 1);
        addRigidBody(Vec3(0, 0, 0), Vec3(0.25), 1);
        addRigidBody(Vec3(0.5, 0, 0), Vec3(0.25), 1);
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}



void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    
}



void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
    switch (m_iTestCase)
    {
    case 0: // 1 Step
        // position and velocity update (Euler Step)
        for (auto& rb : rigidBodies) 
        {
            // Euler for linear velocity & position
            rb.position += timeStep * rb.lVelocity;
            rb.lVelocity += timeStep * (rb.force / rb.mass);

            // Rotation update
            rb.orientation += (Quat(rb.aVelocity.x, rb.aVelocity.y, rb.aVelocity.z, 0) * rb.orientation) * (0.5 * timeStep);
            rb.orientation = rb.orientation.unit();

            // Angular momentum
            rb.aMomentum += timeStep * rb.torque;

            
            Mat4 rot = rb.orientation.getRotMat();
            Mat4 rotT = rot;
            rotT.transpose();
            Mat4 newInertiaTensorInv = rot * rb.inertiaTensorInv * rotT;

            rb.aVelocity = newInertiaTensorInv * rb.aMomentum;

            //reset forces and torque
            rb.force = 0;
            rb.torque = 0;
        }
        break;
    case 1:
        //for (int i{ 0 }; i < rigidBodies.size(); i++)
        //{
        //    // Calculate the force
        //    Vec3 pointDiff = massPoint1.position - massPoint2.position;
        //    spring.currentLength = sqrtf(powf(pointDiff.X, 2) + powf(pointDiff.Y, 2) + powf(pointDiff.Z, 2));
        //    Vec3 forceOnMp1 = (-spring.stiffness) * (spring.currentLength - spring.initialLength) * (pointDiff.operator/(spring.currentLength));
        //    Vec3 forceOnMp2 = forceOnMp1.operator*(-1);

        //    // Calculate accelerations using Euler
        //    Vec3 accAtOldPosMp1 = forceOnMp1.operator/(massPoint1.mass);
        //    Vec3 accAtOldPosMp2 = forceOnMp2.operator/(massPoint2.mass);

        //    // Calculate positions using Euler
        //    integratePosition(massPoint1, timeStep);
        //    integratePosition(massPoint2, timeStep);

        //    // Calculate velocities using Euler
        //    integrateVelocity(massPoint1, accAtOldPosMp1, timeStep);
        //    integrateVelocity(massPoint2, accAtOldPosMp2, timeStep);
        //}
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        break;
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



int RigidBodySystemSimulator::getNumberOfRigidBodies() { return rigidBodies.size(); }
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { return rigidBodies.at(i).position; }
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) { return rigidBodies.at(i).lVelocity; }
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) { return rigidBodies.at(i).aVelocity; }


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
   
    float i1j1 = (1.0f / 12.0f) * (1.0f / (static_cast<float>(mass)) * (size.y * size.y + size.z * size.z));
    float i2j2 = (1.0f / 12.0f) * (1.0f / (static_cast<float>(mass)) * (size.x * size.x + size.z * size.z));
    float i3j3 = (1.0f / 12.0f) * (1.0f / (static_cast<float>(mass)) * (size.x * size.x + size.y * size.y));

    rb.inertiaTensorInv = Mat4(i1j1, 0.0f, 0.0f, 0.0f, 0, i2j2, 0.0f, 0.0f, 0.0f, 0.0f, i3j3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    rigidBodies.push_back(rb);
}



void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) { rigidBodies.at(i).orientation = orientation.unit(); }



void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) { rigidBodies.at(i).lVelocity = velocity; }