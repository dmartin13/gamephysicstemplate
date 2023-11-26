#include "RigidBodySystemSimulator.h"


RigidBodySystemSimulator::RigidBodySystemSimulator() {
    m_iTestCase = 0; 
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
            DUC->drawRigidBody(rigidBodies.at(i).tranformMatrix);
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
        break;
    case 1:
        cout << "Demo 2\n";
        break;
    case 2:
        cout << "Demo 3\n";
        break;
    case 3:
        cout << "Demo 4\n";
        addRigidBody(Vec3(-1.5, 0, 0), Vec3(0.25), 1);
        addRigidBody(Vec3(0, 0, 0), Vec3(0.25), 1);
        addRigidBody(Vec3(1.5, 0, 0), Vec3(0.25), 1);
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}



void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    
}



void RigidBodySystemSimulator::simulateTimestep(float timeStep) {}



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



int RigidBodySystemSimulator::getNumberOfRigidBodies() { return 0; }



Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) { return Vec3(); }



Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {

    return Vec3();
}



Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {

    return Vec3();
}



void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {}



void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) 
{
    RigidBody rb;
    rb.mass = mass;
    rb.tranformMatrix.initId();

    Mat4 translationMat = Mat4(), rotationMat = Mat4(), scalingMat = Mat4();
    translationMat.initId();
    rotationMat.initId();
    scalingMat.initId();

    translationMat.initTranslation(position.X, position.Y, position.Z);
    //rotationMat.initRotationY(45);
    scalingMat.initScaling(size.X, size.Y, size.Z); 
    rb.tranformMatrix *= translationMat * scalingMat;

    rigidBodies.push_back(rb);
}



void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {}



void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {}