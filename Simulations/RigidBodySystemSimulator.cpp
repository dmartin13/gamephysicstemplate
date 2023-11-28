#include "RigidBodySystemSimulator.h"

Mat4 RigidBodySystemSimulator::calcWorldMatrix(RigidBody& rb) {
    const Mat4 scaleMat = Mat4(rb.width, 0, 0, 0, 0, rb.height, 0, 0, 0, 0, rb.depth, 0, 0, 0, 0, 1);
    const Mat4 rotMat = rb.orientation.getRotMat();
    const Mat4 translatMat = Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, rb.position.x, rb.position.y, rb.position.z, 1);
    return scaleMat * rotMat * translatMat;
}

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
        break;
    case 2:
        break;
    case 3:
        TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT,
            &gravity, "min=0.0 step=0.1");
        TwAddVarRW(DUC->g_pTweakBar, "C Factor", TW_TYPE_FLOAT,
            &c, "min=0.0 max=1.0 step=0.1");
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
    constRigidBodies.clear();
}

void RigidBodySystemSimulator::drawFrame(
    ID3D11DeviceContext* pd3dImmediateContext) {
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,
        0.6 * Vec3(1.f, 1.f, 1.f));
    for (const auto& rb : rigidBodies) {
        DUC->drawRigidBody(rb.worldMatrix);
    }
    // for (const auto& rb : constRigidBodies) {
    //     DUC->drawRigidBody(rb.worldMatrix);
    // }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        cout << "Demo1!\n";
        reset();
        setGravity(0);

        addRigidBody(Vec3(0., 0., 0.), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(0, Quat(0., 0., 90.));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1., 1., 0.));
        simulateTimestep(2.);
        std::cout << "linearVelocity: " << getLinearVelocityOfRigidBody(0) << std::endl;
        std::cout << "angularVelocity: " << getAngularVelocityOfRigidBody(0) << std::endl;

        break;
    case 1:
        cout << "Demo2!\n";
        reset();
        setGravity(0);

        timestepOverwrite = true;
        ownTimestep = 0.01;

        addRigidBody(Vec3(0., 0., 0.), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(0, Quat(0., 0., 90.));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1., 1., 0.));

        break;
    case 2:
        cout << "Demo3!\n";
        reset();
        setGravity(0);

        addRigidBody(Vec3(0., 0., 0.), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(0, Quat(0., 0., 90.));

        addRigidBody(Vec3(-1.5, 1., 1), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(1, Quat(45., 25., 45.));
        setVelocityOf(1, Vec3(0.2, -0.1, -0.2));

        break;
    case 3:
        cout << "Demo4!\n";
        reset();
        if (!gravitySet) {
            setGravity(0.5);
            gravitySet = true;
        }

        addRigidBodyInternal(Vec3(0, -2, 0), Vec3(10, 2, 10), 1, constRigidBodies);
        constRigidBodies[0].fixed = true;
        // addRigidBodyInternal(Vec3(0, 10, 0), Vec3(10, 2, 10), 1, constRigidBodies);
        // constRigidBodies[1].fixed = true;
        // addRigidBodyInternal(Vec3(-6, 4, 0), Vec3(2, 10, 10), 1, constRigidBodies);
        // constRigidBodies[2].fixed = true;
        // addRigidBodyInternal(Vec3(6, 4, 0), Vec3(2, 10, 10), 1, constRigidBodies);
        // constRigidBodies[3].fixed = true;
        // addRigidBodyInternal(Vec3(0, 4, 6), Vec3(10, 10, 2), 1, constRigidBodies);
        // constRigidBodies[4].fixed = true;
        // addRigidBodyInternal(Vec3(0, 4, -6), Vec3(10, 10, 2), 1, constRigidBodies);
        // constRigidBodies[5].fixed = true;

        addRigidBody(Vec3(0., 0., 0.), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(0, Quat(0., 0., 90.));

        addRigidBody(Vec3(-0.5, 1., 1), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(1, Quat(1., 10., 70.));
        setVelocityOf(1, Vec3(0.2, -0.1, -0.2));

        addRigidBody(Vec3(0.5, 1.3, 0.5), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(2, Quat(0., 0., 0.));

        addRigidBody(Vec3(0.3, 2., 0), Vec3(1., 0.6, 0.5), 2);
        setOrientationOf(3, Quat(23., 3., 67.));
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    if (m_iTestCase == 3) {
        //std::cout << "apply mouse" << std::endl;
        Point2D mouseDiff;
        mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
        mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
        if (mouseDiff.x != 0 || mouseDiff.y != 0)
        {
            Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() *
                DUC->g_camera.GetViewMatrix()); 	worldViewInv = worldViewInv.inverse();
            Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
            Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
            // find a proper scale!
            float inputScale = 0.1f;
            inputWorld = inputWorld * inputScale;
            for (int i = 0; i < rigidBodies.size(); ++i) {
                applyForceOnBody(i, Vec3(0, 0, 0), inputWorld);
            }
        }
    }

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
    if (timestepOverwrite) {
        timeStep = ownTimestep;
    }

    for (auto& rb : rigidBodies) {
        // position and velocity update (Euler Step)
        rb.position += timeStep * rb.linearVelocity;
        rb.linearVelocity += timeStep * ((rb.force / rb.mass) + Vec3(0.0f, -gravity, 0.0f));

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

        rb.worldMatrix = calcWorldMatrix(rb);
    }

    // check collisions. Check each rigibody against each other
    if (rigidBodies.size() >= 2) {
        for (int i = 0; i < rigidBodies.size(); ++i) {
            for (int j = i + 1; j < rigidBodies.size(); ++j) {
                CollisionInfo colInfo = checkCollisionSAT(rigidBodies[i].worldMatrix, rigidBodies[j].worldMatrix);
                if (colInfo.isValid) {
                    calculateImpulse(colInfo, rigidBodies[i], rigidBodies[j]);
                }
            }
        }
    }

    // check floor collisions. Check each rigibody against const rigid bodies
    if (constRigidBodies.size() > 0) {
        for (int i = 0; i < rigidBodies.size(); ++i) {
            for (int j = 0; j < constRigidBodies.size(); ++j) {
                CollisionInfo colInfo = checkCollisionSAT(rigidBodies[i].worldMatrix, constRigidBodies[j].worldMatrix);
                if (colInfo.isValid) {
                    calculateImpulse(colInfo, rigidBodies[i], constRigidBodies[j]);
                }
            }
        }
    }
}

void RigidBodySystemSimulator::calculateImpulse(CollisionInfo& colInfo, RigidBody& rbA, RigidBody& rbB) {
    // obatin normal
    const auto n = colInfo.normalWorld;
    // obtain colPointWorld
    const auto colPoint = colInfo.collisionPointWorld;

    // calculate relative positions
    const auto xA = colPoint - rbA.position;
    const auto xB = colPoint - rbB.position;

    // calculate relative velocities
    const auto vA = rbA.linearVelocity + cross(rbA.angularVelocity, xA);
    const auto vB = rbB.linearVelocity + cross(rbB.angularVelocity, xB);
    const auto relV = vA - vB;

    if (dot(relV, n) <= 0) {
        // calculate impulse strength (numerator)
        const auto numerator = -(1.0 + c) * dot(relV, n);

        // calculate denominator
        // calculate inverse masses (if fixed, set to 0)
        const auto invMassA = rbA.fixed ? 0 : (1.0 / rbA.mass);
        const auto invMassB = rbB.fixed ? 0 : (1.0 / rbB.mass);
        // calculate cross products
        const auto crossA = rbA.fixed ? Vec3(0, 0, 0) : cross(rbA.InvIntertiaTensor * cross(xA, n), xA);
        const auto crossB = rbB.fixed ? Vec3(0, 0, 0) : cross(rbB.InvIntertiaTensor * cross(xB, n), xB);
        const auto denominator = invMassA + invMassB + dot(crossA + crossB, n);

        // calculate J
        const auto J = numerator / denominator;

        // update velocities
        if (!rbA.fixed) {
            rbA.linearVelocity += (J * n) / rbA.mass;
        }

        if (!rbB.fixed) {
            rbB.linearVelocity -= (J * n) / rbB.mass;
        }

        // update angular momentum
        if (!rbA.fixed) {
            rbA.angularMomentum += cross(xA, J * n);
        }
        if (!rbB.fixed) {
            rbB.angularMomentum -= cross(xB, J * n);
        }
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
    addRigidBodyInternal(position, size, mass, rigidBodies);
}

void RigidBodySystemSimulator::addRigidBodyInternal(Vec3 position, Vec3 size,
    int mass, std::vector<RigidBody>& storage) {
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

    rb.worldMatrix = calcWorldMatrix(rb);
    rb.fixed = false;

    storage.push_back(rb);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
    // make sure we use unit length orientation
    rigidBodies[i].orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
    rigidBodies[i].linearVelocity = velocity;
}

void RigidBodySystemSimulator::setGravity(float g) {
    gravity = g;
}