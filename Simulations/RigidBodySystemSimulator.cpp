#include "RigidBodySystemSimulator.h"

Mat4 RigidBodySystemSimulator::calcWorldMatrix(RigidBody& rb) {
    const Mat4 scaleMat = Mat4(rb.width, 0, 0, 0, 0, rb.height, 0, 0, 0, 0,
        rb.depth, 0, 0, 0, 0, 1);
    const Mat4 rotMat = rb.orientation.getRotMat();
    const Mat4 translatMat = Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
        rb.position.x, rb.position.y, rb.position.z, 1);
    return scaleMat * rotMat * translatMat;
}

RigidBodySystemSimulator::RigidBodySystemSimulator() {
    m_iTestCase = 0;

    _mass = 1.f;
    _stiffness = 1.f;
    _damping = 0.f;
    _externalForce = Vec3(0.f, 0.f, 0.f);

    _mouse.x = _mouse.y = 0;
    _trackmouse.x = _trackmouse.y = 0;
    _oldtrackmouse.x = _oldtrackmouse.y = 0;
}
const char* RigidBodySystemSimulator::getTestCasesStr() {
    return "Demo1,Demo2,Demo3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    switch (m_iTestCase) {
    case 0: {
        //size_t rb0 = addRigidBody(Vec3(-1, 0, 0), Vec3(0.5, 0.25, 0.25), 1);
        //size_t rb1 = addRigidBody(Vec3(1, 0, 0), Vec3(0.5, 0.25, 0.25), 1);
        //addSpring(rb0, rb1, 1.0);

        reset();
        createGrid(2, 3);

    } break;
    case 1:
        break;
    case 2:
        TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &_gravity,
            "min=0.0 step=0.1");
        TwAddVarRW(DUC->g_pTweakBar, "C Factor", TW_TYPE_FLOAT, &_c,
            "min=0.0 max=1.0 step=0.1");
        break;
    default:
        break;
    }
}

void RigidBodySystemSimulator::reset() {
    _mouse.x = _mouse.y = 0;
    _trackmouse.x = _trackmouse.y = 0;
    _oldtrackmouse.x = _oldtrackmouse.y = 0;

    _rigidBodies.clear();
    _constRigidBodies.clear();
    _springs.clear();
}

void RigidBodySystemSimulator::drawFrame(
    ID3D11DeviceContext* pd3dImmediateContext) {
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,
        0.6 * Vec3(1.f, 1.f, 1.f));
    for (const auto& rb : _rigidBodies) {
        DUC->drawRigidBody(rb.worldMatrix);
    }
    for (const auto& s : _springs) {
        DUC->beginLine();
        DUC->drawLine(_rigidBodies[s._masspoint1].position, Vec3(1.f, 0.f, 0.f),
            _rigidBodies[s._masspoint2].position, Vec3(1.f, 0.f, 0.f));
        DUC->endLine();
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

        break;
    case 1:
        cout << "Demo2!\n";

        break;
    case 2:
        cout << "Demo3!\n";

        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    if (m_iTestCase == 3) {
        // std::cout << "apply mouse" << std::endl;
        Point2D mouseDiff;
        mouseDiff.x = _trackmouse.x - _oldtrackmouse.x;
        mouseDiff.y = _trackmouse.y - _oldtrackmouse.y;
        if (mouseDiff.x != 0 || mouseDiff.y != 0) {
            Mat4 worldViewInv =
                Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
            worldViewInv = worldViewInv.inverse();
            Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
            Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
            // find a proper scale!
            float inputScale = 0.1f;
            inputWorld = inputWorld * inputScale;
            for (int i = 0; i < _rigidBodies.size(); ++i) {
                // applyForceOnBody(i, Vec3(0, 0, 0), inputWorld);
            }
        }
    }
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {
    for (auto& rb : _rigidBodies) {
        // position and velocity update (Euler Step)
        rb.position += timeStep * rb.linearVelocity;
        rb.linearVelocity +=
            timeStep * ((rb.force / rb.mass) + Vec3(0.0f, -_gravity, 0.0f));

        // rotation update
        rb.orientation += (Quat(rb.angularVelocity.x, rb.angularVelocity.y,
            rb.angularVelocity.z, 0) *
            rb.orientation) *
            (0.5 * timeStep);
        // bring back to unit length as mentioned in exercise sheet
        rb.orientation = rb.orientation.unit();

        // angular momentum
        rb.angularMomentum += timeStep * rb.torque;

        // angular velcocity
        //  first calculate I^-1 = Rot_r * I_0^-1 * Ro_r^T
        Mat4 rot = rb.orientation.getRotMat();
        Mat4 rotT = rot;
        rotT.transpose();
        Mat4 InewInv = rot * rb.InvIntertiaTensor * rotT;

        rb.angularVelocity = InewInv * rb.angularMomentum;

        // reset forces and torque
        rb.force = 0;
        rb.torque = 0;

        rb.worldMatrix = calcWorldMatrix(rb);
    }

    // check collisions. Check each rigibody against each other
    if (_rigidBodies.size() >= 2) {
        for (int i = 0; i < _rigidBodies.size(); ++i) {
            for (int j = i + 1; j < _rigidBodies.size(); ++j) {
                CollisionInfo colInfo = checkCollisionSAT(_rigidBodies[i].worldMatrix,
                    _rigidBodies[j].worldMatrix);
                if (colInfo.isValid) {
                    calculateImpulse(colInfo, _rigidBodies[i], _rigidBodies[j]);
                }
            }
        }
    }

    // check floor collisions. Check each rigibody against const rigid bodies
    if (_constRigidBodies.size() > 0) {
        for (int i = 0; i < _rigidBodies.size(); ++i) {
            for (int j = 0; j < _constRigidBodies.size(); ++j) {
                CollisionInfo colInfo = checkCollisionSAT(
                    _rigidBodies[i].worldMatrix, _constRigidBodies[j].worldMatrix);
                if (colInfo.isValid) {
                    calculateImpulse(colInfo, _rigidBodies[i], _constRigidBodies[j]);
                }
            }
        }
    }

    // force computation of springs
    computeForces(_rigidBodies);
}

void RigidBodySystemSimulator::calculateImpulse(CollisionInfo& colInfo,
    RigidBody& rbA,
    RigidBody& rbB) {
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
        const auto numerator = -(1.0 + _c) * dot(relV, n);

        // calculate denominator
        // calculate inverse masses (if fixed, set to 0)
        const auto invMassA = rbA.fixed ? 0 : (1.0 / rbA.mass);
        const auto invMassB = rbB.fixed ? 0 : (1.0 / rbB.mass);
        // calculate cross products
        const auto crossA = rbA.fixed
            ? Vec3(0, 0, 0)
            : cross(rbA.InvIntertiaTensor * cross(xA, n), xA);
        const auto crossB = rbB.fixed
            ? Vec3(0, 0, 0)
            : cross(rbB.InvIntertiaTensor * cross(xB, n), xB);
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
    _trackmouse.x = x;
    _trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
    _oldtrackmouse.x = x;
    _oldtrackmouse.y = y;
    _trackmouse.x = x;
    _trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
    return _rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
    return _rigidBodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
    return _rigidBodies[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
    return _rigidBodies[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
    // calculate and accumulate the torque
    _rigidBodies[i].torque += cross((loc - _rigidBodies[i].position), force);
    // accumulate the force
    _rigidBodies[i].force += force;
}

size_t RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size,
    int mass) {
    return addRigidBodyInternal(position, size, mass, _rigidBodies);
}

size_t RigidBodySystemSimulator::addRigidBodyInternal(
    Vec3 position, Vec3 size, int mass, std::vector<RigidBody>& storage) {
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

    return storage.size() - 1;
}

void RigidBodySystemSimulator::createGrid(size_t rows, size_t cols) {
    std::mt19937 eng;
    std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    std::uniform_real_distribution<float> randVel(-0.5f, 0.5f);

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            size_t rb = addRigidBody(randPos(eng), Vec3(0.5, 0.25, 0.25), 1);

            if (j < cols - 1)
                addSpring(i * cols + j, i * cols + j + 1, 1.0);

            if (i < rows - 1)
                addSpring(i * cols + j, (i + 1) * cols + j, 1.0);
        }
    }
}


void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
    // make sure we use unit length orientation
    _rigidBodies[i].orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
    _rigidBodies[i].linearVelocity = velocity;
}

void RigidBodySystemSimulator::setGravity(float g) { _gravity = g; }

float RigidBodySystemSimulator::degToRad(float degree) {
    return degree * (XM_PI / 180);
}

void RigidBodySystemSimulator::setMass(float mass) { _mass = mass; }

void RigidBodySystemSimulator::setStiffness(float stiffness) {
    _stiffness = stiffness;
}

void RigidBodySystemSimulator::setDampingFactor(float damping) {
    _damping = damping;
}

void RigidBodySystemSimulator::setBounceFactor(float bounceFactor) {
    _bounceFactor = bounceFactor;
}

void RigidBodySystemSimulator::addSpring(int masspoint1, int masspoint2,
    float initialLength) {
    _springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

void RigidBodySystemSimulator::computeForces(
    std::vector<RigidBody>& rigidBodies) {
    for (auto& s : _springs) {
        auto rb1 = rigidBodies[s._masspoint1];
        auto rb2 = rigidBodies[s._masspoint2];
        Vec3 d = rb1.position - rb2.position;
        float dist = sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z);
        Vec3 f1 = -_stiffness * (dist - s._initialLength) * (d / dist);
        Vec3 f2 = -f1;

        applyForceOnBody(s._masspoint1, rb1.position, f1);
        applyForceOnBody(s._masspoint2, rb2.position, f2);
    }
}

void RigidBodySystemSimulator::applyDamping(
    std::vector<MassPoint>& massPoints) {
    for (auto& p : massPoints) {
        p._force -= _damping * p._velocity;
    }
}