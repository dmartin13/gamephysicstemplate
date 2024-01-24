#include "RigidBodySystemSimulator.h"

void RigidBodySystemSimulator::createSpringMesh(size_t m, size_t n,
    double spacing, Vec3 pos,
    double scale) {
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            bool fixed =
                (i == 0 || j == 0 || i == (m - 1) || j == (n - 1)) ? true : false;
            size_t rb = addRigidBody(pos + Vec3(i * spacing, 0, j * spacing),
                Vec3(scale, scale, scale), 0.1, fixed, true);
            if (j > 0) {
                addSpring(rb - 1, rb, spacing);
            }
            if (i > 0) {
                addSpring(rb - n, rb, spacing);
            }
        }
    }
}

Mat4 RigidBodySystemSimulator::calcWorldMatrix(RigidBody& rb) {
    const Mat4 scaleMat = Mat4(rb.width, 0, 0, 0, 0, rb.height, 0, 0, 0, 0,
        rb.depth, 0, 0, 0, 0, 1);
    const Mat4 rotMat = rb.orientation.getRotMat();
    const Mat4 translatMat = Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
        rb.position.x, rb.position.y, rb.position.z, 1);
    // Mat4 translatMatTmp;
    // translatMatTmp.initTranslation(rb.position.x, rb.position.y,
    // rb.position.z); std::cout << "translatMat" << translatMat << std::endl;
    // std::cout << "translatMatTmp" << translatMatTmp << std::endl;
    return scaleMat * rotMat * translatMat;
}

RigidBodySystemSimulator::RigidBodySystemSimulator() {
    m_iTestCase = 0;

    _mass = 1.f;
    _stiffness = 400.f;
    _damping = 0.f;
    _externalForce = Vec3(0.f, 0.f, 0.f);
    _gravity = -1.;
    _damping = 0.5;

    _mouse.x = _mouse.y = 0;
    _trackmouse.x = _trackmouse.y = 0;
    _oldtrackmouse.x = _oldtrackmouse.y = 0;

    // test for mat4 vec3 multiplication
    Mat4 m4(1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 1);
    Vec3 v3(1, 2, 3);
    // expected result: (51,58,65)
    const auto result = m4 * v3;
    std::cout << "mat4 * vec3 test: " << result << std::endl;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
    return "Demo1,Demo2,Demo3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    switch (m_iTestCase) {
    case 0: {
        reset();
        size_t rb0 = addRigidBody(Vec3(-1, 0, 0), Vec3(0.5, 0.25, 0.25), 1);
        size_t rb1 = addRigidBody(Vec3(1, 0, 0), Vec3(0.5, 0.25, 0.25), 1);
        addSpring(rb0, rb1, 1.0);
    } break;
    case 1: {
        reset();
        createSpringMesh(25, 25, 0.02, Vec3(0, 0, 0), SPHERERADIUS);

        size_t anotherRB =
            addRigidBody(Vec3(0.25, 0.2, 0.25), Vec3(0.1, 0.1, 0.1), 1);
        setVelocityOf(anotherRB, Vec3(0, -0.5, 0));
    } break;
    case 2: {
        // TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &_gravity,
        //     "min=0.0 step=0.1");
        // TwAddVarRW(DUC->g_pTweakBar, "C Factor", TW_TYPE_FLOAT, &_c,
        //     "min=0.0 max=1.0 step=0.1");

        reset();
        // createGrid(2, 3);
        size_t rb0 = addRigidBody(Vec3(-1, 0.2, 0.5),
            Vec3(SPHERERADIUS, SPHERERADIUS, SPHERERADIUS), 1,
            false, true);
        size_t rb1 = addRigidBody(Vec3(-2, 0.2, 0),
            Vec3(SPHERERADIUS, SPHERERADIUS, SPHERERADIUS), 1,
            false, true);
        size_t rb2 = addRigidBody(Vec3(0.1, 1, -0.5),
            Vec3(SPHERERADIUS, SPHERERADIUS, SPHERERADIUS), 1,
            false, true);
        // size_t rb1 = addRigidBody(Vec3(1, 0, 0),
        //                           Vec3(SPHERERADIUS, SPHERERADIUS, SPHERERADIUS),
        //                           1, false, true);
        size_t rb3 = addRigidBody(Vec3(1, 0, 0), Vec3(1, 0.5, 0.5), 1, false, false);
        addSpring(rb0, rb3, 1.0);
        addSpring(rb1, rb3, 1.0);
        addSpring(rb2, rb3, 1.0);
    } break;
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
        if (rb.isSphere)
            DUC->drawSphere(rb.position, Vec3(SPHERERADIUS));
        else
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
        if (rb.fixed) {
            continue;
        }
        // position and velocity update (Leapfrog)
        rb.linearVelocity +=
            timeStep * ((rb.force / rb.mass) + Vec3(0.0f, _gravity, 0.0f));
        rb.position += timeStep * rb.linearVelocity;


        // rotation update
        rb.orientation += (Quat(rb.angularVelocity.x, rb.angularVelocity.y,
            rb.angularVelocity.z, 0) *
            rb.orientation) *
            (0.5 * timeStep);
        // bring back to unit length as mentioned in exercise sheet
        rb.orientation = rb.orientation.unit();

        // angular momentum
        rb.angularMomentum += timeStep * rb.torque;

        if (!rb.isSphere) {
            // angular velcocity
            //  first calculate I^-1 = Rot_r * I_0^-1 * Ro_r^T
            Mat4 rot = rb.orientation.getRotMat();
            Mat4 rotT = rot;
            rotT.transpose();
            Mat4 InewInv = rot * rb.InvIntertiaTensor * rotT;

            rb.angularVelocity = InewInv * rb.angularMomentum;
        }

        // reset forces and torque
        rb.force = 0;
        rb.torque = 0;

        rb.worldMatrix = calcWorldMatrix(rb);
    }

    // check collisions. Check each rigid body against each other
    if (_rigidBodies.size() >= 2) {
        for (int i = 0; i < _rigidBodies.size(); ++i) {
            for (int j = i + 1; j < _rigidBodies.size(); ++j) {
                if (!_rigidBodies[i].isSphere && !_rigidBodies[j].isSphere) {
                    CollisionInfo colInfo = checkCollisionSAT(
                        _rigidBodies[i].worldMatrix, _rigidBodies[j].worldMatrix);
                    if (colInfo.isValid) {
                        calculateImpulse(colInfo, _rigidBodies[i], _rigidBodies[j]);
                    }
                }
                else if ((_rigidBodies[i].isSphere && !_rigidBodies[j].isSphere) ||
                    (!_rigidBodies[i].isSphere && _rigidBodies[j].isSphere)) {
                    CollisionInfo colInfo =
                        checkCollisionSphereBox(_rigidBodies[i], _rigidBodies[j]);
                    if (colInfo.isValid) {
                        calculateImpulse(colInfo, _rigidBodies[i], _rigidBodies[j]);
                    }
                }
                else {
                    // Both are spheres
                    CollisionInfo colInfo =
                        checkCollisionSphere(_rigidBodies[i], _rigidBodies[j]);
                    if (colInfo.isValid) {
                        calculateImpulse(colInfo, _rigidBodies[i], _rigidBodies[j]);
                    }
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
    applyDamping(_rigidBodies);
}

CollisionInfo RigidBodySystemSimulator::checkCollisionSphere(RigidBody& rbA,
    RigidBody& rbB) {
    CollisionInfo info;
    info.isValid = false;

    if (!rbA.isSphere || !rbB.isSphere) {
        return info;
    }

    const auto direction = rbA.position - rbB.position;
    const auto dist = norm(rbA.position - rbB.position);
    if (dist < SPHERERADIUS * 2.) {
        // collision happens
        info.isValid = true;

        Vec3 collisionNormal = getNormalized(direction);
        const float collisionDepth = (SPHERERADIUS * 2. - dist) / 2.;
        const Vec3 collisionPointWorld =
            rbB.position + collisionNormal * (SPHERERADIUS - collisionDepth);

        info.normalWorld = collisionNormal;
        info.depth = collisionDepth;
        info.collisionPointWorld = collisionPointWorld;
    }

    return info;
}

CollisionInfo
RigidBodySystemSimulator::checkCollisionSphereBox(RigidBody& rbA,
    RigidBody& rbB) {

    // std::cout << "checkCollisionSphereBox" << std::endl;

    CollisionInfo info;
    info.isValid = false;

    if ((rbA.isSphere && rbB.isSphere) || (!rbA.isSphere && !rbB.isSphere)) {
        return info;
    }

    RigidBody& box = rbA.isSphere ? rbB : rbA;
    RigidBody& sphere = rbA.isSphere ? rbA : rbB;

    // we project the spheres center into the local space of the box
    const Mat4 rotMat = box.orientation.getRotMat();
    const Mat4 translatMat =
        Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, box.position.x, box.position.y,
            box.position.z, 1);
    auto rotTrans = rotMat * translatMat;
    auto sphereCenterInLocalBoxSpace =
        rotTrans.inverse() * sphere.position;
    // sphereCenterInLocalBoxSpace.x *= box.width;
    // sphereCenterInLocalBoxSpace.y *= box.height;
    // sphereCenterInLocalBoxSpace.z *= box.depth;

    // std::cout << "box.worldMatrix.inverse() " << box.worldMatrix.inverse()
    //           << std::endl;
    // std::cout << "sphereCenterInLocalBoxSpace " << sphereCenterInLocalBoxSpace
    //           << std::endl;

    // now we can work on the box's local coords. We calculate the closest point
    // from the sphere to the box's surface
    const Vec3 boxMin(-box.width / 2., -box.height / 2., -box.depth / 2.);
    const Vec3 boxMax(box.width / 2., box.height / 2., box.depth / 2.);
    const Vec3 closestPoint(
        std::max(boxMin.x, std::min(sphereCenterInLocalBoxSpace.x, boxMax.x)),
        std::max(boxMin.y, std::min(sphereCenterInLocalBoxSpace.y, boxMax.y)),
        std::max(boxMin.z, std::min(sphereCenterInLocalBoxSpace.z, boxMax.z)));
    // transform the closestPoint back into worldspace

    auto closestPointWorldSpace = rotTrans * closestPoint;
    // closestPointWorldSpace.x *= box.width;
    // closestPointWorldSpace.y *= box.height;
    // closestPointWorldSpace.z *= box.depth;

    // std::cout << "closestPointLocal " << closestPoint << std::endl;
    // std::cout << "closestPointWorldSpace " << closestPointWorldSpace <<
    // std::endl; std::cout << "box.worldMatrix " << box.worldMatrix << std::endl;
    // std::cout << "sphere.position " << sphere.position << std::endl;
    // std::cout << "box.position " << box.position << std::endl;

    // now we check the distance from the closestPointWorldSpace to sphere center
    // in world space
    const auto direction = closestPointWorldSpace - sphere.position;
    const auto distFromSphereToBox =
        norm(closestPointWorldSpace - sphere.position);

    if (distFromSphereToBox < SPHERERADIUS) {
        // std::cout << "the shit is colliding" << std::endl;

        info.isValid = true;

        Vec3 collisionNormal = getNormalized(direction);

        const float collisionDepth = SPHERERADIUS - distFromSphereToBox;
        const Vec3 collisionPointWorld =
            sphere.position + collisionNormal * (SPHERERADIUS - collisionDepth);

        if (rbA.isSphere) {
            collisionNormal = -collisionNormal;
        }

        info.normalWorld = collisionNormal;
        info.depth = collisionDepth;
        info.collisionPointWorld = collisionPointWorld;
    }

    return info;
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
    float mass, bool fixed,
    bool isSphere) {
    return addRigidBodyInternal(position, size, mass, _rigidBodies, fixed,
        isSphere);
}

size_t RigidBodySystemSimulator::addRigidBodyInternal(
    Vec3 position, Vec3 size, float mass, std::vector<RigidBody>& storage,
    bool fixed, bool isSphere) {

    RigidBody rb;

    // init values from arguments
    rb.position = position;
    rb.mass = mass;
    rb.width = size.x;
    rb.height = size.y;
    rb.depth = size.z;

    // If it is a sphere
    rb.isSphere = isSphere;

    // default inits
    rb.linearVelocity = Vec3();
    rb.angularVelocity = Vec3();
    rb.angularMomentum = Vec3();
    rb.force = Vec3();
    rb.torque = Vec3();
    rb.orientation = Quat(0., 0., 0., 1.);

    float iit0, iit1, iit2;

    // inverse inertia tensor for sphere
    //  https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    if (isSphere) {
        iit0 = iit1 = iit2 =
            (2.f * float(mass) * SPHERERADIUS * SPHERERADIUS) / 3.f;
    }
    else {
        // calculate InVIntertaTensor (use version for rectengular boxes)
        // Note: Y and Z axis are flipped in contrast to the version from
        // wikipedia
        iit0 = 1.f /
            (float(mass) * (1.f / 12.f) * (size.y * size.y + size.z * size.z));
        iit1 = 1.f /
            (float(mass) * (1.f / 12.f) * (size.x * size.x + size.z * size.z));
        iit2 = 1.f /
            (float(mass) * (1.f / 12.f) * (size.x * size.x + size.y * size.y));
    }

    rb.InvIntertiaTensor =
        Mat4(iit0, 0., 0., 0., 0, iit1, 0., 0., 0., 0., iit2, 0., 0., 0., 0., 0.);

    rb.worldMatrix = calcWorldMatrix(rb);
    rb.fixed = fixed;

    storage.push_back(rb);

    return storage.size() - 1;
}

void RigidBodySystemSimulator::createGrid(size_t rows, size_t cols) {
    std::mt19937 eng;
    std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
    std::uniform_real_distribution<float> randVel(-0.5f, 0.5f);

    // To create a box, this line is for testing only
    // addRigidBody(Vec3(), Vec3(0.5, 0.25, 0.25), 1, false, false);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Last parameter if true, the grid is a grid of spheres, false = grid
            // of boxes
            size_t rb = addRigidBody(Vec3(randPos(eng)), Vec3(0.5, 0.25, 0.25), 1,
                false, true);

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
    std::vector<RigidBody>& rigidBodies) {
    for (auto& rb : rigidBodies) {
        rb.force -= _damping * rb.linearVelocity;
    }
}