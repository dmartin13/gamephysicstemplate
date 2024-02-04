#include "RigidBodySystemSimulator.h"

void RigidBodySystemSimulator::createSpringMesh(size_t m, size_t n,
    double spacing, Vec3 pos,
    double scale, double mass) {
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            bool fixed =
                (i == 0 || j == 0 || i == (m - 1) || j == (n - 1)) ? true : false;
            size_t rb = addRigidBody(pos + Vec3(i * spacing, 0, j * spacing),
                Vec3(scale, scale, scale), mass, fixed, true);
            if (j > 0) {
                addSpring(rb - 1, rb, spacing,
                    std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
            }
            if (i > 0) {
                addSpring(rb - n, rb, spacing,
                    std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
            }
        }
    }
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
                addSpring(i * cols + j, i * cols + j + 1, 1.0,
                    std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});

            if (i < rows - 1)
                addSpring(i * cols + j, (i + 1) * cols + j, 1.0,
                    std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
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

    _mouse.x = _mouse.y = 0;
    _trackmouse.x = _trackmouse.y = 0;
    _oldtrackmouse.x = _oldtrackmouse.y = 0;

    // test for mat4 vec3 multiplication
    // Mat4 m4(1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 1);
    // Vec3 v3(1, 2, 3);
    // expected result: (51,58,65)
    // const auto result = m4 * v3;
    // std::cout << "mat4 * vec3 test: " << result << std::endl;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
    return "Spring RB Basic,Spring RB Offset,Sphere RBs,Net RB,Rel Offset "
        "Implulse,Force Apply";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;
    switch (m_iTestCase) {
    case 0: {

        reset();

        _mass = 1.f;
        _stiffness = 20.f;
        _damping = 0.f;
        _gravity = 0;
        _damping = 0;
        _c = 1.;

        size_t rb0 = addRigidBody(Vec3(-1, 0, 0), Vec3(0.5, 0.25, 0.25), _mass);
        size_t rb1 = addRigidBody(Vec3(1, 0, 0), Vec3(0.5, 0.25, 0.25), _mass);
        addSpring(rb0, rb1, 1.0,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
    } break;
    case 1: {

        reset();

        _mass = 1.f;
        _stiffness = 20.f;
        _damping = 0.f;
        _gravity = 0;
        _damping = 0;
        _c = 1.;

        size_t rb0 = addRigidBody(Vec3(-1, 0, 0), Vec3(0.5, 0.25, 0.25), _mass);
        size_t rb1 = addRigidBody(Vec3(1, 0, 0), Vec3(0.5, 0.25, 0.25), _mass);
        addSpring(rb0, rb1, 1.0,
            std::array<Vec3, 2>{
                {Vec3(0.25, 0.125, 0.125), Vec3(-0.25, -0.125, -0.125)}});
    } break;
    case 2: {
        reset();

        _mass = 1.f;
        _stiffness = 100.f;
        _gravity = 0;
        _damping = 0;
        _sphereRadius = 0.2;
        _c = 1.;

        size_t rb0 = addRigidBody(Vec3(-1, 0.2, 0.5),
            Vec3(_sphereRadius, _sphereRadius, _sphereRadius),
            1, false, true);
        size_t rb1 = addRigidBody(Vec3(-2, 0.2, 0),
            Vec3(_sphereRadius, _sphereRadius, _sphereRadius),
            1, false, true);
        size_t rb2 = addRigidBody(Vec3(0.1, 1, -0.5),
            Vec3(_sphereRadius, _sphereRadius, _sphereRadius),
            1, false, true);
        size_t rb3 =
            addRigidBody(Vec3(1, 0, 0), Vec3(1, 0.5, 0.5), _mass, false, false);
        addSpring(rb0, rb3, 1.0,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
        addSpring(rb1, rb3, 1.0,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
        addSpring(rb2, rb3, 1.0,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0, 0, 0)}});
    } break;
    case 3: {
        // TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &_gravity,
        //     "min=0.0 step=0.1");
        // TwAddVarRW(DUC->g_pTweakBar, "C Factor", TW_TYPE_FLOAT, &_c,
        //     "min=0.0 max=1.0 step=0.1");

        reset();

        _mass = 1.f;
        _stiffness = 400.f;
        _gravity = -9.81;
        _damping = 0.5;
        _sphereRadius = 0.1;
        _c = 1.;

        TwAddVarRW(DUC->g_pTweakBar, "Mass Grid Spheres", TW_TYPE_FLOAT,
            &_massGridSpheres, "min=0.0 max=100.0 step=0.1");

        createSpringMesh(25, 25, 0.04, Vec3(-0.5, 0, -0.5), 0.01, _massGridSpheres);

        size_t anotherRB =
            addRigidBody(Vec3(0, 0.2, 0), Vec3(0.2, 0.2, 0.2), _mass);
        setVelocityOf(anotherRB, Vec3(0, -0.5, 0));
    } break;
    case 4: {
        reset();

        _mass = 1.f;
        _stiffness = 200.f;
        _gravity = -9.81;
        _damping = 1;
        _sphereRadius = 0.01;
        _c = 1.;

        size_t fixed0 = addRigidBody(
            Vec3(-0.5, 0.5, -0.5),
            Vec3(_sphereRadius, _sphereRadius, _sphereRadius), 1, true, true);
        size_t fixed1 = addRigidBody(
            Vec3(-0.5, 0.5, 0.5), Vec3(_sphereRadius, _sphereRadius, _sphereRadius),
            1, true, true);
        size_t fixed2 = addRigidBody(
            Vec3(0.5, 0.5, -0.5), Vec3(_sphereRadius, _sphereRadius, _sphereRadius),
            1, true, true);
        size_t fixed3 = addRigidBody(
            Vec3(0.5, 0.5, 0.5), Vec3(_sphereRadius, _sphereRadius, _sphereRadius),
            1, true, true);

        size_t plane =
            addRigidBody(Vec3(0, 0.3, 0), Vec3(0.25, 0.05, 0.25), 1, false, false);

        addSpring(fixed0, plane, 0.1,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(-0.125, 0.0, -0.125)}});
        addSpring(fixed1, plane, 0.1,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(-0.125, 0.0, 0.125)}});
        addSpring(fixed2, plane, 0.1,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0.125, 0.0, -0.125)}});
        addSpring(fixed3, plane, 0.1,
            std::array<Vec3, 2>{{Vec3(0, 0, 0), Vec3(0.125, 0.0, 0.125)}});

        size_t ball = addRigidBody(
            Vec3(-0.1, 1, 0), Vec3(_sphereRadius, _sphereRadius, _sphereRadius), 1,
            false, true);

        setOrientationOf(plane, Quat(0, degToRad(90.), 0));

    } break;
    case 5: {

        reset();

        _mass = 1.f;
        _stiffness = 20.f;
        _damping = 0.f;
        _gravity = 0;
        _damping = 0;
        _c = 1.;

        size_t rb0 = addRigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), _mass);
        setOrientationOf(rb0, Quat(0, degToRad(45.), 0));

        size_t rb1 = addRigidBody(Vec3(0, 0.2, 1), Vec3(0.25, 0.25, 0.25), _mass);

        size_t rb2 =
            addRigidBody(Vec3(0, -0.1, 1), Vec3(0.1, 0.1, 0.1), _mass, false, true);
    } break;
    default:
        break;
    }
    TwAddVarRW(DUC->g_pTweakBar, "Force Factor", TW_TYPE_FLOAT, &_forceFactor,
        "min=0.0 max=100.0 step=1");
    TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &_damping,
        "min=0.0 max=100.0 step=0.1");
    TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &_stiffness,
        "min=0.0 max=400.0 step=1");
    TwAddVarRW(DUC->g_pTweakBar, "C", TW_TYPE_FLOAT, &_c,
        "min=0.0 max=1.0 step=0.05");
    TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &_gravity,
        "min=-100 max=0 step=1");
    TwAddVarRW(DUC->g_pTweakBar, "Mass RB", TW_TYPE_FLOAT, &_mass,
        "min=0.0 max=100.0 step=1");
    TwAddVarRW(DUC->g_pTweakBar, "Size Cube", TW_TYPE_FLOAT, &_rbSize,
        "min=0.05 max=2.0 step=0.05");
    TwAddVarRW(DUC->g_pTweakBar, "Size Sphere", TW_TYPE_FLOAT, &_sphereRadius,
        "min=0.05 max=2.0 step=0.05");
}

void RigidBodySystemSimulator::reset() {
    _mouse.x = _mouse.y = 0;
    _trackmouse.x = _trackmouse.y = 0;
    _oldtrackmouse.x = _oldtrackmouse.y = 0;

    _rigidBodies.clear();
    _springs.clear();
}

void RigidBodySystemSimulator::drawFrame(
    ID3D11DeviceContext* pd3dImmediateContext) {
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,
        0.6 * Vec3(1.f, 1.f, 1.f));
    for (const auto& rb : _rigidBodies) {
        if (rb.isSphere)
            DUC->drawSphere(rb.position, Vec3(rb.width));
        else
            DUC->drawRigidBody(rb.worldMatrix);
    }
    for (const auto& s : _springs) {
        // calc positions for line
        const auto app1 = _rigidBodies[s._masspoint1].orientation.getRotMat() *
            s._relativeApplicationOffsets[0];
        const auto app2 = _rigidBodies[s._masspoint2].orientation.getRotMat() *
            s._relativeApplicationOffsets[1];
        DUC->beginLine();
        DUC->drawLine(
            _rigidBodies[s._masspoint1].position + app1, Vec3(1.f, 0.f, 0.f),
            _rigidBodies[s._masspoint2].position + app2, Vec3(1.f, 0.f, 0.f));
        DUC->endLine();
    }
    // for (const auto& rb : constRigidBodies) {
    //     DUC->drawRigidBody(rb.worldMatrix);
    // }

    // debug output for ray
    // DUC->beginLine();
    // DUC->drawLine(_rayOrigin, Vec3(0.f, 1.f, 0.f), _rayOrigin + _rayDirection,
    //              Vec3(0.f, 1.f, 0.f));
    // DUC->endLine();
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
    case 3:
        cout << "Demo4!\n";

        break;
    case 4:
        cout << "Demo5!\n";

        break;
    case 5:
        cout << "Demo6!\n";

        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    if (m_iTestCase == 3) {
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

        if (!rb.isSphere) {
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
    if (dist < rbA.width + rbB.width) {
        // collision happens
        info.isValid = true;

        Vec3 collisionNormal = getNormalized(direction);
        const float collisionDepth = ((rbA.width + rbB.width) - dist) / 2.;
        const Vec3 collisionPointWorld =
            rbB.position + collisionNormal * (rbB.width - collisionDepth);

        info.normalWorld = collisionNormal;
        info.depth = collisionDepth;
        info.collisionPointWorld = collisionPointWorld;
    }

    return info;
}

CollisionInfo
RigidBodySystemSimulator::checkCollisionSphereBox(RigidBody& rbA,
    RigidBody& rbB) {

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
    auto sphereCenterInLocalBoxSpace = rotTrans.inverse() * sphere.position;

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

    // now we check the distance from the closestPointWorldSpace to sphere center
    // in world space
    const auto direction = closestPointWorldSpace - sphere.position;
    const auto distFromSphereToBox =
        norm(closestPointWorldSpace - sphere.position);

    if (distFromSphereToBox < sphere.width) {

        info.isValid = true;

        Vec3 collisionNormal = getNormalized(direction);

        const float collisionDepth = sphere.width - distFromSphereToBox;
        const Vec3 collisionPointWorld =
            sphere.position + collisionNormal * (sphere.width - collisionDepth);

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

    std::pair<float, size_t> minRB = { std::numeric_limits<float>::max(), -1 };

    for (size_t i{ 0 }; i < _rigidBodies.size(); ++i) {
        if (_rigidBodies[i].fixed) {
            continue;
        }
        float tNear;
        bool intersecting = false;

        if (_rigidBodies[i].isSphere) {
            intersecting = calcRaySphereIntersection(_rigidBodies[i], _rayOrigin,
                _rayDirection, tNear);
        }
        else {
            intersecting = calcRayAABBIntersection(_rigidBodies[i], _rayOrigin,
                _rayDirection, tNear);
        }

        if (intersecting) {
            if (tNear < std::get<0>(minRB)) {
                std::get<0>(minRB) = tNear;
                std::get<1>(minRB) = i;
            }
            // std::cout << colPoint << std::endl;
        }
    }
    if (std::get<1>(minRB) != -1) {
        Vec3 colPoint = _rayOrigin + std::get<0>(minRB) * _rayDirection;
        applyForceOnBody(std::get<1>(minRB), colPoint, _rayDirection * _forceFactor);
    }
}

Vec3 RigidBodySystemSimulator::mulMat4Vec3(Mat4& mat, Vec3& vec) {
    Vec3 nvec(0.0);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            nvec[i] += (vec[j] * mat.value[j][i]);
        }
    }
    // std::cout << "nvec between loops: " << nvec << std::endl;
    //  assume normalized w coord
    // for (int i = 0; i < 3; i++) {
    //   nvec[i] += (1.0 * mat.value[3][i]);
    // }
    return nvec;
}

bool RigidBodySystemSimulator::calcRayAABBIntersection(RigidBody& rb,
    Vec3 rayOrigin,
    Vec3 rayDirection,
    float& out) {

    auto rbInverseWorld = rb.worldMatrix;
    rbInverseWorld = rbInverseWorld.inverse();

    // std::cout << "rbInverseWorld: " << rbInverseWorld << std::endl;

    // std::cout << "rbInverseWorld * rayDirection: "
    //           << rbInverseWorld * rayDirection << std::endl;
    // std::cout << "mulMat4Vec3(rbInverseWorld * rayDirection): "
    //           << mulMat4Vec3(rbInverseWorld, rayDirection) << std::endl;

    const auto localRay = std::array<Vec3, 2>{
        rbInverseWorld* rayOrigin,
            getNormalized(mulMat4Vec3(rbInverseWorld, rayDirection))};

    // std::cout << "rayOrigin: " << rayOrigin << std::endl;
    // std::cout << "rayDirection: " << rayDirection << std::endl;

    // std::cout << "localRay[0]: " << localRay[0] << std::endl;
    // std::cout << "localRay[1]: " << localRay[1] << std::endl;

    Vec3 invRayDirection =
        Vec3(1.0 / localRay[1].x, 1.0 / localRay[1].y, 1.0 / localRay[1].z);

    const auto localBoxMin = Vec3(-0.5, -0.5, -0.5);
    const auto localBoxMax = Vec3(0.5, 0.5, 0.5);

    const auto tMin = (localBoxMin - localRay[0]) * invRayDirection;
    const auto tMax = (localBoxMax - localRay[0]) * invRayDirection;

    // std::cout << "tMin: " << tMin << std::endl;
    // std::cout << "tMax: " << tMax << std::endl;

    const auto t1 = Vec3(std::min(tMin.x, tMax.x), std::min(tMin.y, tMax.y),
        std::min(tMin.z, tMax.z));
    const auto t2 = Vec3(std::max(tMin.x, tMax.x), std::max(tMin.y, tMax.y),
        std::max(tMin.z, tMax.z));

    // std::cout << "t1 " << t1 << std::endl;
    // std::cout << "t2 " << t2 << std::endl;

    float tNear = std::max({ t1.x, t1.y, t1.z });
    float tFar = std::min({ t2.x, t2.y, t2.z });

    // std::cout << "tNear " << tNear << std::endl;
    // std::cout << "tFar " << tFar << std::endl;

    if (tNear > tFar || tFar < 0) {
        // std::cout << "no intersection" << std::endl;
        return false;
    }
    else {
        // Transform intersection point back to world space
        // out = rayOrigin + tNear * rayDirection;
        out = tNear;
        // std::cout << "intersection" << std::endl;
        return true;
    }
}

bool RigidBodySystemSimulator::calcRaySphereIntersection(RigidBody& rb,
    Vec3 rayOrigin,
    Vec3 rayDirection,
    float& out) {
    const auto oc = rayOrigin - rb.position;
    float a = dot(rayDirection, rayDirection);
    float b = 2.0f * dot(oc, rayDirection);
    float c = dot(oc, oc) - (rb.width * rb.width);
    float discriminant = b * b - 4 * a * c;

    // std::cout << discriminant << std::endl;

    if (discriminant > 0) {
        float t = (-b - sqrt(discriminant)) / (2.0f * a);
        out = t;
        return true;
    }

    return false;
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
    _oldtrackmouse.x = x;
    _oldtrackmouse.y = y;
    _trackmouse.x = x;
    _trackmouse.y = y;

    RECT clientRect;
    GetClientRect(DXUTGetHWND(), &clientRect);

    const auto windowWidth = clientRect.right - clientRect.left;
    const auto windowHeight = clientRect.bottom - clientRect.top;

    // View and projection matrices
    XMMATRIX viewMatrix = DUC->g_camera.GetViewMatrix();
    XMMATRIX projectionMatrix = DUC->g_camera.GetProjMatrix();

    // Convert mouse coordinates to normalized device coordinates (-1 to 1)
    const auto ndcX =
        (static_cast<double>(x) / static_cast<double>(windowWidth)) * 2.0 - 1.0;
    const auto ndcY =
        1.0 - (static_cast<double>(y) / static_cast<double>(windowHeight)) * 2.0;

    // Create a ray in clip space
    XMFLOAT4 rayClip(ndcX, ndcY, 1.0f, 1.0f);

    XMVECTOR rayEye = XMVector4Transform(
        XMLoadFloat4(&rayClip), XMMatrixInverse(nullptr, projectionMatrix));
    rayEye = XMVectorSet(XMVectorGetX(rayEye), XMVectorGetY(rayEye), 1.0f, 0.0f);

    // Ray in world space
    XMVECTOR rayWorld =
        XMVector4Transform(rayEye, XMMatrixInverse(nullptr, viewMatrix));

    XMFLOAT3 rayDirectionDX;
    XMStoreFloat3(&rayDirectionDX, rayWorld);

    XMVECTOR camPos = DUC->g_camera.GetEyePt();
    XMFLOAT3 camPosF;
    XMStoreFloat3(&camPosF, camPos);

    Vec3 rayOrigin(camPosF.x, camPosF.y, camPosF.z);
    Vec3 rayDirection(rayDirectionDX.x, rayDirectionDX.y, rayDirectionDX.z);
    rayDirection = getNormalized(rayDirection);

    _rayOrigin = rayOrigin;
    _rayDirection = rayDirection;

    // Calculate intersection with ground plane
    Vec3 normal(0, 1, 0);
    float d = 0;
    float denom = dot(rayDirection, normal);
    if (abs(denom) >= 1e-4f) {
        float t = -(dot(normal, rayOrigin) + d) / denom;
        if (t > 1e-4) {
            _intersectionWithGroundPlane = rayOrigin + t * rayDirection;
        }
    }

    // std::cout << "rayOrigin: " << rayOrigin << std::endl;
    // std::cout << "rayDirection: " << rayDirection << std::endl;
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
    // std::cout << cross((loc - _rigidBodies[i].position), force) << std::endl;
    //  accumulate the force
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
        iit0 = iit1 = iit2 = (2.f * float(mass) * size.x * size.x) / 3.f;
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

void RigidBodySystemSimulator::addSpring(
    int masspoint1, int masspoint2, float initialLength,
    std::array<Vec3, 2>& relativeApplicationOffsets) {
    _springs.push_back(Spring(masspoint1, masspoint2, initialLength,
        relativeApplicationOffsets));
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

        applyForceOnBody(s._masspoint1,
            rb1.position + s._relativeApplicationOffsets[0], f1);
        applyForceOnBody(s._masspoint2,
            rb2.position + s._relativeApplicationOffsets[1], f2);
    }
}

void RigidBodySystemSimulator::applyDamping(
    std::vector<RigidBody>& rigidBodies) {
    for (auto& rb : rigidBodies) {
        rb.force -= _damping * rb.linearVelocity;
    }
}

void RigidBodySystemSimulator::onKey(UINT nChar) {
    switch (nChar) {
    case 0x53: {
        if (m_iTestCase == 3) {
            addRigidBody(_intersectionWithGroundPlane + Vec3(0, 0.3, 0),
                Vec3(_sphereRadius, _sphereRadius, _sphereRadius), 1, false,
                true);
        }
        break;
    }
    case 0x43: {
        if (m_iTestCase == 3) {
            addRigidBody(_intersectionWithGroundPlane + Vec3(0, 0.3, 0),
                Vec3(_rbSize, _rbSize, _rbSize), 1, false, false);
        }
        break;
    }
    default:
        break;
    }
}