#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 
#include "collisionDetect.h"

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);


	// New Additions
	struct RigidBody {
		Vec3 position, lVelocity = Vec3(), aVelocity = Vec3(), aMomentum = Vec3(), force = Vec3(), torque = Vec3();
		Vec3 size; // width, height, depth
		Quat orientation = Quat(0.0f, 0.0f, 0.0f, 1.0f);
		Mat4 objToWorldMat, inertiaTensorInv;

		float mass = 1.0f;
		bool isFixed = false;
	};
	const int MAX_LINEAR_VELOCITY = 5, MAX_ANGULAR_VELOCITY = 5;

	void applyImpulse(CollisionInfo& collisionInfo, RigidBody& rbA, RigidBody& rbB);
	Mat4 computeWorldMat(RigidBody& rb);
	float degToRad(float degree);
	void integrateVelPos(float timeStep);

	void addFixedRigidBody(Vec3 position, Vec3 size, int mass);
	void spawnWalls();
	void checkAndApplyCollision(RigidBody& rbA, RigidBody& rbB);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	Vec3  m_vfMovableObjectFinalPos;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;


	// New Additions
	std::vector<RigidBody> rigidBodies, fixedRigidBodies;
	float c = 0.5f, dampingLinear = 0.0f, dampingAngular = 0.0f, gravity = 0.0f;
	bool isFirstTime = true;
};
#endif