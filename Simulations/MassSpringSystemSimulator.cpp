#include "MassSpringSystemSimulator.h"

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 1.0f;
	m_fStiffness = 1.0f;
	m_fDamping = 0.0f;
	m_iIntegrator = 0;

	m_externalForce = Vec3();

	// Creating some mass points and a spring
	MassPoint mp1, mp2;
	mp1.position = Vec3(0, 2, 0);
	mp1.initPos = mp1.position;
	mp1.velocity = Vec3(0, -1, 0);
	mp1.initVel = mp1.velocity;
	mp1.mass = 10.0f;
	mp1.isFixed = false;

	mp2.position = Vec3(0, -0.4, 0);
	mp2.initPos = mp2.position;
	mp2.velocity = Vec3(0, 1, 0);
	mp2.initVel = mp2.velocity;
	mp2.mass = 10.0f;
	mp2.isFixed = false;

	Spring spring;
	spring.initialLength = 3.0f;
	spring.stiffness = 30.0f;

	massPoints.push_back(mp1);
	spring.massPoint1 = massPoints.size() - 1;
	massPoints.push_back(mp2);
	spring.massPoint2 = massPoints.size() - 1;
	springs.push_back(spring);

}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler,Midpoint,Leap Frog";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	switch (m_iTestCase)
	{
	case 0:
		for (int i{ 0 }; i < massPoints.size(); i++)
		{
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
			DUC->drawSphere(massPoints.at(i).position, Vec3(0.05f));
		}
		break;
	case 1: 
		for (int i{ 0 }; i < massPoints.size(); i++)
		{
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
			DUC->drawSphere(massPoints.at(i).position, Vec3(0.05f));
		}
		break;
	case 2: 
		for (int i{ 0 }; i < massPoints.size(); i++)
		{
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
			DUC->drawSphere(massPoints.at(i).position, Vec3(0.05f));
		}
		break;
	defualt:
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Euler !\n";
		// Reset position & velocity
		for (int i{ 0 }; i < massPoints.size(); i++)
		{
			massPoints.at(i).position = massPoints.at(i).initPos;
			massPoints.at(i).velocity = massPoints.at(i).initVel;
		}
		break;
	case 1:
		cout << "Midpoint !\n";
		for (int i{ 0 }; i < massPoints.size(); i++)
		{
			massPoints.at(i).position = massPoints.at(i).initPos;
			massPoints.at(i).velocity = massPoints.at(i).initVel;
		}
		break;
	case 2:
		cout << "Leap Frog !\n";
		for (int i{ 0 }; i < massPoints.size(); i++)
		{
			massPoints.at(i).position = massPoints.at(i).initPos;
			massPoints.at(i).velocity = massPoints.at(i).initVel;
		}
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	//Point2D mouseDiff;
	//mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	//mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	//if (mouseDiff.x != 0 || mouseDiff.y != 0)
	//{
	//	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
	//	worldViewInv = worldViewInv.inverse();
	//	Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
	//	Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
	//	// find a proper scale!
	//	float inputScale = 0.001f;
	//	inputWorld = inputWorld * inputScale;
	//	m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	//}
	//else {
	//	m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	//}
}

void MassSpringSystemSimulator::integratePosition(MassPoint& p, float timeStep, bool isMidstep)
{
	int midStepValue = 1.0f;
	if (isMidstep)
		midStepValue = 0.5f;
	else
		midStepValue = 1.0f;

	Vec3 newPos = p.position + (p.velocity.operator*(timeStep));
	/*if (newPos.X <= -6.0f)
		newPos.X = -6.0f;
	if (newPos.X >= 6.0f)
		newPos.X = 6.0f;*/
	if (newPos.Y <= -0.5f)
		newPos.Y = -0.5f;	
	p.position = newPos;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0: // Euler Method
		for (int i{ 0 }; i < springs.size(); i++)
		{
			// Getting the mass point for a string
			Spring &spring = springs.at(i);
			MassPoint &massPoint1 = massPoints.at(spring.massPoint1);
			MassPoint &massPoint2 = massPoints.at(spring.massPoint2);

			// Calculate the force
			Vec3 pointDiff = massPoint1.position - massPoint2.position;
			spring.currentLength = sqrtf(powf(pointDiff.X, 2) + powf(pointDiff.Y, 2) + powf(pointDiff.Z, 2));
			Vec3 forceOnMp1 = (-spring.stiffness) * (spring.currentLength - spring.initialLength) * (pointDiff.operator/(spring.currentLength));
			Vec3 forceOnMp2 = forceOnMp1.operator*(-1);

			// Calculate accelerations using Euler
			Vec3 accAtOldPosMp1 = forceOnMp1.operator/(massPoint1.mass) - (m_fDamping * massPoint1.velocity);
			Vec3 accAtOldPosMp2 = forceOnMp2.operator/(massPoint2.mass) - (m_fDamping * massPoint2.velocity);

			// Calculate positions using Euler
			integratePosition(massPoint1, timeStep);
			integratePosition(massPoint2, timeStep);

			// Calculate velocities using Euler
			massPoint1.velocity = massPoint1.velocity + (accAtOldPosMp1.operator*(timeStep));
			massPoint2.velocity = massPoint2.velocity + (accAtOldPosMp2.operator*(timeStep));
			
			// #TODO --> Add gravity, and prevent position y from going below 0

			/*std::cout << "Length: " << spring.currentLength - spring.initialLength << std::endl;
			std::cout << "Mp1 Pos: " << massPoint1.position << std::endl;
			std::cout << "Mp1 Vel: " << massPoint1.velocity << std::endl << std::endl;
			std::cout << "Mp2 Pos: " << massPoint2.position << std::endl;
			std::cout << "Mp2 Vel: " << massPoint2.velocity << std::endl << std::endl;*/
		}
		break;
	case 1: // Midpoint Method
		for (int i{ 0 }; i < springs.size(); i++)
		{
			// Getting the mass point for a string
			Spring& spring = springs.at(i);
			MassPoint& massPoint1 = massPoints.at(spring.massPoint1);
			MassPoint& massPoint2 = massPoints.at(spring.massPoint2);

			// Calculate Initial Forces
			Vec3 pointDiff = massPoint1.position - massPoint2.position;
			spring.currentLength = sqrtf(powf(pointDiff.X, 2) + powf(pointDiff.Y, 2) + powf(pointDiff.Z, 2));
			Vec3 forceOnMp1 = (-spring.stiffness) * (spring.currentLength - spring.initialLength) * (pointDiff.operator/(spring.currentLength));
			Vec3 forceOnMp2 = forceOnMp1.operator*(-1);

			// Calculate Initial Acceleration
			Vec3 accAtOldPosMp1 = forceOnMp1.operator/(massPoint1.mass);
			Vec3 accAtOldPosMp2 = forceOnMp2.operator/(massPoint2.mass);


			/* ========================== Midstep ========================== */
			// Calculate Midstep Positions
			integratePosition(massPoint1, timeStep, true);
			integratePosition(massPoint2, timeStep, true);

			// Calculate Midstep Velocities
			Vec3 midVelMp1 = massPoint1.velocity + (accAtOldPosMp1.operator*(timeStep*0.5f));
			Vec3 midVelMp2 = massPoint2.velocity + (accAtOldPosMp2.operator*(timeStep*0.5f));

			// Calculate Midstep Force
			pointDiff = massPoint1.position - massPoint2.position;
			spring.currentLength = sqrtf(powf(pointDiff.X, 2) + powf(pointDiff.Y, 2) + powf(pointDiff.Z, 2));
			forceOnMp1 = (-spring.stiffness) * (spring.currentLength - spring.initialLength) * (pointDiff.operator/(spring.currentLength));
			forceOnMp2 = forceOnMp1.operator*(-1);

			// Calculate Midstep Acceleration
			Vec3 accAtMidPosMp1 = forceOnMp1.operator/(massPoint1.mass) - (m_fDamping * midVelMp1);
			Vec3 accAtMidPosMp2 = forceOnMp2.operator/(massPoint2.mass) - (m_fDamping * midVelMp2);
			/* ========================== Midstep ========================== */


			// Calculate Final Positions
			integratePosition(massPoint1, timeStep);
			integratePosition(massPoint2, timeStep);

			// Calculate Final Velocities
			massPoint1.velocity = massPoint1.velocity + (accAtMidPosMp1.operator*(timeStep));
			massPoint2.velocity = massPoint2.velocity + (accAtMidPosMp2.operator*(timeStep));
		}
		break;
	case 2: // Leap-Frog Method
		for (int i{ 0 }; i < springs.size(); i++)
		{
			// Getting the mass point for a string
			Spring& spring = springs.at(i);
			MassPoint& massPoint1 = massPoints.at(spring.massPoint1);
			MassPoint& massPoint2 = massPoints.at(spring.massPoint2);

			// Calculate the force
			Vec3 pointDiff = massPoint1.position - massPoint2.position;
			spring.currentLength = sqrtf(powf(pointDiff.X, 2) + powf(pointDiff.Y, 2) + powf(pointDiff.Z, 2));
			Vec3 forceOnMp1 = (-spring.stiffness) * (spring.currentLength - spring.initialLength) * (pointDiff.operator/(spring.currentLength));
			Vec3 forceOnMp2 = forceOnMp1.operator*(-1);

			// Calculate accelerations
			Vec3 accAtOldPosMp1 = forceOnMp1.operator/(massPoint1.mass) - (m_fDamping * massPoint1.velocity);
			Vec3 accAtOldPosMp2 = forceOnMp2.operator/(massPoint2.mass) - (m_fDamping * massPoint2.velocity);

			// Calculate velocities (Velocity is calculated first, because Leap-Frog)
			massPoint1.velocity = massPoint1.velocity + (accAtOldPosMp1.operator*(timeStep));
			massPoint2.velocity = massPoint2.velocity + (accAtOldPosMp2.operator*(timeStep));

			// Calculate positions
			integratePosition(massPoint1, timeStep);
			integratePosition(massPoint2, timeStep);
		}
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint massPoint = MassPoint();
	massPoint.position = position;
	massPoint.velocity = Velocity;
	massPoint.isFixed = isFixed;
	massPoint.mass = m_fMass;
	massPoint.damping = m_fDamping;

	massPoints.push_back(massPoint);

	// Returns the index of the newly added mass point
	return massPoints.size()-1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring spring = Spring();
	spring.massPoint1 = masspoint1;
	spring.massPoint2 = masspoint2;
	spring.initialLength = initialLength;

	springs.push_back(spring);
}

void MassSpringSystemSimulator::setMass(float mass) {
	if (mass > 0.0f)
		m_fMass = mass;
	/*else
		m_fMass = 0.0001f;*/
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	if (stiffness >= 0.0f)
		m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	// Not sure if there should be a condition
	m_fDamping = damping;
}

int MassSpringSystemSimulator::getNumberOfMassPoints() { return massPoints.size(); }
int MassSpringSystemSimulator::getNumberOfSprings() { return springs.size(); }
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) { return massPoints.at(index).position; }
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) { return massPoints.at(index).velocity; }
void MassSpringSystemSimulator::applyExternalForce(Vec3 force) { m_externalForce = force; }
