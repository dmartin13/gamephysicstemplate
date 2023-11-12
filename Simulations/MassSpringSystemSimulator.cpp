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
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1: Step,Demo 2: Euler,Demo 3: Midpoint,Demo 4: Complex,Demo 5: Leap Frog";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	case 3: break;
	case 4: break;
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
		//break;
	case 1:
		//break;
	case 2: 
		//break;
	case 3:
		//break;
	case 4: 
		for (int i{ 0 }; i < springs.size(); i++)
		{
			Vec3 massPoint1Pos = massPoints.at(springs.at(i).massPoint1).position;
			Vec3 massPoint2Pos = massPoints.at(springs.at(i).massPoint2).position;

			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
			DUC->drawSphere(massPoint1Pos, Vec3(0.05f));
			DUC->drawSphere(massPoint2Pos, Vec3(0.05f));

			DUC->beginLine();
			DUC->drawLine(massPoint1Pos, Vec3(1), massPoint2Pos, Vec3(1));
			DUC->endLine();
		}
		break;
	defualt:
		break;
	}
}

void MassSpringSystemSimulator::createTwoMassPoints()
{
	MassPoint mp1, mp2;
	mp1.position = Vec3(0, 0, 0);
	mp1.velocity = Vec3(-1, 0, 0);
	mp1.mass = 10.0f;
	mp1.isFixed = false;

	mp2.position = Vec3(0, 2, 0);
	mp2.velocity = Vec3(1, 0, 0);
	mp2.mass = 10.0f;
	mp2.isFixed = false;

	Spring spring;
	spring.initialLength = 3.0f;
	spring.stiffness = 30.0f;

	massPoints.clear();
	springs.clear();

	massPoints.push_back(mp1);
	spring.massPoint1 = massPoints.size() - 1;
	massPoints.push_back(mp2);
	spring.massPoint2 = massPoints.size() - 1;
	springs.push_back(spring);
}

void MassSpringSystemSimulator::createTenSprings()
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	std::uniform_real_distribution<float> randVel(-0.5f, 0.5f);

	massPoints.clear();
	springs.clear();
	for (int i{ 0 }; i < 10; i++)
	{
		MassPoint mp1, mp2;
		mp1.position = Vec3(randPos(eng), randPos(eng), randPos(eng));
		mp1.velocity = Vec3(randVel(eng), randVel(eng), randVel(eng));
		mp1.mass = 3.0f;
		mp1.isFixed = false;

		mp2.position = Vec3(randPos(eng), randPos(eng), randPos(eng));
		mp2.velocity = Vec3(randVel(eng), randVel(eng), randVel(eng));
		mp2.mass = 3.0f;
		mp2.isFixed = false;

		Spring spring;
		spring.initialLength = 0.5f;
		spring.stiffness = 20.0f;

		massPoints.push_back(mp1);
		spring.massPoint1 = massPoints.size() - 1;
		massPoints.push_back(mp2);
		spring.massPoint2 = massPoints.size() - 1;
		springs.push_back(spring);
	}
}

bool firstTimeOneStep = true;
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1: Step ! \n";
		createTwoMassPoints();
		firstTimeOneStep = true;
		break;
	case 1:
		cout << "Demo 2: Euler !\n";
		createTwoMassPoints();
		break;
	case 2:
		cout << "Demo 3: Midpoint !\n";
		createTwoMassPoints();
		break;
	case 3:
		cout << "Demo 4: Complex\n";
		createTenSprings();
		break;
	case 4:
		cout << "Demo 5: Leap Frog !\n";
		createTwoMassPoints();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

void MassSpringSystemSimulator::integratePosition(MassPoint& p, float timeStep, bool isMidstep)
{
	if (!p.isFixed)
	{
		int midStepValue = 1.0f;
		if (isMidstep)
			midStepValue = 0.5f;
		else
			midStepValue = 1.0f;

		Vec3 newPos = p.position + (p.velocity.operator*(timeStep));
		if (m_iTestCase == 3) // Apply just for the complex case
		{
			if (newPos.X < -0.5f)
				newPos.X = -0.5f;
			if (newPos.X > 0.5f)
				newPos.X = 0.5f;

			if (newPos.Z > 0.5f)
				newPos.Z = 0.5f;
			if (newPos.Z < -0.5f)
				newPos.Z = -0.5f;

			if (newPos.Y < -0.5f)
				newPos.Y = -0.5f;
			if (newPos.Y > 0.5f)
				newPos.Y = 0.5f;
		}
		p.position = newPos;
	}
}

void MassSpringSystemSimulator::integrateVelocity(MassPoint& p, Vec3 acc, float timeStep)
{
	p.velocity = p.velocity + (acc.operator*(timeStep));
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0: // 1 Step
		if (firstTimeOneStep)
		{
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

				// Calculate accelerations using Euler
				Vec3 accAtOldPosMp1 = forceOnMp1.operator/(massPoint1.mass) - (m_fDamping * massPoint1.velocity);
				Vec3 accAtOldPosMp2 = forceOnMp2.operator/(massPoint2.mass) - (m_fDamping * massPoint2.velocity);

				// Calculate positions using Euler
				integratePosition(massPoint1, timeStep);
				integratePosition(massPoint2, timeStep);

				// Calculate velocities using Euler
				integrateVelocity(massPoint1, accAtOldPosMp1, timeStep);
				integrateVelocity(massPoint2, accAtOldPosMp2, timeStep);

				std::cout << "Mp1 Pos: " << massPoint1.position << std::endl;
				std::cout << "Mp1 Vel: " << massPoint1.velocity << std::endl << std::endl;
				std::cout << "Mp2 Pos: " << massPoint2.position << std::endl;
				std::cout << "Mp2 Vel: " << massPoint2.velocity << std::endl << std::endl;
			}
			firstTimeOneStep = false;
		}
		break;
	case 1: // Euler Method
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
			integrateVelocity(massPoint1, accAtOldPosMp1, timeStep);
			integrateVelocity(massPoint2, accAtOldPosMp2, timeStep);
		}
		break;
	case 2: // Midpoint Method
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
			integrateVelocity(massPoint1, accAtMidPosMp1, timeStep);
			integrateVelocity(massPoint2, accAtMidPosMp2, timeStep);
		}
		break;
	case 3: // Complext simulation of 10 springs
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
			Vec3 accAtOldPosMp1 = forceOnMp1.operator/(massPoint1.mass) - (0.0f * massPoint1.velocity) + Vec3(0, -9.8, 0);
			Vec3 accAtOldPosMp2 = forceOnMp2.operator/(massPoint2.mass) - (0.0f * massPoint2.velocity) + Vec3(0, -9.8, 0);

			// Calculate velocities (Velocity is calculated first, because Leap-Frog)
			integrateVelocity(massPoint1, accAtOldPosMp1, timeStep);
			integrateVelocity(massPoint2, accAtOldPosMp2, timeStep);

			// Calculate positions
			integratePosition(massPoint1, timeStep);
			integratePosition(massPoint2, timeStep);
		}
		break;
	case 4: // Leap-Frog Method
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
			integrateVelocity(massPoint1, accAtOldPosMp1, timeStep);
			integrateVelocity(massPoint2, accAtOldPosMp2, timeStep);

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
