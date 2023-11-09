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
	/*switch (m_iTestCase)
	{
	case 0: drawMovableTeapot(); break;
	case 1: drawSomeRandomObjects(); break;
	case 2: drawTriangle(); break;
	}*/
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	/*switch (m_iTestCase)
	{
	case 0:
		cout << "Teapot !\n";
		m_vfMovableObjectPos = Vec3(0, 0, 0);
		m_vfRotate = Vec3(0, 0, 0);
		break;
	case 1:
		cout << "Random Object!\n";
		m_iNumSpheres = 100;
		m_fSphereSize = 0.05f;
		break;
	case 2:
		cout << "Triangle !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}*/
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

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		break;
	case 1:
		break;
	case 2:
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
