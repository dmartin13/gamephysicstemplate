#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// rest to be implemented

	// Initializing the 2D grid
	temperatureGrid.resize(m);
	for (int i = 0; i < m; ++i) {
		temperatureGrid[i].resize(n, 0.0f);
	}
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	// Resetting the grid & heat source
	vector<vector<float>> T(m, vector<float>(n, 0.0f));
	temperatureGrid = T;
	temperatureGrid[m / 2][n / 2] = 5.0f;
	temperatureGrid[m - 2][n - 2] = 5.0f;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		reset();
		break;
	case 1:
		cout << "Implicit solver!\n";
		reset();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}


void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {

	// Set initial conditions (Heat source)
	temperatureGrid[m / 2][n / 2] = 5.0f;
	temperatureGrid[m - 2][n - 2] = 5.0f;

	vector<vector<float>> T_temp = temperatureGrid;

	// Update temperature values using explicit method
	for (int i = 1; i < m - 1; ++i)
		for (int j = 1; j < n - 1; ++j)
			T_temp[i][j] = temperatureGrid[i][j] + D * (temperatureGrid[i + 1][j] + temperatureGrid[i - 1][j] + temperatureGrid[i][j + 1] + temperatureGrid[i][j - 1] - 4 * temperatureGrid[i][j]) * timeStep;

	// Copy updated values back to the original grid
	temperatureGrid = T_temp;

	// Ensure boundary conditions (temperature is always zero at boundaries)
	for (int i = 0; i < m; ++i)
		temperatureGrid[i][0] = temperatureGrid[i][n - 1] = 0.0f;

	for (int j = 0; j < n; ++j)
		temperatureGrid[0][j] = temperatureGrid[m - 1][j] = 0.0f;
	
	drawObjects();
}




void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {

	// Set initial conditions (Heat source)
	temperatureGrid[m / 2][n / 2] = 5.0f;
	temperatureGrid[m - 2][n - 2] = 5.0f;

	const int nx = temperatureGrid.size();
	const int ny = temperatureGrid[0].size();
	const int N = nx * ny;

	SparseMatrix<float> A(N);
	std::vector<float> b(N);

	// Assemble the system matrix A and the right-hand side b
	for (int i = 1; i < nx - 1; ++i) {
		for (int j = 1; j < ny - 1; ++j) {
			int index = i + nx * j;

			// Assemble the matrix A and vector b here
			// Modify the following lines based on your implicit diffusion discretization

			// Set the diagonal entry
			A.set_element(index, index, 1 + 4 * D * timeStep);

			// Set the neighboring entries, adjust the coefficients based on your scheme
			A.set_element(index, index - 1, -D * timeStep);
			A.set_element(index, index + 1, -D * timeStep);
			A.set_element(index, index - nx, -D * timeStep);
			A.set_element(index, index + nx, -D * timeStep);

			// Set the right-hand side vector
			b[index] = temperatureGrid[i][j] + D * timeStep * (
				temperatureGrid[i + 1][j] + temperatureGrid[i - 1][j] +
				temperatureGrid[i][j + 1] + temperatureGrid[i][j - 1] - 4 * temperatureGrid[i][j]
				);
		}
	}

	// Perform solve
	float pcg_target_residual = 1e-05;
	int pcg_max_iterations = 1000;
	float ret_pcg_residual = 1e10;
	int ret_pcg_iterations = -1;

	SparsePCGSolver<float> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<float> x(N, 0.0);
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 2);

	// Update the temperatureGrid with the solution
	for (int i = 1; i < nx - 1; ++i) {
		for (int j = 1; j < ny - 1; ++j) {
			int index = i + nx * j;

			// Extract temperature from solution vector and update temperatureGrid
			temperatureGrid[i][j] = x[index];
		}
	}

	drawObjects();
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	float sphereSize = 0.1f;

	// Hard-code the first sphere position
	Vec3 position = Vec3(-0.5, -0.5, 0);

	// Iterate through the temperature grid and draw spheres
	for (size_t i = 0; i < temperatureGrid.size(); ++i) {
		for (size_t j = 0; j < temperatureGrid[i].size(); ++j) {
			float temperature = temperatureGrid[i][j];

			// Map temperature to a color (you can adjust this based on your desired color mapping)
			Vec3 color = Vec3(temperature, 0.0f, 1.0 - temperature);

			// Set up lighting with the determined color
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);

			// Draw the sphere at the current position
			DUC->drawSphere(position, Vec3(sphereSize));

			// Update the position based on grid indices
			position.x += sphereSize;  // Adjust based on your grid spacing
		}

		// Move to the next row in the grid
		position.x = -0.5;
		position.y += sphereSize;  // Adjust based on your grid spacing
	}
}




void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
