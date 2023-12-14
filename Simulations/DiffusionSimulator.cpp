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

	//for (auto& row : temperatureGrid)
		//row.clear();

	// Clear the outer vector
	//temperatureGrid.clear();
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

	// Compute the diffusion coefficient (D)
	float D = 0.1f; // Higher = faster spreading (0.1 is good for now)

	// Set initial conditions (e.g., a heat source)
	temperatureGrid[m / 2][n / 2] = 1.0f;

	// Perform explicit Euler scheme for diffusion
	for (int step = 0; step < 100; ++step) {

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
	}

	drawObjects();
}




void DiffusionSimulator::diffuseTemperatureImplicit() {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	const int nx = 5;
	const int ny = 5;
	const int nz = 5;
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
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
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	float sphereSize = 0.1f;

	// Iterate through the temperature grid and draw spheres
	for (size_t i = 0; i < temperatureGrid.size(); ++i) {
		for (size_t j = 0; j < temperatureGrid[i].size(); ++j) {
			float temperature = temperatureGrid[i][j];

			// Map temperature to a color (you can adjust this based on your desired color mapping)
			Vec3 color = Vec3(temperature, 0.0f, 1.0 - temperature);

			// Set up lighting with the determined color
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);

			// Calculate the position of the sphere based on grid indices
			Vec3 position(i * sphereSize, j * sphereSize, 0.0f);

			// Draw the sphere
			DUC->drawSphere(position, Vec3(sphereSize));
		}
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
