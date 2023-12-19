#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid() : n_{ 0 }, m_{ 0 } {}

Grid::Grid(unsigned int m, unsigned int n) : n_{ n }, m_{ m } {
    values.resize(n * m);
}

unsigned int Grid::GetN() { return n_; }
unsigned int Grid::GetM() { return m_; }

std::vector<double>& Grid::GetValues() { return values; }
void Grid::SetValues(std::vector<double>& newValues) { values = newValues; }

double Grid::GetValueAt(unsigned int x, unsigned int y) {
    return values[y * n_ + x];
}

void Grid::SetValueAt(double value, unsigned int x, unsigned int y) {
    values[y * n_ + x] = value;
}

void DiffusionSimulator::setUpInitialScenario() {
    // this (should) create a flat area with a plateau in the middle
    int plateauSizeY = T.GetM() / 2;
    int plateauSizeX = T.GetN() / 2;
    for (int y = 0; y < T.GetM(); ++y) {
        for (int x = 0; x < T.GetN(); ++x) {
            // boundaries
            if (x == 0 || x == (T.GetN() - 1) || y == 0 || y == (T.GetM() - 1)) {
                T.SetValueAt(0, x, y);
                continue;
            }
            if ((x >= (plateauSizeX - (plateauSizeX / 2))) &&
                (x <= (plateauSizeX + (plateauSizeX / 2))) &&
                (y >= (plateauSizeY - (plateauSizeY / 2))) &&
                (y <= (plateauSizeY + (plateauSizeY / 2)))) {
                T.SetValueAt(1., x, y);
            }
            else {
                T.SetValueAt(-1., x, y);
            }
        }
    }
}

void Grid::Resize(unsigned int m, unsigned int n) {
    m_ = m;
    n_ = n;
    values.resize(n * m);
}

DiffusionSimulator::DiffusionSimulator() {
    m_iTestCase = 0;
    m_vfMovableObjectPos = Vec3();
    m_vfMovableObjectFinalPos = Vec3();
    m_vfRotate = Vec3();

    T = Grid(m, n);
}

const char* DiffusionSimulator::getTestCasesStr() {
    return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;

    TwAddVarRW(DUC->g_pTweakBar, "M", TW_TYPE_UINT32, &m, "");
    TwAddVarRW(DUC->g_pTweakBar, "N", TW_TYPE_UINT32, &n, "");
    TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha,
        "step=0.1 min=0.0");

    setUpInitialScenario();
}

void DiffusionSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    m_vfMovableObjectPos = Vec3(0, 0, 0);
    m_vfRotate = Vec3(0, 0, 0);

    switch (m_iTestCase) {
    case 0:
        cout << "Explicit solver!\n";
        break;
    case 1:
        cout << "Implicit solver!\n";
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void DiffusionSimulator::diffuseTemperatureExplicit(double timeStep) {
    // this should use the default copy constructor to create a copy of the
    // current values
    auto oldValues(T);

    // we start with the second element and iterate to the second last to ensure
    // the boundaries stay 0
    for (unsigned int y = 1; y < T.GetM() - 1; ++y) {
        for (unsigned int x = 1; x < T.GetN() - 1; ++x) {
            // solve the heat equation

            // current value at (x,y)
            auto t_n = oldValues.GetValueAt(x, y);

            // note: In our simulator the grid points are equally spaced with a
            // distance of 1. Therefore h = h^2 = 1
            auto factor = (alpha * timeStep) /* /h^2 */;

            // T^n_(i+1,j)
            auto t_n_xp = oldValues.GetValueAt(x + 1, y);

            // T^n_(i-1,j)
            auto t_n_xm = oldValues.GetValueAt(x - 1, y);

            // T^n_(i,j+1)
            auto t_n_yp = oldValues.GetValueAt(x, y + 1);

            // T^n_(i,j-1)
            auto t_n_ym = oldValues.GetValueAt(x, y - 1);

            // update step
            T.SetValueAt(
                t_n + factor * (t_n_xp + t_n_xm + t_n_yp + t_n_ym - (4 * t_n)), x, y);
        }
    }
}

void DiffusionSimulator::diffuseTemperatureImplicit(double timeStep) {
    // solve A T = b

    // This is just an example to show how to work with the PCG solver,
    const int nx = T.GetN();
    const int ny = T.GetM();
    const int N = nx * ny;

    // This is the part where you have to assemble the system matrix A and the
    // right-hand side b!

    SparseMatrix<double> A(N);
    std::vector<double>& b = T.GetValues();

    // set diagonal values of boundaries to 1
    A.set_element(0, 0, 1.0);
    A.set_element(N - 1, N - 1, 1.0);

    // we only do this for non-boundary cells
    for (size_t y = 1; y < ny - 1; ++y) {
        for (size_t x = 1; x < nx - 1; ++x) {
            // this is the index how we store elements in T
            size_t idx = y * nx + x;

            // set leftmost element (j-1)
            A.set_element(idx, idx - nx, -alpha * timeStep);
            // element directly left (i-1)
            A.set_element(idx, idx - 1, -alpha * timeStep);
            // the diagonal value
            A.set_element(idx, idx, 1.0 + 4.0 * alpha * timeStep);
            // element directly right (i+1)
            A.set_element(idx, idx + 1, -alpha * timeStep);
            // set rightmost element (j+1)
            A.set_element(idx, idx + nx, -alpha * timeStep);
        }
    }

    // perform solve
    double pcg_target_residual = 1e-05;
    double pcg_max_iterations = 1000;
    double ret_pcg_residual = 1e10;
    int ret_pcg_iterations = -1;

    SparsePCGSolver<double> solver;
    solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97,
        0.25);

    std::vector<double> x(N);
    for (int j = 0; j < N; ++j) {
        x[j] = 0.;
    }

    // preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
    solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

    // Final step is to extract the grid temperatures from the solution vector x
    T.SetValues(x);
}

void DiffusionSimulator::simulateTimestep(float timeStep) {
    // check if grid size changed
    if (m != T.GetM() || n != T.GetN()) {
        T.Resize(m, n);
        setUpInitialScenario();
    }

    // update current setup for each frame
    switch (m_iTestCase) {
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

void DiffusionSimulator::drawObjects() {
    // double minVal = 100000000.0;
    // double maxVal = -100000000.0;
    auto sphereSize = 0.1;
    auto halfDomainSizeX = T.GetN() * sphereSize * 0.5;
    auto halfDomainSizeY = T.GetM() * sphereSize * 0.5;

    for (unsigned int y = 0; y < T.GetM(); ++y) {
        for (unsigned int x = 0; x < T.GetN(); ++x) {

            // if (T.GetValueAt(x, y) < minVal) {
            //   minVal = T.GetValueAt(x, y);
            // }
            // if (T.GetValueAt(x, y) > maxVal) {
            //   maxVal = T.GetValueAt(x, y);
            // }

            // calculate color
            // auto color = Vec3(1., (1. + T.GetValueAt(x, y)) / 2., (1. +
            // T.GetValueAt(x, y)) / 2.);

            // auto color = Vec3(T.GetValueAt(x, y), 0.0f, 1.0 - T.GetValueAt(x, y));

            // remap values from (-1, 1) to (0, 1)
            auto xP = ((-T.GetValueAt(x, y)) / 2.) + 0.5;
            auto yP = (xP - 0.5) * (xP - 0.5) * 4.;
            auto red = abs((xP - 0.5) * 2.) * yP;
            auto green = (1. - xP) * yP;
            auto blue = (1. - xP) * yP;

            auto color = Vec3(red, green, blue);

            DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 15., color);
            auto pos = Vec3(-halfDomainSizeX + x * sphereSize,
                -halfDomainSizeY + y * sphereSize, 0);
            DUC->drawSphere(pos, Vec3(sphereSize, sphereSize, sphereSize));
        }
    }
    // std::cout << "minVal: " << minVal << ", maxVal: " << maxVal << std::endl;
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
    drawObjects();
}

void DiffusionSimulator::onClick(int x, int y) {
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y) {
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}
