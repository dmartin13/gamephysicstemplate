#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
public:
	Grid();
	Grid(unsigned int m, unsigned int n);
	std::vector<double>& GetValues();
	void SetValues(std::vector<double>& newValues);
	unsigned int GetN();
	unsigned int GetM();
	double GetValueAt(unsigned int x, unsigned int y);
	void SetValueAt(double value, unsigned int x, unsigned int y);
	void Resize(unsigned int m, unsigned int n);

private:
	std::vector<double> values;
	unsigned int m_;
	unsigned int n_;
};

class DiffusionSimulator : public Simulator {
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char* getTestCasesStr();
	const char* getIntegratorStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(double timeStep);
	void diffuseTemperatureImplicit(double timeStep);

	void setUpInitialScenario();
	void setUpInitialScenarioPlateu();
	void setUpInitialScenarioRandom1();
	void setUpInitialScenarioRandom2();

private:
	// Attributes
	Vec3 m_vfMovableObjectPos;
	Vec3 m_vfMovableObjectFinalPos;
	Vec3 m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;
	int m{ 16 };
	int n{ 16 };
	double alpha = 0.3;
	int scenario = 0;
	int scenarioOld = 0;
};

#endif