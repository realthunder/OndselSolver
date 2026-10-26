/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <assert.h>
#include <exception>

#include "PosICNewtonRaphson.h"
#include "SingularMatrixError.h"
#include "SystemSolver.h"
#include "Part.h"
#include "Constraint.h"
#include "CREATE.h"
#include "GESpMatParPvPrecise.h"
#include "GESpMatFullPvPosIC.h"
#include "SparseLUSolver.h"

using namespace MbD;

void PosICNewtonRaphson::run()
{
	while (true) {
		try {
			//VectorNewtonRaphson::run();   //Inline to help debugging
			restoring = false;
			preRun();
			initializeLocally();
			initializeGlobally();
			iterate();
			postRun();
			break;
		}
		catch (const SingularMatrixError& ex) {
			auto redundantEqnNos = ex.getRedundantEqnNos();
			auto nRedundant = system->allRedundantConstraints()->size();
			system->partsJointsMotionsDo([&](std::shared_ptr<Item> item) { item->removeRedundantConstraints(redundantEqnNos); });
			if (system->allRedundantConstraints()->size() == nRedundant) {
				// Nothing new set aside: the next round would solve the same
				// system from the same start and end here again, for ever.
				std::string str("MbD: Singular matrix with no redundant constraint left to remove.");
				system->logString(str);
				throw SimulationStoppingError("singular matrix, no redundant constraint found");
			}
			system->partsJointsMotionsDo([&](std::shared_ptr<Item> item) { item->constraintsReport(); });
			system->partsJointsMotionsDo([&](std::shared_ptr<Item> item) { item->setqsu(qsuOld); });
		}
	}
}

void PosICNewtonRaphson::preRun()
{
	std::string str("MbD: Assembling system. ");
	system->logString(str);
	PosNewtonRaphson::preRun();
}

void PosICNewtonRaphson::assignEquationNumbers()
{
	auto parts = system->parts();
	//auto contactEndFrames = system->contactEndFrames();
	//auto uHolders = system->uHolders();
	auto essentialConstraints = system->essentialConstraints();
	auto displacementConstraints = system->displacementConstraints();
	auto perpendicularConstraints = system->perpendicularConstraints();
	size_t eqnNo = 0;
	for (auto& part : *parts) {
		part->iqX(eqnNo);
		eqnNo = eqnNo + 3;
		part->iqE(eqnNo);
		eqnNo = eqnNo + 4;
	}
	//for (auto& endFrm : *contactEndFrames) {
	//	endFrm->is(eqnNo);
	//	eqnNo = eqnNo + endFrm->sSize();
	//}
	//for (auto& uHolder : *uHolders) {
	//	uHolder->iu(eqnNo);
	//	eqnNo += 1;
	//}
	auto nEqns = eqnNo;	//C++ uses index 0.
	nqsu = nEqns;
	for (auto& con : *essentialConstraints) {
		con->iG = eqnNo;
		eqnNo += 1;
	}
	auto lastEssenConEqnNo = eqnNo - 1;
	for (auto& con : *displacementConstraints) {
		con->iG = eqnNo;
		eqnNo += 1;
	}
	auto lastDispConEqnNo = eqnNo - 1;
	for (auto& con : *perpendicularConstraints) {
		con->iG = eqnNo;
		eqnNo += 1;
	}
	auto lastEqnNo = eqnNo - 1;
	nEqns = eqnNo;	//C++ uses index 0.
	n = nEqns;
	auto rangelimits = { lastEssenConEqnNo + 1, lastDispConEqnNo + 1, lastEqnNo + 1 };
	pivotRowLimits = std::make_shared<std::vector<size_t>>(rangelimits);
}

bool PosICNewtonRaphson::isConverged()
{
	return this->isConvergedToNumericalLimit();
}

double PosICNewtonRaphson::constraintNorm() const
{
	double sum = 0.0;
	for (size_t i = nqsu; i < y->size(); i++) {
		sum += y->at(i) * y->at(i);
	}
	return sum;
}

void PosICNewtonRaphson::calcDXNormImproveRootCalcYNorm()
{
	// A Newton step is taken while it reduces the constraints' error |g|^2
	// (Armijo). The first that does not ends the search for the pose closest
	// to the start: from there the steps restore the constraints only, each
	// the step of least weighted movement onto the linearised constraints
	// (the same system with the multipliers at zero, which drops their
	// products with the constraints' second derivatives, and nothing pulling
	// back to the start), cut back by halves until it reduces |g|^2. The rows
	// of the Jacobian for g are its exact first derivatives, so either step is
	// a descent direction for |g|^2. The multipliers' terms are what made the
	// Newton steps of a solve started far from its joints wander off, metres
	// and further, until the redundancy search set aside equations that were
	// not redundant. A solve whose Newton steps all reduce |g|^2 is solved as
	// it was.
	const double gOld = constraintNorm();
	this->calcdxNorm();
	this->updatexold();
	auto reduces = [&](FColDsptr from, FColDsptr step, double alpha) {
		x = from->plusFullColumn(alpha == 1.0 ? step : step->times(alpha));
		this->passRootToSystem();
		this->askSystemToUpdate();
		this->fillY();
		return constraintNorm() <= (1.0 - 1.0e-4 * alpha) * gOld;
	};
	// The system with the multipliers at zero and nothing pulling back to
	// the start, at x: what the next solveEquations() takes for a restoring
	// step.
	auto restoringSystem = [&]() {
		for (size_t i = nqsu; i < x->size(); i++) {
			x->at(i) = 0.0;
		}
		this->passRootToSystem();
		this->askSystemToUpdate();
		this->fillY();
		for (size_t i = 0; i < nqsu; i++) {
			y->at(i) = 0.0;
		}
	};
	// Below 1e-18 (an error of 1e-9) a step is taken as it is: it is
	// converging there, or rounding.
	if (!restoring) {
		if (reduces(xold, dx, 1.0) || gOld <= 1.0e-18) {
			dxNorms->push_back(dxNorm);
			this->calcyNorm();
			yNorms->push_back(yNorm);
			yNormOld = yNorm;
			return;
		}
		restoring = true;
		x = xold->copy();
		restoringSystem();
		xold = x;
		this->fillPyPx();
		this->solveEquations();
	}
	// dx is a restoring step from xold, whose multipliers are zero.
	auto step = dx;
	double alpha = 1.0;
	while (!reduces(xold, step, alpha) && gOld > 1.0e-18 && alpha >= 1.0e-3) {
		alpha *= 0.5;
	}
	if (alpha != 1.0) {
		dx = step->times(alpha);
	}
	this->calcdxNorm();
	dxNorms->push_back(dxNorm);
	this->calcyNorm();
	yNorms->push_back(yNorm);
	yNormOld = yNorm;
	restoringSystem();
}

void PosICNewtonRaphson::handleSingularMatrix()
{
	nSingularMatrixError++;
	if (nSingularMatrixError == 1) {
		this->lookForRedundantConstraints();
		matrixSolver = this->matrixSolverClassNew();
	}
	else {
        auto& r = *matrixSolver;
		std::string str = typeid(r).name();
		if (SparseLUSolver::isFastSolver(r)) {
		    matrixSolver = CREATE<GESpMatParPvPrecise>::With();
		    this->solveEquations();
		}
		else {
            auto& msRef = *matrixSolver.get(); // extrapolated to suppress warning
            str = typeid(msRef).name();
            (void) msRef;                      // also for warning suppression
			if (str.find("GESpMatParPvPrecise") != std::string::npos) {
				this->lookForRedundantConstraints();
				matrixSolver = this->matrixSolverClassNew();
			} else {
				throw SimulationStoppingError("To be implemented.");
			}
		}
	}
}

void PosICNewtonRaphson::lookForRedundantConstraints()
{
	std::string str("MbD: Checking for redundant constraints.");
	system->logString(str);
	auto posICsolver = CREATE<GESpMatFullPvPosIC>::With();
	posICsolver->system = this;
	dx = posICsolver->solvewithsaveOriginal(pypx, y->negated(), false);
}
