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

namespace {
	// Thrown by isConverged() when the steps grow far beyond the first one
	struct Wandering {};
}

void PosICNewtonRaphson::run()
{
	// Where the iteration wanders far from its start before it converges, if
	// it does, the outcome -- which equations are found redundant on the way,
	// which pose it ends in -- turns on the last bits of every step, and
	// SparseLUSolver's differ from those of the row scaled Gaussian
	// elimination this solver used before it. Such a run is solved once more
	// from its start that way, all constraints back, so that it ends as it
	// did before.
	canReplay = !legacyElimination && system->allRedundantConstraints()->empty();
	auto replayWithElimination = [&]() {
		if (!canReplay) {
			return false;
		}
		std::string str("MbD: Solving again with Gaussian elimination.");
		system->logString(str);
		canReplay = false;
		legacyElimination = true;
		nSingularMatrixError = 0;
		system->partsJointsMotionsDo([&](std::shared_ptr<Item> item) { item->reactivateRedundantConstraints(); });
		system->partsJointsMotionsDo([&](std::shared_ptr<Item> item) { item->setqsu(qsuOld); });
		return true;
	};
	while (true) {
		try {
			//VectorNewtonRaphson::run();   //Inline to help debugging
			preRun();
			initializeLocally();
			initializeGlobally();
			iterate();
			postRun();
			break;
		}
		catch (const Wandering&) {
			replayWithElimination();
		}
		catch (const SingularMatrixError& ex) {
			auto redundantEqnNos = ex.getRedundantEqnNos();
			auto nRedundant = system->allRedundantConstraints()->size();
			system->partsJointsMotionsDo([&](std::shared_ptr<Item> item) { item->removeRedundantConstraints(redundantEqnNos); });
			if (system->allRedundantConstraints()->size() == nRedundant) {
				// Nothing new set aside: the next round would solve the same
				// system from the same start and end here again, for ever.
				if (replayWithElimination()) {
					continue;
				}
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
	// A step a thousand times the first (and above 1) is no Newton iteration
	// converging: see run().
	if (canReplay && iterNo > 0) {
		auto dxNormIterNo = dxNorms->at(iterNo);
		if (dxNormIterNo > 1.0 && dxNormIterNo > 1.0e3 * dxNorms->at(0)) {
			throw Wandering();
		}
	}
	return this->isConvergedToNumericalLimit();
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
