/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#include <algorithm>
#include <cmath>
#include <iostream>	
#include <memory>
#include <typeinfo>
#include <assert.h>

#include "PrescribedMotion.h"
#include "EndFrameqct.h"
#include "Constant.h"
#include "System.h"

using namespace MbD;

PrescribedMotion::PrescribedMotion() {

}

PrescribedMotion::PrescribedMotion(const std::string& str) : Joint(str) {

}

void PrescribedMotion::initialize()
{
	Joint::initialize();
	xBlk = std::make_shared<Constant>(0.0);
	yBlk = std::make_shared<Constant>(0.0);
	zBlk = std::make_shared<Constant>(0.0);
	phiBlk = std::make_shared<Constant>(0.0);
	theBlk = std::make_shared<Constant>(0.0);
	psiBlk = std::make_shared<Constant>(0.0);
}

void MbD::PrescribedMotion::initMotions()
{
	auto xyzBlkList = std::initializer_list<Symsptr>{ xBlk, yBlk, zBlk };
	std::static_pointer_cast<EndFrameqct>(frmI)->rmemBlks = (std::make_shared<FullColumn<Symsptr>>(xyzBlkList));
	auto xyzRotBlkList = std::initializer_list<Symsptr>{ phiBlk, theBlk, psiBlk };
	std::static_pointer_cast<EndFrameqct>(frmI)->phiThePsiBlks = (std::make_shared<FullColumn<Symsptr>>(xyzRotBlkList));
}

void PrescribedMotion::connectsItoJ(EndFrmsptr frmi, EndFrmsptr frmj)
{
	Joint::connectsItoJ(frmi, frmj);
	std::static_pointer_cast<EndFrameqc>(frmI)->initEndFrameqct();
}

double MbD::PrescribedMotion::suggestSmallerOrAcceptDynFirstStepSize(double hnew)
{
	return suggestSmallerOrAcceptTurnStepSize(hnew);
}

double MbD::PrescribedMotion::suggestSmallerOrAcceptDynStepSize(double hnew)
{
	return suggestSmallerOrAcceptTurnStepSize(hnew);
}

double MbD::PrescribedMotion::suggestSmallerOrAcceptTurnStepSize(double hnew)
{
	// The step the integrator proposes (to the next output time in a
	// kinematic run), split into equal parts so that frame I turns about z by
	// at most maxTurnPerStep in each: its turn read at t + h with the system's
	// time set there for the reading and put back. A turn that is not about z
	// alone, or not defined at t + h, leaves the step as it is.
	auto frm = std::dynamic_pointer_cast<EndFrameqct>(frmI);
	if (!frm || !frm->phiThePsiBlks || hnew == 0.0) {
		return hnew;
	}
	auto sys = root();
	auto t = sys->mbdTimeValue();
	auto turn = frm->turnAboutZ();
	if (std::isnan(turn)) {
		return hnew;
	}
	auto h = hnew;
	for (int n = 1, tries = 0; tries < 8; tries++) {
		sys->mbdTimeValue(t + h);
		auto moved = std::abs(frm->turnAboutZ() - turn);
		if (!(moved > maxTurnPerStep)) {
			break;
		}
		n = std::max(n + 1, static_cast<int>(std::ceil(n * moved / maxTurnPerStep)));
		h = hnew / n;
	}
	sys->mbdTimeValue(t);
	return h;
}
