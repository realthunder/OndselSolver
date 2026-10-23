/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#include "FullMotion.h"
#include "AngleZConstraintIqctJqc.h"
#include "CREATE.h"
#include "System.h"

using namespace MbD;

MbD::FullMotion::FullMotion()
{
}

MbD::FullMotion::FullMotion(const std::string& str) : PrescribedMotion(str)
{
}

void MbD::FullMotion::connectsItoJ(EndFrmsptr frmi, EndFrmsptr frmj)
{
	Joint::connectsItoJ(frmi, frmj);
	std::static_pointer_cast<EndFrameqc>(frmI)->initEndFrameqct2();
}

void MbD::FullMotion::initializeGlobally()
{
	if (constraints->empty()) {
		initMotions();
		addConstraint(CREATE<TranslationConstraintIqcJqc>::ConstraintWith(frmI, frmJ, 0));
		addConstraint(CREATE<TranslationConstraintIqcJqc>::ConstraintWith(frmI, frmJ, 1));
		addConstraint(CREATE<TranslationConstraintIqcJqc>::ConstraintWith(frmI, frmJ, 2));
		// The turn about z as an angle with its turns counted, as ZRotation
		// holds it, not the direction cosine of I's y and J's x: with the two
		// below that cosine also held the half turn about z.
		addConstraint(AngleZConstraintIqctJqc::With(frmI, frmJ));
		addConstraint(CREATE<DirectionCosineConstraintIqcJqc>::ConstraintWith(frmI, frmJ, 2, 0));
		addConstraint(CREATE<DirectionCosineConstraintIqcJqc>::ConstraintWith(frmI, frmJ, 2, 1));
		this->root()->hasChanged = true;
	}
	else {
		PrescribedMotion::initializeGlobally();
	}
}

void MbD::FullMotion::initMotions()
{
	auto efrmI = std::static_pointer_cast<EndFrameqct>(frmI);
	efrmI->rmemBlks = frIJI;
	efrmI->phiThePsiBlks = fangIJJ;
}
