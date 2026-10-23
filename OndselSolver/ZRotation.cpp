/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#include "System.h"
#include "ZRotation.h"
#include "FullColumn.h"
#include "AngleZConstraintIqctJqc.h"
#include "EndFrameqc.h"
#include "EndFrameqct.h"
#include "CREATE.h"

using namespace MbD;

ZRotation::ZRotation() {

}

ZRotation::ZRotation(const std::string& str) : PrescribedMotion(str) {

}

void ZRotation::initializeGlobally()
{
	if (constraints->empty()) {
		initMotions();
		// The angle itself, with its turns counted, not the direction cosine
		// of I's y and J's x (its sine): the sine is zero half a turn away
		// too, and a command that moved more than a quarter turn since the
		// last output time took the body to that root.
		auto angleCon = AngleZConstraintIqctJqc::With(frmI, frmJ);
		addConstraint(angleCon);
		this->root()->hasChanged = true;
	}
	else {
		PrescribedMotion::initializeGlobally();
	}
}
