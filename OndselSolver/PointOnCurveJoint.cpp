/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include "PointOnCurveJoint.h"
#include "System.h"

using namespace MbD;

MbD::PointOnCurveJoint::PointOnCurveJoint()
{
}

MbD::PointOnCurveJoint::PointOnCurveJoint(const std::string& str) : Joint(str)
{
}

void MbD::PointOnCurveJoint::initializeGlobally()
{
	if (constraints->empty())
	{
		addConstraint(PointOnCurveConstraintIJ::With(frmI, frmJ, 0, function));
		addConstraint(PointOnCurveConstraintIJ::With(frmI, frmJ, 1, function));
		this->root()->hasChanged = true;
	}
	else {
		Joint::initializeGlobally();
	}
}
