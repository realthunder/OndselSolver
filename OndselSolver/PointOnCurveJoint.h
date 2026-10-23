/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include "Joint.h"
#include "PointOnCurveConstraintIJ.h"

namespace MbD {
	// Frame J's origin held on a curve fixed in frame I: two equations, both
	// from the host's function (PointOnCurveConstraintIJ). A kinematic cam.
	class PointOnCurveJoint : public Joint
	{
		//function
	public:
		PointOnCurveJoint();
		PointOnCurveJoint(const std::string& str);
		void initializeGlobally() override;

		PointOnCurveFunction function;
	};
}
