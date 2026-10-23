/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include "ASMTJoint.h"
#include "PointOnCurveConstraintIJ.h"

namespace MbD {
	// A point of marker J's part held on a curve fixed in marker I (a
	// kinematic cam). The curve is the host's: OndselSolver evaluates it
	// through 'function', in the model's length unit, and the ASMT file
	// records only its description ('curve', one line of text the host
	// gives). A file read back has the description and no function, so it
	// loads but cannot be solved.
	class ASMTPointOnCurveJoint : public ASMTJoint
	{
		//curve function
	public:
		static std::shared_ptr<ASMTPointOnCurveJoint> With();
		std::shared_ptr<ItemIJ> mbdClassNew() override;
		void parseASMT(std::vector<std::string>& lines) override;
		void createMbD(std::shared_ptr<System> mbdSys, std::shared_ptr<Units> mbdUnits) override;
		void storeOnLevel(std::ofstream& os, size_t level) override;
		void setCurve(const std::string& text, PointOnCurveFunction curveFunction);

		std::string curve;
		PointOnCurveFunction function;
	};
}
