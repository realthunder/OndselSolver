/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include "AngleZConstraintIqcJqc.h"

namespace MbD {
	// A motion's angle about z: G = thez(I, J) - aConstant, aConstant 0, where
	// frame I is the motion's EndFrameqct, turned by the commanded angle, and
	// thez counts whole turns (AngleZIeqctJeqc). One equation, as the
	// direction cosine of I's y and J's x it replaces, with the same gradient
	// where it is satisfied; but that cosine, the sine of the angle, is zero
	// half a turn away too, and this has the one root.
	class AngleZConstraintIqctJqc : public AngleZConstraintIqcJqc
	{
		//pGpt ppGpEIpt ppGpEJpt ppGptpt
	public:
		AngleZConstraintIqctJqc(EndFrmsptr frmi, EndFrmsptr frmj);

		static std::shared_ptr<AngleZConstraintIqctJqc> With(EndFrmsptr frmi, EndFrmsptr frmj);

		void initthezIeJe() override;
		void fillAccICIterError(FColDsptr col) override;
		void fillVelICError(FColDsptr col) override;
		void preAccIC() override;
		void preVelIC() override;
		std::string constraintSpec() override;

		double pGpt = 0.0;
		FRowDsptr ppGpEIpt;
		FRowDsptr ppGpEJpt;
		double ppGptpt = 0.0;
	};
}
