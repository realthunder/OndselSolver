/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#include <limits>

#include "EndFrameqct2.h"
#include "MarkerFrame.h"
#include "System.h"
#include "Symbolic.h"
#include "SymTime.h"
#include "EulerParameters.h"
#include "CREATE.h"
#include "EulerAngleszxz.h"
#include "EulerAngleszxzDot.h"
#include "EulerAngleszxzDDot.h"

using namespace MbD;

EndFrameqct2::EndFrameqct2() {
}

EndFrameqct2::EndFrameqct2(const std::string& str) : EndFrameqct(str) {
}

void EndFrameqct2::initpPhiThePsiptBlks()
{
	auto& mbdTime = this->root()->time;
	auto eulerAngles = std::static_pointer_cast<EulerAngles<Symsptr>>(phiThePsiBlks);
	//pPhiThePsiptBlks = differentiateWRT(*eulerAngles, mbdTime);
	pPhiThePsiptBlks = eulerAngles->differentiateWRT(mbdTime);
}

void EndFrameqct2::initppPhiThePsiptptBlks()
{
	auto& mbdTime = this->root()->time;
	auto eulerAnglesDot = std::static_pointer_cast<EulerAnglesDot<Symsptr>>(pPhiThePsiptBlks);
	ppPhiThePsiptptBlks = eulerAnglesDot->differentiateWRT(mbdTime);
}

void EndFrameqct2::evalAme()
{
	auto eulerAngles = std::static_pointer_cast<EulerAngles<Symsptr>>(phiThePsiBlks);
	eulerAngles->calc();
	aAme = eulerAngles->aA;
}

void EndFrameqct2::evalpAmept()
{
	auto eulerAnglesDot = std::static_pointer_cast<EulerAnglesDot<Symsptr>>(pPhiThePsiptBlks);
	eulerAnglesDot->calc();
	pAmept = eulerAnglesDot->aAdot;

}

void EndFrameqct2::evalppAmeptpt()
{
	auto eulerAnglesDDot = std::static_pointer_cast<EulerAnglesDDot<Symsptr>>(ppPhiThePsiptptBlks);
	eulerAnglesDDot->calc();
	ppAmeptpt = eulerAnglesDDot->aAddot;
}

double MbD::EndFrameqct2::turnAboutZ()
{
	// aAme is the product of the turns in rotOrder (1, 2, 3 for x, y, z): a
	// turn about z alone when every angle about x or y is 0.
	if (!phiThePsiBlks) {
		return 0.0;
	}
	auto eulerAngles = std::static_pointer_cast<EulerAngles<Symsptr>>(phiThePsiBlks);
	double turn = 0.0;
	for (size_t i = 0; i < 3; i++)
	{
		auto angle = eulerAngles->at(i)->getValue();
		if (eulerAngles->rotOrder->at(i) == 3) {
			turn += angle;
		}
		else if (angle != 0.0) {
			return std::numeric_limits<double>::quiet_NaN();
		}
	}
	return turn;
}
