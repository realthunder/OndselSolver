/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <cmath>

#include "AngleZIeqctJeqc.h"
#include "DirectionCosineIeqctJeqc.h"
#include "EndFrameqct.h"
#include "CREATE.h"

using namespace MbD;

MbD::AngleZIeqctJeqc::AngleZIeqctJeqc()
{
}

MbD::AngleZIeqctJeqc::AngleZIeqctJeqc(EndFrmsptr frmi, EndFrmsptr frmj) : AngleZIeqcJeqc(frmi, frmj)
{
	ppthezpEIpt = std::make_shared<FullRow<double>>(4);
	ppthezpEJpt = std::make_shared<FullRow<double>>(4);
}

void MbD::AngleZIeqctJeqc::init_aAijIeJe()
{
	aA00IeJe = CREATE<DirectionCosineIeqctJeqc>::With(frmI, frmJ, 0, 0);
	aA10IeJe = CREATE<DirectionCosineIeqctJeqc>::With(frmI, frmJ, 1, 0);
}

double MbD::AngleZIeqctJeqc::frameTurn()
{
	return std::static_pointer_cast<EndFrameqct>(frmI)->turnAboutZ();
}

void MbD::AngleZIeqctJeqc::calcPostDynCorrectorIteration()
{
	// Frame I is evaluated at the system's time before anything here runs
	// (the parts' end frames update first), so its turn now goes with the
	// direction cosines read below. Within one time it does not change.
	auto turn = frameTurn();
	if (thez != std::numeric_limits<double>::min() && !std::isnan(turn) && !std::isnan(turnI)) {
		thez -= turn - turnI;
	}
	turnI = turn;
	AngleZIeqcJeqc::calcPostDynCorrectorIteration();
}

void MbD::AngleZIeqctJeqc::postInput()
{
	aA00IeJe->postInput();
	aA10IeJe->postInput();
	// The count starts at the turn nearest the commanded angle: in (-pi, pi].
	thez = std::atan2(aA10IeJe->value(), aA00IeJe->value());
	turnI = frameTurn();
	KinematicIeJe::postInput();
}

void MbD::AngleZIeqctJeqc::calc_pthezpt()
{
	pthezpt = aA10IeJe->pvaluept() * cosOverSSq - (aA00IeJe->pvaluept() * sinOverSSq);
}

void MbD::AngleZIeqctJeqc::preVelIC()
{
	AngleZIeqcJeqc::preVelIC();
	calc_pthezpt();
}

void MbD::AngleZIeqctJeqc::preAccIC()
{
	// The second partials of atan2(s, c) as AngleZIeqcJec forms them for two
	// Euler parameters, with the time as one variable or both.
	AngleZIeqcJeqc::preAccIC();
	calc_pthezpt();
	auto pcthezpt = aA00IeJe->pvaluept();
	auto psthezpt = aA10IeJe->pvaluept();
	auto pcthezpEJ = aA00IeJe->pvaluepEJ();
	auto psthezpEJ = aA10IeJe->pvaluepEJ();
	auto ppcthezpEIpt = aA00IeJe->ppvaluepEIpt();
	auto ppsthezpEIpt = aA10IeJe->ppvaluepEIpt();
	auto ppcthezpEJpt = aA00IeJe->ppvaluepEJpt();
	auto ppsthezpEJpt = aA10IeJe->ppvaluepEJpt();
	auto pcthezpEIl = aA00IeJe->pvaluepEI();
	auto psthezpEIl = aA10IeJe->pvaluepEI();
	for (size_t i = 0; i < 4; i++)
	{
		auto pc = pcthezpEIl->at(i);
		auto ps = psthezpEIl->at(i);
		auto term1 = (pc * pcthezpt - (ps * psthezpt)) * twoCosSinOverSSqSq;
		auto term2 = ppsthezpEIpt->at(i) * cosOverSSq - (ppcthezpEIpt->at(i) * sinOverSSq);
		auto term3 = (ps * pcthezpt + (pc * psthezpt)) * dSqOverSSqSq;
		ppthezpEIpt->atiput(i, term1 + term2 + term3);
	}
	for (size_t i = 0; i < 4; i++)
	{
		auto pc = pcthezpEJ->at(i);
		auto ps = psthezpEJ->at(i);
		auto term1 = (pc * pcthezpt - (ps * psthezpt)) * twoCosSinOverSSqSq;
		auto term2 = ppsthezpEJpt->at(i) * cosOverSSq - (ppcthezpEJpt->at(i) * sinOverSSq);
		auto term3 = (ps * pcthezpt + (pc * psthezpt)) * dSqOverSSqSq;
		ppthezpEJpt->atiput(i, term1 + term2 + term3);
	}
	auto term1 = (pcthezpt * pcthezpt - (psthezpt * psthezpt)) * twoCosSinOverSSqSq;
	auto term2 = aA10IeJe->ppvalueptpt() * cosOverSSq - (aA00IeJe->ppvalueptpt() * sinOverSSq);
	auto term3 = 2.0 * psthezpt * pcthezpt * dSqOverSSqSq;
	ppthezptpt = term1 + term2 + term3;
}

double MbD::AngleZIeqctJeqc::pvaluept()
{
	return pthezpt;
}

FRowDsptr MbD::AngleZIeqctJeqc::ppvaluepEIpt()
{
	return ppthezpEIpt;
}

FRowDsptr MbD::AngleZIeqctJeqc::ppvaluepEJpt()
{
	return ppthezpEJpt;
}

double MbD::AngleZIeqctJeqc::ppvalueptpt()
{
	return ppthezptpt;
}
