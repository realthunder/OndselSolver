/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include "AngleZConstraintIqctJqc.h"
#include "AngleZIeqctJeqc.h"
#include "EndFrameqc.h"

using namespace MbD;

MbD::AngleZConstraintIqctJqc::AngleZConstraintIqctJqc(EndFrmsptr frmi, EndFrmsptr frmj) : AngleZConstraintIqcJqc(frmi, frmj)
{
}

std::shared_ptr<AngleZConstraintIqctJqc> MbD::AngleZConstraintIqctJqc::With(EndFrmsptr frmi, EndFrmsptr frmj)
{
	// frmi is an EndFrameqct, which does not answer isEndFrameqc().
	assert(frmj->isEndFrameqc());
	auto angleZConIJ = std::make_shared<AngleZConstraintIqctJqc>(frmi, frmj);
	angleZConIJ->initialize();
	return angleZConIJ;
}

void MbD::AngleZConstraintIqctJqc::initthezIeJe()
{
	thezIeJe = std::make_shared<AngleZIeqctJeqc>(frmI, frmJ);
}

void MbD::AngleZConstraintIqctJqc::preVelIC()
{
	AngleZConstraintIJ::preVelIC();
	pGpt = thezIeJe->pvaluept();
}

void MbD::AngleZConstraintIqctJqc::fillVelICError(FColDsptr col)
{
	col->atiminusNumber(iG, pGpt);
}

void MbD::AngleZConstraintIqctJqc::preAccIC()
{
	AngleZConstraintIJ::preAccIC();
	pGpt = thezIeJe->pvaluept();
	ppGpEIpt = thezIeJe->ppvaluepEIpt();
	ppGpEJpt = thezIeJe->ppvaluepEJpt();
	ppGptpt = thezIeJe->ppvalueptpt();
}

void MbD::AngleZConstraintIqctJqc::fillAccICIterError(FColDsptr col)
{
	AngleZConstraintIqcJqc::fillAccICIterError(col);
	auto efrmIqc = std::static_pointer_cast<EndFrameqc>(frmI);
	auto efrmJqc = std::static_pointer_cast<EndFrameqc>(frmJ);
	auto qEdotI = efrmIqc->qEdot();
	auto qEdotJ = efrmJqc->qEdot();
	double sum = (ppGpEIpt->timesFullColumn(qEdotI)) * 2.0;
	sum += (ppGpEJpt->timesFullColumn(qEdotJ)) * 2.0;
	sum += ppGptpt;
	col->atiplusNumber(iG, sum);
}

std::string MbD::AngleZConstraintIqctJqc::constraintSpec()
{
	return "AngleZConstraintIqctJqc";
}
