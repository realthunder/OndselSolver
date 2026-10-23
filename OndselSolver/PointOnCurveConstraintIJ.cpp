/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include "PointOnCurveConstraintIJ.h"
#include "CREATE.h"
#include "EndFrameqc.h"
#include "SimulationStoppingError.h"

using namespace MbD;

namespace {
	using Rows3 = std::array<FRowDsptr, 3>;
	using Mats3 = std::array<FMatDsptr, 3>;
	using Vec3 = std::array<double, 3>;
	using Mat3 = std::array<Vec3, 3>;

	// sum_m g[m] a_m: a first partial through the three components.
	FRowDsptr firstPartial(size_t n, const Vec3& g, const Rows3& a)
	{
		auto out = std::make_shared<FullRow<double>>(n, 0.0);
		for (size_t m = 0; m < 3; m++) {
			for (size_t i = 0; i < n; i++) {
				out->at(i) += g[m] * a[m]->at(i);
			}
		}
		return out;
	}

	// sum_m g[m] pp_m + sum_mn H[m][n] a_m' b_n: a second partial through the
	// three components; pp null where the components' own second partial is
	// zero.
	FMatDsptr secondPartial(size_t rows, size_t cols, const Vec3& g, const Mat3& H, const Mats3* pp,
		const Rows3& a, const Rows3& b)
	{
		auto out = std::make_shared<FullMatrix<double>>(rows, cols);
		for (size_t i = 0; i < rows; i++) {
			auto& row = out->at(i);
			for (size_t j = 0; j < cols; j++) {
				double s = 0.0;
				for (size_t m = 0; m < 3; m++) {
					if (pp) {
						s += g[m] * (*pp)[m]->at(i)->at(j);
					}
					for (size_t n = 0; n < 3; n++) {
						s += H[m][n] * a[m]->at(i) * b[n]->at(j);
					}
				}
				row->at(j) = s;
			}
		}
		return out;
	}
}

MbD::PointOnCurveConstraintIJ::PointOnCurveConstraintIJ(EndFrmsptr frmi, EndFrmsptr frmj, size_t row, PointOnCurveFunction function)
	: ConstraintIJ(frmi, frmj), row(row), function(std::move(function))
{
}

std::shared_ptr<PointOnCurveConstraintIJ> MbD::PointOnCurveConstraintIJ::With(EndFrmsptr frmi, EndFrmsptr frmj, size_t row, PointOnCurveFunction function)
{
	assert(frmi->isEndFrameqc());
	assert(frmj->isEndFrameqc());
	auto con = std::make_shared<PointOnCurveConstraintIJ>(frmi, frmj, row, std::move(function));
	for (size_t axis = 0; axis < 3; axis++) {
		con->rIeJeIe[axis] = CREATE<DispCompIeqcJeqcIe>::With(frmi, frmj, axis);
	}
	return con;
}

void MbD::PointOnCurveConstraintIJ::calcPostDynCorrectorIteration()
{
	const std::array<double, 3> x {rIeJeIe[0]->value(), rIeJeIe[1]->value(), rIeJeIe[2]->value()};
	PointOnCurveValues values;
	if (!function) {
		throw SimulationStoppingError("A point-on-curve constraint has no curve function.");
	}
	function(x, values);
	aG = values.value[row];
	const Vec3& g = values.gradient[row];
	const Mat3& H = values.hessian[row];

	Rows3 pXI, pEI, pXJ, pEJ;
	Mats3 ppXIEI, ppEIEI, ppEIXJ, ppEIEJ, ppEJEJ;
	for (size_t m = 0; m < 3; m++) {
		const auto& c = rIeJeIe[m];
		pXI[m] = c->pvaluepXI();
		pEI[m] = c->pvaluepEI();
		pXJ[m] = c->pvaluepXJ();
		pEJ[m] = c->pvaluepEJ();
		ppXIEI[m] = c->ppvaluepXIpEI();
		ppEIEI[m] = c->ppvaluepEIpEI();
		ppEIXJ[m] = c->ppvaluepEIpXJ();
		ppEIEJ[m] = c->ppvaluepEIpEJ();
		ppEJEJ[m] = c->ppvaluepEJpEJ();
	}
	pGpXI = firstPartial(3, g, pXI);
	pGpEI = firstPartial(4, g, pEI);
	pGpXJ = firstPartial(3, g, pXJ);
	pGpEJ = firstPartial(4, g, pEJ);
	// A displacement component is linear in XI and XJ, and its XI and XJ
	// partials do not depend on EJ: those second partials of x are zero
	// (DistancexyConstraintIqcJqc's comments).
	ppGpXIpXI = secondPartial(3, 3, g, H, nullptr, pXI, pXI);
	ppGpXIpEI = secondPartial(3, 4, g, H, &ppXIEI, pXI, pEI);
	ppGpEIpEI = secondPartial(4, 4, g, H, &ppEIEI, pEI, pEI);
	ppGpXIpXJ = secondPartial(3, 3, g, H, nullptr, pXI, pXJ);
	ppGpEIpXJ = secondPartial(4, 3, g, H, &ppEIXJ, pEI, pXJ);
	ppGpXJpXJ = secondPartial(3, 3, g, H, nullptr, pXJ, pXJ);
	ppGpXIpEJ = secondPartial(3, 4, g, H, nullptr, pXI, pEJ);
	ppGpEIpEJ = secondPartial(4, 4, g, H, &ppEIEJ, pEI, pEJ);
	ppGpXJpEJ = secondPartial(3, 4, g, H, nullptr, pXJ, pEJ);
	ppGpEJpEJ = secondPartial(4, 4, g, H, &ppEJEJ, pEJ, pEJ);
}

void MbD::PointOnCurveConstraintIJ::addToJointForceI(FColDsptr col)
{
	col->equalSelfPlusFullVectortimes(pGpXI, lam);
}

void MbD::PointOnCurveConstraintIJ::addToJointTorqueI(FColDsptr jointTorque)
{
	// As DistancexyConstraintIqcJc's.
	auto cForceT = pGpXI->times(lam);
	auto frmIeqc = std::static_pointer_cast<EndFrameqc>(frmI);
	auto rIpIeIp = frmIeqc->rpep();
	auto pAOIppEI = frmIeqc->pAOppE();
	auto aBOIp = frmIeqc->aBOp();
	auto fpAOIppEIrIpIeIp = std::make_shared<FullColumn<double>>(4, 0.0);
	for (size_t i = 0; i < 4; i++)
	{
		auto dum = cForceT->timesFullColumn(pAOIppEI->at(i)->timesFullColumn(rIpIeIp));
		fpAOIppEIrIpIeIp->atiput(i, dum);
	}
	auto lampGpE = pGpEI->transpose()->times(lam);
	auto c2Torque = aBOIp->timesFullColumn(lampGpE->minusFullColumn(fpAOIppEIrIpIeIp));
	jointTorque->equalSelfPlusFullColumntimes(c2Torque, 0.5);
}

void MbD::PointOnCurveConstraintIJ::fillAccICIterError(FColDsptr col)
{
	col->atiplusFullVectortimes(iqXI, pGpXI, lam);
	col->atiplusFullVectortimes(iqEI, pGpEI, lam);
	col->atiplusFullVectortimes(iqXJ, pGpXJ, lam);
	col->atiplusFullVectortimes(iqEJ, pGpEJ, lam);
	auto frmIeqc = std::static_pointer_cast<EndFrameqc>(frmI);
	auto frmJeqc = std::static_pointer_cast<EndFrameqc>(frmJ);
	auto qXdotI = frmIeqc->qXdot();
	auto qEdotI = frmIeqc->qEdot();
	auto qXdotJ = frmJeqc->qXdot();
	auto qEdotJ = frmJeqc->qEdot();
	double sum = pGpXI->timesFullColumn(frmIeqc->qXddot());
	sum += pGpEI->timesFullColumn(frmIeqc->qEddot());
	sum += pGpXJ->timesFullColumn(frmJeqc->qXddot());
	sum += pGpEJ->timesFullColumn(frmJeqc->qEddot());
	sum += qXdotI->transposeTimesFullColumn(ppGpXIpXI->timesFullColumn(qXdotI));
	sum += 2.0 * (qXdotI->transposeTimesFullColumn(ppGpXIpEI->timesFullColumn(qEdotI)));
	sum += qEdotI->transposeTimesFullColumn(ppGpEIpEI->timesFullColumn(qEdotI));
	sum += 2.0 * (qXdotI->transposeTimesFullColumn(ppGpXIpXJ->timesFullColumn(qXdotJ)));
	sum += 2.0 * (qEdotI->transposeTimesFullColumn(ppGpEIpXJ->timesFullColumn(qXdotJ)));
	sum += qXdotJ->transposeTimesFullColumn(ppGpXJpXJ->timesFullColumn(qXdotJ));
	sum += 2.0 * (qXdotI->transposeTimesFullColumn(ppGpXIpEJ->timesFullColumn(qEdotJ)));
	sum += 2.0 * (qEdotI->transposeTimesFullColumn(ppGpEIpEJ->timesFullColumn(qEdotJ)));
	sum += 2.0 * (qXdotJ->transposeTimesFullColumn(ppGpXJpEJ->timesFullColumn(qEdotJ)));
	sum += qEdotJ->transposeTimesFullColumn(ppGpEJpEJ->timesFullColumn(qEdotJ));
	col->atiplusNumber(iG, sum);
}

void MbD::PointOnCurveConstraintIJ::fillPosICError(FColDsptr col)
{
	Constraint::fillPosICError(col);
	col->atiplusFullVectortimes(iqXI, pGpXI, lam);
	col->atiplusFullVectortimes(iqEI, pGpEI, lam);
	col->atiplusFullVectortimes(iqXJ, pGpXJ, lam);
	col->atiplusFullVectortimes(iqEJ, pGpEJ, lam);
}

void MbD::PointOnCurveConstraintIJ::fillPosICJacob(SpMatDsptr mat)
{
	mat->atijplusFullRow(iG, iqXI, pGpXI);
	mat->atijplusFullColumn(iqXI, iG, pGpXI->transpose());
	mat->atijplusFullRow(iG, iqEI, pGpEI);
	mat->atijplusFullColumn(iqEI, iG, pGpEI->transpose());
	mat->atijplusFullRow(iG, iqXJ, pGpXJ);
	mat->atijplusFullColumn(iqXJ, iG, pGpXJ->transpose());
	mat->atijplusFullRow(iG, iqEJ, pGpEJ);
	mat->atijplusFullColumn(iqEJ, iG, pGpEJ->transpose());
	mat->atijplusFullMatrixtimes(iqXI, iqXI, ppGpXIpXI, lam);
	auto ppGpXIpEIlam = ppGpXIpEI->times(lam);
	mat->atijplusFullMatrix(iqXI, iqEI, ppGpXIpEIlam);
	mat->atijplusTransposeFullMatrix(iqEI, iqXI, ppGpXIpEIlam);
	mat->atijplusFullMatrixtimes(iqEI, iqEI, ppGpEIpEI, lam);
	auto ppGpXIpXJlam = ppGpXIpXJ->times(lam);
	mat->atijplusFullMatrix(iqXI, iqXJ, ppGpXIpXJlam);
	mat->atijplusTransposeFullMatrix(iqXJ, iqXI, ppGpXIpXJlam);
	auto ppGpEIpXJlam = ppGpEIpXJ->times(lam);
	mat->atijplusFullMatrix(iqEI, iqXJ, ppGpEIpXJlam);
	mat->atijplusTransposeFullMatrix(iqXJ, iqEI, ppGpEIpXJlam);
	mat->atijplusFullMatrixtimes(iqXJ, iqXJ, ppGpXJpXJ, lam);
	auto ppGpXIpEJlam = ppGpXIpEJ->times(lam);
	mat->atijplusFullMatrix(iqXI, iqEJ, ppGpXIpEJlam);
	mat->atijplusTransposeFullMatrix(iqEJ, iqXI, ppGpXIpEJlam);
	auto ppGpEIpEJlam = ppGpEIpEJ->times(lam);
	mat->atijplusFullMatrix(iqEI, iqEJ, ppGpEIpEJlam);
	mat->atijplusTransposeFullMatrix(iqEJ, iqEI, ppGpEIpEJlam);
	auto ppGpXJpEJlam = ppGpXJpEJ->times(lam);
	mat->atijplusFullMatrix(iqXJ, iqEJ, ppGpXJpEJlam);
	mat->atijplusTransposeFullMatrix(iqEJ, iqXJ, ppGpXJpEJlam);
	mat->atijplusFullMatrixtimes(iqEJ, iqEJ, ppGpEJpEJ, lam);
}

void MbD::PointOnCurveConstraintIJ::fillPosKineJacob(SpMatDsptr mat)
{
	mat->atijplusFullRow(iG, iqXI, pGpXI);
	mat->atijplusFullRow(iG, iqEI, pGpEI);
	mat->atijplusFullRow(iG, iqXJ, pGpXJ);
	mat->atijplusFullRow(iG, iqEJ, pGpEJ);
}

void MbD::PointOnCurveConstraintIJ::fillVelICJacob(SpMatDsptr mat)
{
	mat->atijplusFullRow(iG, iqXI, pGpXI);
	mat->atijplusFullColumn(iqXI, iG, pGpXI->transpose());
	mat->atijplusFullRow(iG, iqEI, pGpEI);
	mat->atijplusFullColumn(iqEI, iG, pGpEI->transpose());
	mat->atijplusFullRow(iG, iqXJ, pGpXJ);
	mat->atijplusFullColumn(iqXJ, iG, pGpXJ->transpose());
	mat->atijplusFullRow(iG, iqEJ, pGpEJ);
	mat->atijplusFullColumn(iqEJ, iG, pGpEJ->transpose());
}

void MbD::PointOnCurveConstraintIJ::initializeGlobally()
{
	for (auto& c : rIeJeIe) {
		c->initializeGlobally();
	}
}

void MbD::PointOnCurveConstraintIJ::initializeLocally()
{
	for (auto& c : rIeJeIe) {
		c->initializeLocally();
	}
}

void MbD::PointOnCurveConstraintIJ::postInput()
{
	for (auto& c : rIeJeIe) {
		c->postInput();
	}
	ConstraintIJ::postInput();
}

void MbD::PointOnCurveConstraintIJ::postPosICIteration()
{
	for (auto& c : rIeJeIe) {
		c->postPosICIteration();
	}
	ConstraintIJ::postPosICIteration();
}

void MbD::PointOnCurveConstraintIJ::preAccIC()
{
	for (auto& c : rIeJeIe) {
		c->preAccIC();
	}
	ConstraintIJ::preAccIC();
}

void MbD::PointOnCurveConstraintIJ::prePosIC()
{
	for (auto& c : rIeJeIe) {
		c->prePosIC();
	}
	ConstraintIJ::prePosIC();
}

void MbD::PointOnCurveConstraintIJ::preVelIC()
{
	for (auto& c : rIeJeIe) {
		c->preVelIC();
	}
	ConstraintIJ::preVelIC();
}

void MbD::PointOnCurveConstraintIJ::simUpdateAll()
{
	for (auto& c : rIeJeIe) {
		c->simUpdateAll();
	}
	ConstraintIJ::simUpdateAll();
}

void MbD::PointOnCurveConstraintIJ::useEquationNumbers()
{
	auto frmIeqc = std::static_pointer_cast<EndFrameqc>(frmI);
	iqXI = frmIeqc->iqX();
	iqEI = frmIeqc->iqE();
	auto frmJeqc = std::static_pointer_cast<EndFrameqc>(frmJ);
	iqXJ = frmJeqc->iqX();
	iqEJ = frmJeqc->iqE();
}

ConstraintType MbD::PointOnCurveConstraintIJ::type()
{
	return displacement;
}

std::string MbD::PointOnCurveConstraintIJ::constraintSpec()
{
	return "PointOnCurveConstraintIJ";
}
