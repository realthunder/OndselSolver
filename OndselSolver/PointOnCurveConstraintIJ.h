/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include <array>
#include <functional>

#include "ConstraintIJ.h"
#include "DispCompIeqcJeqcIe.h"

namespace MbD {
	// What a host's point-on-curve function gives at a point: for each of the
	// two equations, its value, its gradient and its Hessian with respect to
	// the point -- frame J's origin in frame I's coordinates, in the system's
	// length unit.
	struct PointOnCurveValues
	{
		std::array<double, 2> value {};
		std::array<std::array<double, 3>, 2> gradient {};
		std::array<std::array<std::array<double, 3>, 3>, 2> hessian {};
	};

	// The host evaluates the curve: OndselSolver has no curve geometry of its
	// own. The function may keep state between calls (the host tracks the
	// curve parameter of the closest point from one call to the next).
	using PointOnCurveFunction =
		std::function<void(const std::array<double, 3>& point, PointOnCurveValues& values)>;

	// One equation of a point of frame J held on a curve fixed in frame I:
	// G = f_row(x), x = rIeJeIe, the displacement of J's origin from I's in
	// I's axes (three DispCompIeqcJeqcIe), f the host's function. The
	// partial derivatives follow by the chain rule through x:
	//   pG/pq = f' px/pq,  ppG/pqpq = f' ppx/pqpq + px/pq' f'' px/pq.
	// Frames I and J both on parts (EndFrameqc), as DistancexyConstraintIJ.
	class PointOnCurveConstraintIJ : public ConstraintIJ
	{
		//rIeJeIe row function pGpXI pGpEI pGpXJ pGpEJ ppG... iqXI iqEI iqXJ iqEJ
	public:
		PointOnCurveConstraintIJ(EndFrmsptr frmi, EndFrmsptr frmj, size_t row, PointOnCurveFunction function);

		static std::shared_ptr<PointOnCurveConstraintIJ> With(EndFrmsptr frmi, EndFrmsptr frmj, size_t row, PointOnCurveFunction function);

		void addToJointForceI(FColDsptr col) override;
		void addToJointTorqueI(FColDsptr col) override;
		void calcPostDynCorrectorIteration() override;
		void fillAccICIterError(FColDsptr col) override;
		void fillPosICError(FColDsptr col) override;
		void fillPosICJacob(SpMatDsptr mat) override;
		void fillPosKineJacob(SpMatDsptr mat) override;
		void fillVelICJacob(SpMatDsptr mat) override;
		void initializeGlobally() override;
		void initializeLocally() override;
		void postInput() override;
		void postPosICIteration() override;
		void preAccIC() override;
		void prePosIC() override;
		void preVelIC() override;
		void simUpdateAll() override;
		void useEquationNumbers() override;
		ConstraintType type() override;
		std::string constraintSpec() override;

		std::array<std::shared_ptr<DispCompIeqcJeqcIe>, 3> rIeJeIe;
		size_t row = 0;
		PointOnCurveFunction function;

		FRowDsptr pGpXI, pGpEI, pGpXJ, pGpEJ;
		FMatDsptr ppGpXIpXI, ppGpXIpEI, ppGpEIpEI, ppGpXIpXJ, ppGpEIpXJ, ppGpXJpXJ;
		FMatDsptr ppGpXIpEJ, ppGpEIpEJ, ppGpXJpEJ, ppGpEJpEJ;
		size_t iqXI = SIZE_MAX, iqEI = SIZE_MAX, iqXJ = SIZE_MAX, iqEJ = SIZE_MAX;
	};
}
