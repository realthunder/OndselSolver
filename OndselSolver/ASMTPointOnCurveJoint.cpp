/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <fstream>

#include "ASMTPointOnCurveJoint.h"
#include "CREATE.h"
#include "PointOnCurveJoint.h"
#include "SimulationStoppingError.h"
#include "Units.h"

using namespace MbD;

std::shared_ptr<ASMTPointOnCurveJoint> MbD::ASMTPointOnCurveJoint::With()
{
	auto asmt = std::make_shared<ASMTPointOnCurveJoint>();
	asmt->initialize();
	return asmt;
}

std::shared_ptr<ItemIJ> MbD::ASMTPointOnCurveJoint::mbdClassNew()
{
	return CREATE<PointOnCurveJoint>::With();
}

void MbD::ASMTPointOnCurveJoint::parseASMT(std::vector<std::string>& lines)
{
	ASMTJoint::parseASMT(lines);
	if (!lines.empty() && lines[0].find("curve") != std::string::npos) {
		lines.erase(lines.begin());
		curve = readString(lines[0]);
		lines.erase(lines.begin());
	}
}

void MbD::ASMTPointOnCurveJoint::createMbD(std::shared_ptr<System> mbdSys, std::shared_ptr<Units> mbdUnits)
{
	if (!function) {
		throw SimulationStoppingError("A PointOnCurveJoint needs its curve evaluated by the host.");
	}
	ASMTJoint::createMbD(mbdSys, mbdUnits);
	auto joint = std::static_pointer_cast<PointOnCurveJoint>(mbdObject);
	// The host works in the model's length unit; the solver in its own. With a
	// length unit L, x_model = L x, G = f(x_model) / L, so the gradient is the
	// host's and the Hessian L times it.
	const double length = mbdUnits ? mbdUnits->length : 1.0;
	if (length == 1.0) {
		joint->function = function;
		return;
	}
	auto host = function;
	joint->function = [host, length](const std::array<double, 3>& x, PointOnCurveValues& values) {
		host({x[0] * length, x[1] * length, x[2] * length}, values);
		for (size_t k = 0; k < 2; k++) {
			values.value[k] /= length;
			for (auto& row : values.hessian[k]) {
				for (auto& h : row) {
					h *= length;
				}
			}
		}
	};
}

void MbD::ASMTPointOnCurveJoint::storeOnLevel(std::ofstream& os, size_t level)
{
	ASMTJoint::storeOnLevel(os, level);
	storeOnLevelString(os, level + 1, "curve");
	storeOnLevelString(os, level + 2, curve);
}

void MbD::ASMTPointOnCurveJoint::setCurve(const std::string& text, PointOnCurveFunction curveFunction)
{
	curve = text;
	function = std::move(curveFunction);
}
