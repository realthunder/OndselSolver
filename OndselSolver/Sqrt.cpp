/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <cmath>

#include "Sqrt.h"
#include "Constant.h"
#include "Reciprocal.h"

using namespace MbD;

MbD::Sqrt::Sqrt(Symsptr arg) : FunctionX(arg)
{
}

double MbD::Sqrt::getValue()
{
    return std::sqrt(xx->getValue());
}

Symsptr MbD::Sqrt::differentiateWRTx()
{
    // d sqrt(x) / dx = 0.5 / sqrt(x)
    return Symbolic::times(sptrConstant(0.5), std::make_shared<Reciprocal>(std::make_shared<Sqrt>(xx)));
}

Symsptr MbD::Sqrt::copyWith(Symsptr arg)
{
	return std::make_shared<Sqrt>(arg);
}

std::ostream& MbD::Sqrt::printOn(std::ostream& s) const
{
	s << "sqrt(" << *xx << ")";
	return s;
}
