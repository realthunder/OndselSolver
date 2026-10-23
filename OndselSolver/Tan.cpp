/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <cmath>

#include "Tan.h"
#include "Cosine.h"
#include "Reciprocal.h"

using namespace MbD;

MbD::Tan::Tan(Symsptr arg) : FunctionX(arg)
{
}

double MbD::Tan::getValue()
{
    return std::tan(xx->getValue());
}

Symsptr MbD::Tan::differentiateWRTx()
{
    // d tan(x) / dx = 1 / cos(x)^2, written as a product so that no Power of
    // a negative base appears.
    auto cosx = std::make_shared<Cosine>(xx);
    return std::make_shared<Reciprocal>(Symbolic::times(cosx, cosx));
}

Symsptr MbD::Tan::copyWith(Symsptr arg)
{
	return std::make_shared<Tan>(arg);
}

std::ostream& MbD::Tan::printOn(std::ostream& s) const
{
	s << "tan(" << *xx << ")";
	return s;
}
