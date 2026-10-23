/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#include "Abs.h"
#include "Sign.h"

using namespace MbD;

MbD::Abs::Abs(Symsptr arg) : FunctionX(arg)
{
	//Do nothing.
}

double MbD::Abs::getValue()
{
    return std::abs(xx->getValue());
}

Symsptr MbD::Abs::differentiateWRTx()
{
	// d abs(x) / dx = sign(x). At x = 0 that is 0, the mean of the two
	// one-sided slopes; x/abs(x) would be NaN there, and a NaN factor is
	// dropped by Product::getValue.
	return std::make_shared<Sign>(xx);
}

Symsptr MbD::Abs::copyWith(Symsptr arg)
{
	return std::make_shared<Abs>(arg);
}

std::ostream& MbD::Abs::printOn(std::ostream& s) const
{
	s << "abs(" << *xx << ")";
	return s;
}
