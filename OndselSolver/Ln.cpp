/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#include <cmath>

#include "Ln.h"
#include "Reciprocal.h"

using namespace MbD;

MbD::Ln::Ln(Symsptr arg) : FunctionX(arg)
{
}

double MbD::Ln::getValue()
{
    return std::log(xx->getValue());
}

Symsptr MbD::Ln::differentiateWRTx()
{
    return std::make_shared<Reciprocal>(xx);
}

Symsptr MbD::Ln::copyWith(Symsptr arg)
{
	return std::make_shared<Ln>(arg);
}

std::ostream& MbD::Ln::printOn(std::ostream& s) const
{
	s << "ln(" << *xx << ")";
	return s;
}
