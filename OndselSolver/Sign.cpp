/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include "Sign.h"
#include "Constant.h"

using namespace MbD;

MbD::Sign::Sign(Symsptr arg) : FunctionX(arg)
{
}

double MbD::Sign::getValue()
{
    auto x = xx->getValue();
    return (x > 0.0) - (x < 0.0);
}

Symsptr MbD::Sign::differentiateWRTx()
{
    return sptrConstant(0.0);
}

Symsptr MbD::Sign::copyWith(Symsptr arg)
{
    return std::make_shared<Sign>(arg);
}

std::ostream& MbD::Sign::printOn(std::ostream& s) const
{
    s << "sign(" << *xx << ")";
    return s;
}
