/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <utility>

#include "ExternalFunction.h"
#include "Constant.h"

using namespace MbD;

MbD::ExternalFunction::ExternalFunction(Symsptr time, ExternalTimeFunction func, int ord, std::string str)
    : FunctionX(time), function(std::move(func)), order(ord), name(std::move(str))
{
}

double MbD::ExternalFunction::getValue()
{
    return function(xx->getValue(), order);
}

Symsptr MbD::ExternalFunction::differentiateWRT(Symsptr var)
{
    if (this == var.get()) return sptrConstant(1.0);
    auto dxdvar = xx->differentiateWRT(var);
    dxdvar = dxdvar->simplified(dxdvar);
    if (dxdvar->isConstant()) {
        auto slope = dxdvar->getValue();
        if (slope == 0.0) return sptrConstant(0.0);
        if (slope == 1.0) return differentiateWRTx();
        return Symbolic::times(sptrConstant(slope), differentiateWRTx());
    }
    return Symbolic::times(differentiateWRTx(), dxdvar);
}

Symsptr MbD::ExternalFunction::differentiateWRTx()
{
    return std::make_shared<ExternalFunction>(xx, function, order + 1, name);
}

Symsptr MbD::ExternalFunction::copyWith(Symsptr arg)
{
    return std::make_shared<ExternalFunction>(arg, function, order, name);
}

Symsptr MbD::ExternalFunction::expandUntil(Symsptr sptr, std::shared_ptr<std::unordered_set<Symsptr>>)
{
    return sptr;
}

Symsptr MbD::ExternalFunction::simplifyUntil(Symsptr sptr, std::shared_ptr<std::unordered_set<Symsptr>>)
{
    return sptr;
}

Symsptr MbD::ExternalFunction::clonesptr()
{
    return std::make_shared<ExternalFunction>(*this);
}

bool MbD::ExternalFunction::isConstant()
{
    // Never folded into a number: the host's function is not known to be
    // constant even when xx is.
    return false;
}

std::ostream& MbD::ExternalFunction::printOn(std::ostream& s) const
{
    // external[36*time](t), its derivatives external[36*time]'(t), ''(t), ...
    s << "external[" << name << "]";
    for (int i = 0; i < order; i++) s << "'";
    s << "(" << *xx << ")";
    return s;
}
