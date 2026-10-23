/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include <functional>
#include <string>

#include "FunctionX.h"

namespace MbD {
    // f(time, n): the n-th derivative with respect to time at that time, n = 0
    // the value. runKINEMATIC asks for n = 0, 1 and 2 (positions, velocities,
    // accelerations).
    using ExternalTimeFunction = std::function<double(double time, int order)>;

    // A function of time evaluated outside the symbolic engine: a host that
    // parses and differentiates its formulas itself hands them to a motion as
    // one of these (ASMTRotationalMotion::setRotationZ(text, function) and the
    // like) instead of text for SymbolicParser.
    //
    // xx is the time the function reads (a motion's geoTime). The node is
    // atomic: expansion and simplification leave it as it is, and the engine
    // differentiates it by asking the callback for one order more, times
    // d xx / d var by the chain rule -- the same node with order + 1 when var
    // is the time itself (or xx is that time in units of 1), 0 when xx does not
    // depend on var.
    class ExternalFunction : public FunctionX
    {
        //
    public:
        ExternalFunction() = default;
        ExternalFunction(Symsptr time, ExternalTimeFunction function, int order = 0, std::string name = "");
        double getValue() override;
        Symsptr differentiateWRT(Symsptr var) override;
        Symsptr differentiateWRTx() override;
        Symsptr copyWith(Symsptr arg) override;
        Symsptr expandUntil(Symsptr sptr, std::shared_ptr<std::unordered_set<Symsptr>> set) override;
        Symsptr simplifyUntil(Symsptr sptr, std::shared_ptr<std::unordered_set<Symsptr>> set) override;
        Symsptr clonesptr() override;
        bool isConstant() override;

        std::ostream& printOn(std::ostream& s) const override;

        ExternalTimeFunction function;
        int order = 0;
        std::string name;  // for printing: the formula the host evaluates
    };
}
