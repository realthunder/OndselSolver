/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include "FunctionX.h"

namespace MbD {
    // sign(x): 1 for x > 0, -1 for x < 0, 0 for x = 0 (and for NaN). The
    // derivative of Abs. Its own derivative is 0 everywhere: the jump at 0 is
    // not seen, as a piecewise function's jumps are not.
    class Sign : public FunctionX
    {
        //
    public:
        Sign() = default;
        Sign(Symsptr arg);
        double getValue() override;
        Symsptr differentiateWRTx() override;
        Symsptr copyWith(Symsptr arg) override;

        std::ostream& printOn(std::ostream& s) const override;

    };
}
