/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include <limits>

#include "AngleZIeqcJeqc.h"

namespace MbD {
    // The angle of J's x about I's z, from I's x, where frame I moves with
    // time: a motion's EndFrameqct, turned by the commanded angle. The value
    // is atan2 of the two direction cosines (I's x and y against J's x) with
    // its whole turns counted, as AngleZIecJec counts them across iterations.
    // Across time steps the count also follows frame I's own turn about z
    // (EndFrameqct::turnAboutZ): when the commanded angle moves by d, the
    // value the count is kept nearest to moves by -d. So the value is the
    // turn the bodies made since the start less the commanded angle, and
    // driving it to zero takes them to the commanded angle with its turns,
    // whatever the time step -- not to the nearest root of the sine, half a
    // turn off when the command moved more than a quarter turn.
    class AngleZIeqctJeqc : public AngleZIeqcJeqc
    {
        //turnI pthezpt ppthezpEIpt ppthezpEJpt ppthezptpt
    public:
        AngleZIeqctJeqc();
        AngleZIeqctJeqc(EndFrmsptr frmi, EndFrmsptr frmj);

        void calcPostDynCorrectorIteration() override;
        void init_aAijIeJe() override;
        void postInput() override;
        void preVelIC() override;
        void preAccIC() override;
        double pvaluept() override;
        FRowDsptr ppvaluepEIpt() override;
        FRowDsptr ppvaluepEJpt() override;
        double ppvalueptpt() override;

        double frameTurn();
        void calc_pthezpt();

        // Frame I's turn about z when the count last saw it; NaN when unknown.
        double turnI = std::numeric_limits<double>::quiet_NaN();
        double pthezpt = 0.0, ppthezptpt = 0.0;
        FRowDsptr ppthezpEIpt, ppthezpEJpt;
    };
}
