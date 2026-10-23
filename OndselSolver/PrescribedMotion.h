/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
 
#pragma once
#include <memory>

#include "Joint.h"

namespace MbD {
    class Symbolic;
    using Symsptr = std::shared_ptr<Symbolic>;
    class EndFramec;

    class PrescribedMotion : public Joint
    {
        //xBlk yBlk zBlk phiBlk theBlk psiBlk 
    public:
        PrescribedMotion();
        PrescribedMotion(const std::string& str);

        void connectsItoJ(EndFrmsptr frmI, EndFrmsptr frmJ) override;
        void initialize() override;
        virtual void initMotions();
        double suggestSmallerOrAcceptDynFirstStepSize(double hnew) override;
        double suggestSmallerOrAcceptDynStepSize(double hnew) override;
        double suggestSmallerOrAcceptTurnStepSize(double hnew);

        // The most frame I may turn about z in one step of the integrator.
        // The angle equation (AngleZConstraintIqctJqc) has its one root at the
        // commanded angle however far it moved; what this bounds is the
        // Newton iteration that gets there from the last step's pose. Each
        // iteration turns a body by less than half a turn (a linearised
        // Euler-parameter step), so tens of radians took tens of iterations,
        // ran out of them, or met a singular matrix on the way.
        static constexpr double maxTurnPerStep = 0.7853981633974483;  // pi / 4

        Symsptr xBlk;
        Symsptr yBlk;
        Symsptr zBlk;
        Symsptr phiBlk;
        Symsptr theBlk;
        Symsptr psiBlk;
    };
}

