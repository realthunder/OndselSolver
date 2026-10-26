/***************************************************************************
 *   Copyright (c) 2026 Zheng, Lei <realthunder.dev@gmail.com>             *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#pragma once

#include <memory>

#include "MatrixSolver.h"

namespace MbD {
    // The Newton steps' linear solve as a sparse LU: Eigen's SparseLU with a
    // fill reducing column order (COLAMD), the rows conditioned as
    // GESpMatParPvMarkoFast conditions them, then rows and columns
    // equilibrated. The order depends on the matrix's pattern alone and the
    // factorization runs in a fixed order, so the same matrix gives the same
    // answer bit for bit on every run.
    //
    // A pivot below the tolerance, or a non-finite answer, hands the solve to
    // GESpMatParPvMarkoFast, which solves it or throws SingularMatrixError as
    // before; the callers then fall back as they did for it:
    // GESpMatParPvPrecise, and the full pivoting that names the redundant
    // equations.
    class SparseLUSolver : public MatrixSolver
    {
    public:
        SparseLUSolver();
        ~SparseLUSolver() override;

        FColDsptr basicSolvewithsaveOriginal(FMatDsptr fullMat, FColDsptr fullCol, bool saveOriginal) override;
        FColDsptr basicSolvewithsaveOriginal(SpMatDsptr spMat, FColDsptr fullCol, bool saveOriginal) override;
        void preSolvewithsaveOriginal(FMatDsptr fullMat, FColDsptr fullCol, bool saveOriginal) override;
        void preSolvewithsaveOriginal(SpMatDsptr spMat, FColDsptr fullCol, bool saveOriginal) override;
        void doPivoting(size_t p) override;
        void forwardEliminateWithPivot(size_t p) override;
        void backSubstituteIntoDU() override;
        void postSolve() override;
        double getmatrixArowimaxMagnitude(size_t i) override;

        // The solvers that fall back from a singular matrix as from
        // GESpMatParPvMarkoFast's: this one and that.
        static bool isFastSolver(const MatrixSolver& solver);

        // Smallest pivot of the equilibrated matrix the LU answers for; the
        // smallest of a 100 part chain's were 1e-9.
        static constexpr double pivotTolerance = 1.0e-12;

    private:
        void equilibrate();
        FColDsptr eliminate(SpMatDsptr spMat, FColDsptr fullCol, bool saveOriginal);

        struct Factor;
        std::unique_ptr<Factor> factor;
    };
}
