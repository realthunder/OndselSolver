/***************************************************************************
 *   Copyright (c) 2026 Zheng, Lei <realthunder.dev@gmail.com>             *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <typeinfo>
#include <vector>

#include <Eigen/OrderingMethods>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include "SparseLUSolver.h"
#include "SparseMatrix.h"
#include "FullColumn.h"
#include "SimulationStoppingError.h"
#include "GESpMatParPvMarkoFast.h"
#include "CREATE.h"

using namespace MbD;

namespace {
    // 64 bit indices: a host that links Chrono into the same binary gets its
    // Eigen::SparseMatrix with a vtable (Chrono's EIGEN_SPARSEMATRIX_PLUGIN
    // declares a virtual SetElement), and the linker keeps one copy of each
    // inline member. Chrono's are int indexed (its own matrices and the
    // column major copies its sparse solvers make), so an int indexed matrix
    // here ran Chrono's resize() on the wrong layout.
    using Index = std::int64_t;
    using SpMat = Eigen::SparseMatrix<double, Eigen::ColMajor, Index>;

    // SparseLU, and the diagonal of its U, which it keeps in the supernodes
    // of L (where absDeterminant() reads it).
    class PivotedLU : public Eigen::SparseLU<SpMat, Eigen::COLAMDOrdering<Index>>
    {
    public:
        double smallestPivot() const
        {
            double smallest = std::numeric_limits<double>::infinity();
            for (Eigen::Index j = 0; j < cols(); ++j) {
                double pivot = 0.0;
                for (SCMatrix::InnerIterator it(m_Lstore, j); it; ++it) {
                    if (it.index() == j) {
                        pivot = std::abs(it.value());
                        break;
                    }
                }
                smallest = std::min(smallest, pivot);
            }
            return smallest;
        }
    };
}

struct SparseLUSolver::Factor
{
    PivotedLU lu;
    SpMat matrix;
    std::vector<Eigen::Triplet<double, Index>> triplets;
    // The pattern the column order was found for: the next matrix with the
    // same pattern keeps it.
    std::vector<Index> outer, inner;
    bool analysed = false;
    // Equilibration: the matrix is diag(rowScale) A diag(colScale)
    std::vector<double> rowScale, colScale;
    std::shared_ptr<MatrixSolver> elimination;
};

SparseLUSolver::SparseLUSolver() : factor(std::make_unique<Factor>())
{
}

SparseLUSolver::~SparseLUSolver() = default;

bool SparseLUSolver::isFastSolver(const MatrixSolver& solver)
{
    std::string str = typeid(solver).name();
    return str.find("GESpMatParPvMarkoFast") != std::string::npos
        || str.find("SparseLUSolver") != std::string::npos;
}

void SparseLUSolver::preSolvewithsaveOriginal(SpMatDsptr spMat, FColDsptr fullCol, bool saveOriginal)
{
    // Each row scaled to a largest magnitude of 1, the smaller entries
    // dropped: GESpMatParPvMarkoFast's conditioning.
    m = spMat->nrow();
    n = spMat->ncol();
    if (m != n) {
        throwSingularMatrixError("");
    }
    // A copy either way: the elimination a small pivot hands the solve to
    // takes the column as it came.
    (void)saveOriginal;
    rightHandSideB = fullCol->copy();
    auto& f = *factor;
    f.triplets.clear();
    for (size_t i = 0; i < m; i++) {
        auto& spRowi = spMat->at(i);
        double maxRowMagnitude = spRowi->maxMagnitude();
        if (maxRowMagnitude == 0) {
            throwSingularMatrixError("");
        }
        auto scaling = 1.0 / maxRowMagnitude;
        for (auto const& kv : *spRowi) {
            auto val = kv.second * scaling;
            if (std::abs(val) >= singularPivotTolerance) {
                f.triplets.emplace_back((Index)i, (Index)kv.first, val);
            }
        }
        rightHandSideB->atitimes(i, scaling);
    }
    f.matrix.resize((Eigen::Index)m, (Eigen::Index)n);
    f.matrix.setFromTriplets(f.triplets.begin(), f.triplets.end());
    f.matrix.makeCompressed();
    equilibrate();
}

void SparseLUSolver::equilibrate()
{
    // Rows and columns scaled towards a largest magnitude of 1 (Ruiz). The
    // weights of the minimum movement rows (1e3 to 1e6) against constraint
    // rows of order 1 leave the multipliers' columns tiny after the row
    // scaling alone, and the pivots then fall with the size of the model
    // (1e-12 at 50 parts in a chain): no pivot tolerance could tell such a
    // matrix from a singular one. The factors are powers of 2, so scaling
    // rounds nothing.
    auto& f = *factor;
    auto& A = f.matrix;
    const Index* outer = A.outerIndexPtr();
    const Index* inner = A.innerIndexPtr();
    double* values = A.valuePtr();
    f.rowScale.assign(m, 1.0);
    f.colScale.assign(n, 1.0);
    std::vector<double> rowMax(m), colMax(n), r(m), c(n);
    auto halfPowerOf2 = [](double max) {
        // About 1 / sqrt(max), a power of 2
        if (max == 0.0) {
            return 1.0;
        }
        int e;
        std::frexp(max, &e);
        return std::ldexp(1.0, -e / 2);
    };
    for (int pass = 0; pass < 4; pass++) {
        std::fill(rowMax.begin(), rowMax.end(), 0.0);
        std::fill(colMax.begin(), colMax.end(), 0.0);
        for (size_t j = 0; j < n; j++) {
            for (Index k = outer[j]; k < outer[j + 1]; k++) {
                double a = std::abs(values[k]);
                rowMax[inner[k]] = std::max(rowMax[inner[k]], a);
                colMax[j] = std::max(colMax[j], a);
            }
        }
        for (size_t i = 0; i < m; i++) {
            r[i] = halfPowerOf2(rowMax[i]);
            f.rowScale[i] *= r[i];
        }
        for (size_t j = 0; j < n; j++) {
            c[j] = halfPowerOf2(colMax[j]);
            f.colScale[j] *= c[j];
            for (Index k = outer[j]; k < outer[j + 1]; k++) {
                values[k] *= r[inner[k]] * c[j];
            }
        }
    }
}

FColDsptr SparseLUSolver::basicSolvewithsaveOriginal(SpMatDsptr spMat, FColDsptr fullCol, bool saveOriginal)
{
    preSolvewithsaveOriginal(spMat, fullCol, saveOriginal);
    auto& f = *factor;
    auto& A = f.matrix;
    const Index* outer = A.outerIndexPtr();
    const Index* inner = A.innerIndexPtr();
    auto nnz = (size_t)A.nonZeros();
    bool samePattern = f.analysed && f.outer.size() == n + 1 && f.inner.size() == nnz
        && std::equal(f.outer.begin(), f.outer.end(), outer)
        && std::equal(f.inner.begin(), f.inner.end(), inner);
    if (!samePattern) {
        f.lu.analyzePattern(A);
        f.outer.assign(outer, outer + n + 1);
        f.inner.assign(inner, inner + nnz);
        f.analysed = true;
    }
    f.lu.factorize(A);
    if (f.lu.info() != Eigen::Success) {
        return eliminate(spMat, fullCol, saveOriginal);
    }
    if (!(f.lu.smallestPivot() >= pivotTolerance)) {
        return eliminate(spMat, fullCol, saveOriginal);
    }
    Eigen::VectorXd b((Eigen::Index)m);
    for (size_t i = 0; i < m; i++) {
        b[(Eigen::Index)i] = rightHandSideB->at(i) * f.rowScale[i];
    }
    Eigen::VectorXd x = f.lu.solve(b);
    if (!x.allFinite()) {
        return eliminate(spMat, fullCol, saveOriginal);
    }
    answerX = std::make_shared<FullColumn<double>>(n);
    for (size_t i = 0; i < n; i++) {
        answerX->at(i) = x[(Eigen::Index)i] * f.colScale[i];
    }
    return answerX;
}

FColDsptr SparseLUSolver::eliminate(SpMatDsptr spMat, FColDsptr fullCol, bool saveOriginal)
{
    // A matrix that may be singular: GESpMatParPvMarkoFast solves it, or
    // throws SingularMatrixError, as it did before this solver took its
    // place, so whether a matrix is singular is decided as before -- what
    // the redundant constraint removal and the records rest on.
    if (!factor->elimination) {
        factor->elimination = CREATE<GESpMatParPvMarkoFast>::With();
    }
    answerX = factor->elimination->solvewithsaveOriginal(spMat, fullCol, saveOriginal);
    return answerX;
}

FColDsptr SparseLUSolver::basicSolvewithsaveOriginal(FMatDsptr, FColDsptr, bool)
{
    throw SimulationStoppingError("To be implemented.");
}

void SparseLUSolver::preSolvewithsaveOriginal(FMatDsptr, FColDsptr, bool)
{
    throw SimulationStoppingError("To be implemented.");
}

void SparseLUSolver::doPivoting(size_t)
{
    throw SimulationStoppingError("To be implemented.");
}

void SparseLUSolver::forwardEliminateWithPivot(size_t)
{
    throw SimulationStoppingError("To be implemented.");
}

void SparseLUSolver::backSubstituteIntoDU()
{
    throw SimulationStoppingError("To be implemented.");
}

void SparseLUSolver::postSolve()
{
}

double SparseLUSolver::getmatrixArowimaxMagnitude(size_t i)
{
    // The rows are scaled to a largest magnitude of 1 (by column, as Eigen
    // stores them, but nothing calls this on the hot path)
    const auto& A = factor->matrix;
    double max = 0.0;
    for (Eigen::Index k = 0; k < A.nonZeros(); k++) {
        if ((size_t)A.innerIndexPtr()[k] == i) {
            max = std::max(max, std::abs(A.valuePtr()[k]));
        }
    }
    return max;
}
