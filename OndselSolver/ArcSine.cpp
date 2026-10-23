/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/

#include <cmath>

#include "ArcSine.h"
#include "Constant.h"
#include "Product.h"
#include "Reciprocal.h"
#include "Sqrt.h"
#include "Sum.h"

using namespace MbD;

MbD::ArcSine::ArcSine(Symsptr arg) : FunctionX(arg)
{
}

double MbD::ArcSine::getValue()
{
	return std::asin(xx->getValue());
}

Symsptr MbD::ArcSine::differentiateWRTx()
{
	// d asin(x) / dx = 1 / sqrt(1 - x*x), the square as a product so that
	// no Power of a negative base appears. Infinite at x = +-1, NaN beyond.
	auto minusSquare = std::make_shared<Product>(sptrConstant(-1.0), xx, xx);
	auto oneMinusSquare = std::make_shared<Sum>(sptrConstant(1.0), minusSquare);
	return std::make_shared<Reciprocal>(std::make_shared<Sqrt>(oneMinusSquare));
}

Symsptr MbD::ArcSine::copyWith(Symsptr arg)
{
	return std::make_shared<ArcSine>(arg);
}

std::ostream& MbD::ArcSine::printOn(std::ostream& s) const
{
	s << "arcsin(" << *xx << ")";
	return s;
}
