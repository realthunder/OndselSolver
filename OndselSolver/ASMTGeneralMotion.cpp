/***************************************************************************
 *   Copyright (c) 2023 Ondsel, Inc.                                       *
 *                                                                         *
 *   This file is part of OndselSolver.                                    *
 *                                                                         *
 *   See LICENSE file for details about copyright.                         *
 ***************************************************************************/
#include <fstream>	

#include "ASMTGeneralMotion.h"
#include "ASMTAssembly.h"
#include "SymbolicParser.h"
#include "BasicUserFunction.h"
#include "CREATE.h"
#include "Constant.h"
#include "EulerAngles.h"
#include "FullMotion.h"
#include "ASMTTime.h"

using namespace MbD;

std::shared_ptr<ASMTGeneralMotion> MbD::ASMTGeneralMotion::With()
{
	auto asmt = std::make_shared<ASMTGeneralMotion>();
	asmt->initialize();
	return asmt;
}

void MbD::ASMTGeneralMotion::parseASMT(std::vector<std::string>& lines)
{
	readName(lines);
	readMarkerI(lines);
	readMarkerJ(lines);
	readrIJI(lines);
	readangIJJ(lines);
	readRotationOrder(lines);
}

void MbD::ASMTGeneralMotion::readrIJI(std::vector<std::string>& lines)
{
	rIJI = std::make_shared<FullColumn<std::string>>(3);

	assert(lines[0].find("rIJI1") != std::string::npos);
	lines.erase(lines.begin());
	rIJI->at(0) = readString(lines[0]);
	lines.erase(lines.begin());
	assert(lines[0].find("rIJI2") != std::string::npos);
	lines.erase(lines.begin());
	rIJI->at(1) = readString(lines[0]);
	lines.erase(lines.begin());
	assert(lines[0].find("rIJI3") != std::string::npos);
	lines.erase(lines.begin());
	rIJI->at(2) = readString(lines[0]);
	lines.erase(lines.begin());
}

void MbD::ASMTGeneralMotion::readangIJJ(std::vector<std::string>& lines)
{
	angIJJ = std::make_shared<FullColumn<std::string>>(3);

	assert(lines[0].find("angIJJ1") != std::string::npos);
	lines.erase(lines.begin());
	angIJJ->at(0) = readString(lines[0]);
	lines.erase(lines.begin());
	assert(lines[0].find("angIJJ2") != std::string::npos);
	lines.erase(lines.begin());
	angIJJ->at(1) = readString(lines[0]);
	lines.erase(lines.begin());
	assert(lines[0].find("angIJJ3") != std::string::npos);
	lines.erase(lines.begin());
	angIJJ->at(2) = readString(lines[0]);
	lines.erase(lines.begin());
}

void MbD::ASMTGeneralMotion::readRotationOrder(std::vector<std::string>& lines)
{
	assert(lines[0].find("RotationOrder") != std::string::npos);
	lines.erase(lines.begin());
	std::istringstream iss(lines[0]);
	rotationOrder = std::make_shared<std::vector<size_t>>();
	size_t i;
	while (iss >> i) {
		rotationOrder->push_back(i);
	}
	lines.erase(lines.begin());
}

std::shared_ptr<ItemIJ> MbD::ASMTGeneralMotion::mbdClassNew()
{
	return CREATE<FullMotion>::With();
}

void MbD::ASMTGeneralMotion::createMbD(std::shared_ptr<System> mbdSys, std::shared_ptr<Units> mbdUnits)
{
	ASMTMotion::createMbD(mbdSys, mbdUnits);
	auto parser = std::make_shared<SymbolicParser>();
	parser->owner = this;
	auto geoTime = owner->root()->geoTime();
	parser->variables->insert(std::make_pair("time", geoTime));
	// The text through SymbolicParser, or the host's function when one is set.
	auto geoFunction = [&](const std::string& text, const ExternalTimeFunction& function) -> Symsptr {
		if (function) {
			return std::make_shared<ExternalFunction>(geoTime, function, 0, text);
		}
		auto userFunc = std::make_shared<BasicUserFunction>(text, 1.0);
		parser->parseUserFunction(userFunc);
		return parser->stack->top();
	};
	auto fullMotion = std::static_pointer_cast<FullMotion>(mbdObject);

	//rIJI
	auto geoX = Symbolic::times(geoFunction(rIJI->at(0), rIJIFunctions[0]), sptrConstant(1.0 / mbdUnits->length));
	geoX->createMbD(mbdSys, mbdUnits);
	auto xBlk = geoX->simplified(geoX);

	auto geoY = Symbolic::times(geoFunction(rIJI->at(1), rIJIFunctions[1]), sptrConstant(1.0 / mbdUnits->length));
	geoY->createMbD(mbdSys, mbdUnits);
	auto yBlk = geoY->simplified(geoY);

	auto geoZ = Symbolic::times(geoFunction(rIJI->at(2), rIJIFunctions[2]), sptrConstant(1.0 / mbdUnits->length));
	geoZ->createMbD(mbdSys, mbdUnits);
	auto zBlk = geoZ->simplified(geoZ);

	auto xyzBlkList = std::initializer_list<Symsptr>{ xBlk, yBlk, zBlk };
	fullMotion->frIJI = std::make_shared<FullColumn<Symsptr>>(xyzBlkList);

	//angIJJ
	auto geoPhi = Symbolic::times(geoFunction(angIJJ->at(0), angIJJFunctions[0]), sptrConstant(1.0 / mbdUnits->angle));
	geoPhi->createMbD(mbdSys, mbdUnits);
	auto phiBlk = geoPhi->simplified(geoPhi);

	auto geoThe = Symbolic::times(geoFunction(angIJJ->at(1), angIJJFunctions[1]), sptrConstant(1.0 / mbdUnits->angle));
	geoThe->createMbD(mbdSys, mbdUnits);
	auto theBlk = geoThe->simplified(geoThe);

	auto geoPsi = Symbolic::times(geoFunction(angIJJ->at(2), angIJJFunctions[2]), sptrConstant(1.0 / mbdUnits->angle));
	geoPsi->createMbD(mbdSys, mbdUnits);
	auto psiBlk = geoPsi->simplified(geoPsi);

	auto xyzRotBlkList = std::initializer_list<Symsptr>{ phiBlk, theBlk, psiBlk };
	auto fangIJJ = std::make_shared<EulerAngles<Symsptr>>(xyzRotBlkList);
	fangIJJ->rotOrder = rotationOrder;
	fullMotion->fangIJJ = fangIJJ;
}

void MbD::ASMTGeneralMotion::setrIJI(size_t i, const std::string& text, ExternalTimeFunction function)
{
	rIJI->at(i) = text;
	rIJIFunctions.at(i) = std::move(function);
}

void MbD::ASMTGeneralMotion::setangIJJ(size_t i, const std::string& text, ExternalTimeFunction function)
{
	angIJJ->at(i) = text;
	angIJJFunctions.at(i) = std::move(function);
}

void MbD::ASMTGeneralMotion::storeOnLevel(std::ofstream& os, size_t level)
{
	storeOnLevelString(os, level, "GeneralMotion");
	ASMTItemIJ::storeOnLevel(os, level);
}

void MbD::ASMTGeneralMotion::storeOnTimeSeries(std::ofstream& os)
{
	os << "GeneralMotionSeries\t" << fullName("") << std::endl;
	ASMTItemIJ::storeOnTimeSeries(os);
}
