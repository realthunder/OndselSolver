#include "pch.h"
#include <CADSystem.h>
#include <ASMTAssembly.h>
#include <GESpMatParPvPrecise.h>
#include <MomentOfInertiaSolver.h>
#include <CREATE.h>
#include <ASMTGearJoint.h>
#include <ASMTMarker.h>
#include <ASMTPart.h>
#include <ASMTPrincipalMassMarker.h>
#include <ASMTRevoluteJoint.h>
#include <ASMTRotationalMotion.h>
#include <ASMTSimulationParameters.h>
#include <cmath>

using namespace MbD;

TEST(OndselSolver, TestName) {
	EXPECT_EQ(1, 1);
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runOndselSinglePendulum) {
	//testing::internal::CaptureStdout();
	auto cadSystem = std::make_shared<CADSystem>();
	cadSystem->runOndselSinglePendulum();
	//std::string output = testing::internal::GetCapturedStdout();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runPreDragBackhoe1) {
	auto assembly = ASMTAssembly::assemblyFromFile(std::string(TEST_DATA_PATH) + "/runPreDragBackhoe1.asmt");
	assembly->runDraggingLog(std::string(TEST_DATA_PATH) + "/draggingBackhoe1.log");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runPreDragBackhoe2) {
	auto assembly = ASMTAssembly::assemblyFromFile(std::string(TEST_DATA_PATH) + "/runPreDragBackhoe2.asmt");
	assembly->runDraggingLog(std::string(TEST_DATA_PATH) + "/draggingBackhoe2.log");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runPreDragBackhoe3) {
	auto assembly = ASMTAssembly::assemblyFromFile(std::string(TEST_DATA_PATH) + "/runPreDragBackhoe3.asmt");
	assembly->runDraggingLog(std::string(TEST_DATA_PATH) + "/draggingBackhoe3.log");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, sliderrunDragStep) {
	auto assembly = ASMTAssembly::assemblyFromFile(std::string(TEST_DATA_PATH) + "/sliderrunPreDrag.asmt");
	assembly->runDraggingLog(std::string(TEST_DATA_PATH) + "/sliderdragging.log");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, pistonAllowZRotation) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/pistonAllowZRotation.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, slider_issue) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/slider_issue.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, RevRevJt) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/RevRevJt.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, RevCylJt) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/RevCylJt.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, CylSphJt) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/CylSphJt.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, SphSphJt) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/SphSphJt.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, Gears) {
	ASMTAssembly::readWriteFile(std::string(TEST_DATA_PATH) + "/Gears.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, anglejoint) {
	ASMTAssembly::readWriteFile(std::string(TEST_DATA_PATH) + "/anglejoint.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, constvel) {
	ASMTAssembly::readWriteFile(std::string(TEST_DATA_PATH) + "/constvel.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, rackscrew) {
	ASMTAssembly::readWriteFile(std::string(TEST_DATA_PATH) + "/rackscrew.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, planarbug) {
	ASMTAssembly::readWriteFile(std::string(TEST_DATA_PATH) + "/planarbug.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, cirpendu2) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/cirpendu2.asmt");	//Under constrained. Testing ICKine.
	EXPECT_TRUE(true);
}
TEST(OndselSolver, quasikine) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/quasikine.asmt");	//Under constrained. Testing ICKine.
	EXPECT_TRUE(true);
}
TEST(OndselSolver, piston) {
	ASMTAssembly::readWriteFile(std::string(TEST_DATA_PATH) + "/piston.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runSinglePendulumSuperSimplified) {
	ASMTAssembly::runSinglePendulumSuperSimplified();	//Mass is missing
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runSinglePendulumSuperSimplified2) {
	ASMTAssembly::runSinglePendulumSuperSimplified2();	//DOF has infinite acceleration due to zero mass and inertias
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runSinglePendulumSimplified) {
	ASMTAssembly::runSinglePendulumSimplified();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runSinglePendulum) {
	ASMTAssembly::runSinglePendulum();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, piston2) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/piston.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, 00backhoe) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/00backhoe.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, circular) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/circular.asmt");	//Needs checking
	EXPECT_TRUE(true);
}
TEST(OndselSolver, engine1) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/engine1.asmt");	//Needs checking
	EXPECT_TRUE(true);
}
TEST(OndselSolver, fourbar) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/fourbar.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, fourbot) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/fourbot.asmt");	//Very large but works
	EXPECT_TRUE(true);
}
TEST(OndselSolver, wobpump) {
	ASMTAssembly::runFile(std::string(TEST_DATA_PATH) + "/wobpump.asmt");
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runOndselDoublePendulum) {
	auto cadSystem = std::make_shared<CADSystem>();
	cadSystem->runOndselDoublePendulum();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runOndselPiston) {
	auto cadSystem = std::make_shared<CADSystem>();
	cadSystem->runOndselPiston();		//For debugging
	EXPECT_TRUE(true);
}
TEST(OndselSolver, runPiston) {
	auto cadSystem = std::make_shared<CADSystem>();
	cadSystem->runPiston();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, GESpMatParPvPrecise) {
	GESpMatParPvPrecise::runSpMat();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, MomentOfInertiaSolver) {
	MomentOfInertiaSolver::example1();
	EXPECT_TRUE(true);
}
TEST(OndselSolver, sharedptrTest) {
	auto assm = ASMTAssembly::With();

	std::shared_ptr<ASMTAssembly> assm1 = assm;	//New shared_ptr to old object. Reference count incremented.
	assert(assm == assm1);
	assert(assm.get() == assm1.get());
	assert(&assm != &assm1);
	assert(assm->constantGravity == assm1->constantGravity);
	assert(&(assm->constantGravity) == &(assm1->constantGravity));

	auto assm2 = std::make_shared<ASMTAssembly>(*assm);	//New shared_ptr to new object. Member variables copy old member variables
	assert(assm != assm2);
	assert(assm.get() != assm2.get());
	assert(&assm != &assm2);
	assert(assm->constantGravity == assm2->constantGravity);	//constantGravity is same object pointed to
	assert(&(assm->constantGravity) != &(assm2->constantGravity)); //Different shared_ptrs of same reference counter
	EXPECT_TRUE(true);
}

// A rotational motion holds the commanded angle with its turns counted,
// whatever the output step: before, it held the direction cosine of I's y
// and J's x, zero half a turn away too, and a command that moved more than a
// quarter turn between output times left the body half a turn off. Gear A
// (radius 10, driven at 90*time on its revolute) turns gear B (radius 20)
// by -45*time, which a turn of A counted wrong puts half a turn off.
namespace {
	double turnAboutZ(const std::shared_ptr<ASMTPart>& part) {
		double w, x, y, z;
		part->getQuarternions(w, x, y, z);
		return 2.0 * std::atan2(z, w);
	}
	std::shared_ptr<ASMTMarker> marker(const std::string& name, double x) {
		auto m = CREATE<ASMTMarker>::With();
		m->setName(name);
		m->setPosition3D(x, 0.0, 0.0);
		m->setRotationMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
		return m;
	}
	std::shared_ptr<ASMTPart> part(const std::string& name, double x) {
		auto p = CREATE<ASMTPart>::With();
		p->setName(name);
		auto mass = CREATE<ASMTPrincipalMassMarker>::With();
		mass->setMass(1.0);
		mass->setDensity(1.0);
		mass->setMomentOfInertias(1.0, 1.0, 1.0);
		p->setPrincipalMassMarker(mass);
		p->setPosition3D(x, 0.0, 0.0);
		p->setRotationMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
		p->addMarker(marker("m", 0.0));
		return p;
	}
}
TEST(OndselSolver, rotationalMotionCountsTurns) {
	for (double hout : { 0.01, 0.05, 0.1, 0.2, 0.3 }) {
		auto assembly = CREATE<ASMTAssembly>::With();
		assembly->setName("Assembly");
		assembly->addMarker(marker("mA", 0.0));
		assembly->addMarker(marker("mB", 30.0));
		auto a = part("A", 0.0);
		auto b = part("B", 30.0);
		assembly->addPart(a);
		assembly->addPart(b);
		auto revA = CREATE<ASMTRevoluteJoint>::With();
		revA->setName("RevA");
		revA->setMarkerI("/Assembly/mA");
		revA->setMarkerJ("/Assembly/A/m");
		assembly->addJoint(revA);
		auto revB = CREATE<ASMTRevoluteJoint>::With();
		revB->setName("RevB");
		revB->setMarkerI("/Assembly/mB");
		revB->setMarkerJ("/Assembly/B/m");
		assembly->addJoint(revB);
		auto gear = CREATE<ASMTGearJoint>::With();
		gear->setName("Gear");
		gear->setMarkerI("/Assembly/A/m");
		gear->setMarkerJ("/Assembly/B/m");
		gear->radiusI = 10.0;
		gear->radiusJ = 20.0;
		assembly->addJoint(gear);
		auto motion = CREATE<ASMTRotationalMotion>::With();
		motion->setName("Motion");
		motion->setMarkerI("/Assembly/mA");
		motion->setMarkerJ("/Assembly/A/m");
		motion->setRotationZ("90*time");
		assembly->addMotion(motion);
		auto sim = assembly->simulationParameters;
		sim->settstart(0.0);
		sim->settend(1.2);
		sim->sethout(hout);
		sim->sethmin(1.0e-9);
		sim->sethmax(1.0);
		sim->seterrorTol(1.0e-6);
		assembly->runKINEMATIC();
		auto frames = assembly->numberOfFrames();
		ASSERT_GE(frames, 5u) << hout;
		for (size_t i = 0; i < frames; i++) {
			assembly->updateForFrame(i);
			auto t = assembly->times->at(i);
			EXPECT_NEAR(std::remainder(turnAboutZ(a) - 90.0 * t, 2.0 * M_PI), 0.0, 1.0e-9) << hout << " " << t;
			EXPECT_NEAR(std::remainder(turnAboutZ(b) + 45.0 * t, 2.0 * M_PI), 0.0, 1.0e-8) << hout << " " << t;
		}
		EXPECT_NEAR(assembly->times->back(), 1.2, 1.0e-9) << hout;
	}
}
