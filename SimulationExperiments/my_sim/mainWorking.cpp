/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         
 *
 ***************************************************************************/

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h> // arena
#include <ode_robots/octaplayground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/boxpile.h>
#include <ode_robots/passivesphere.h>  // passive balls

// controller
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/invertmotornstep.h>
#include "ESN.h"
#include "esncontroller.h"

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/hexapod.h>
#include <ode_robots/operators.h>
#include <ode_robots/speedsensor.h>

#include <ode_robots/axisorientationsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

double    teacher   = 0;

class ThisSim : public Simulation {
public:
  AbstractController* controller;
  Sphererobot3Masses* sphere1;
  OdeRobot* vehicle;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
	int numSphere = 1;
	int numHex = 0;
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting initial position of the playground: setPosition(osg::Vec3(double x, double y, double z))
    // - push playground to the global list of obstacles (global list comes from simulation.cpp)
    /*    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 1), 12);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);
    */
    // add passive spheres as obstacles
    // - create pointer to sphere (with odehandle, osghandle and
    //   optional parameters radius and mass,where the latter is not used here) )
    // - set Pose(Position) of sphere
    // - add sphere to list of obstacles
	  // use Playground as boundary:
   /* AbstractGround* playground =
      new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
    //     // playground->setColor(Color(0,0,0,0.8));
    playground->setGroundColor(Color(2,2,2,1));
    playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    Boxpile* boxpile = new Boxpile(odeHandle, osgHandle);
    boxpile->setColor("wall");
    boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(0, 0,0.2));
    global.obstacles.push_back(boxpile);
	*/
/*

    for(int i=0; i<4; i++){
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 0.5);
      s->setPosition(osg::Vec3(0,3,i*3));
      global.obstacles.push_back(s);
    }
*/
    // Spherical Robot with axis (gyro) sensors:
    // - get default configuration for robot
    // - create pointer to spherical robot (with odeHandle, osgHandle and configuration)
    // - place robot (unfortunatelly there is a type cast necessary, which is not quite understandab
    for(int i=0; i<numSphere; i++){
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    // regular behaviour
    conf.motorsensor=false;
    conf.diameter=1.0;
    conf.pendularrange= 0.2;
//     conf.diameter=1.0;
//     conf.pendularrange= 0.35;
    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)),
                                       conf, "Sphere1", 0.2);
    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
    global.configs.push_back ( sphere1 );

    // Selforg - Controller
    // create pointer to controller
    // set some parameters
    // push controller in global list of configurables
		/*
    controller = new InvertMotorSpace(10);
    controller->setParam("epsA",0.3); // model learning rate
    controller->setParam("epsC",0.3); // controller learning rate
    controller->setParam("rootE",3);    // model and contoller learn with square rooted error
    global.configs.push_back ( controller );
*/
//     //  SineController (produces just sine waves)
//     controller = new SineController();
//     controller->setParam("sinerate", 40);
//     controller->setParam("phaseshift", 0.0);
	InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      //      DerControllerConf cc = DerController::getDefaultConf();
      cc.cInit=1.0;
      cc.useSD=true;
      
      controller = new InvertMotorNStep(cc);
      
      controller->setParam("steps", 1);
      controller->setParam("adaptrate", 0.001);
      // controller->setParam("adaptrate", 0.0);
      controller->setParam("nomupdate", 0.005);
      controller->setParam("epsC", 0.1);
      controller->setParam("epsA", 0.1);
      controller->setParam("rootE", 3);
      controller->setParam("logaE", 0);
      
    // create pointer to one2onewiring which uses colored-noise
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise(0.25) );

    // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , sphere1 , wiring );
    // the following line will enable a position tracking of the robot, which is written into a file
    //agent->setTrackOptions(TrackRobot(true, true, true, false, "Sphere_zaxis", 20));
    global.agents.push_back ( agent );
	global.configs.push_back ( controller );
	// display all parameters of all configurable objects on the console
	}
	  
	 for ( int ii = 0; ii< numHex; ii++){

    HexapodConf myHexapodConf        = Hexapod::getDefaultConf();
    myHexapodConf.coxaPower          = 1.5;
    myHexapodConf.tebiaPower         = 0.8;
    myHexapodConf.coxaJointLimitV    = .9; // M_PI/8;  // angle range for vertical dir. of legs
    myHexapodConf.coxaJointLimitH    = 1.3; //M_PI/4;
    myHexapodConf.tebiaJointLimit    = 1.8; // M_PI/4; // +- 45 degree
    myHexapodConf.percentageBodyMass = .5;
    myHexapodConf.useBigBox          = false;
    myHexapodConf.tarsus             = true;
    myHexapodConf.numTarsusSections  = 1;
    myHexapodConf.useTarsusJoints    = true;
    //    myHexapodConf.numTarsusSections = 2;

    OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance.toRubber(20);


    vehicle = new Hexapod(rodeHandle, osgHandle.changeColor("Green"),
                          myHexapodConf, "Hexapod_" + std::itos(teacher*10000));

    // on the top
    vehicle->place(osg::Matrix::rotate(0,0,0,0)*osg::Matrix::translate(0,0,1.5+ 2*ii));
    // normal position
    //    vehicle->place(osg::Matrix::translate(0,0,0));

//     InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
//     cc.cInit=1.0;
//     cc.useS=false;
    //    cc.someInternalParams=true;
//     InvertMotorNStep *semox = new InvertMotorNStep(cc);
//     semox->setParam("steps", 1);
//     semox->setParam("continuity", 0.005);
//     semox->setParam("teacher", teacher);
	
		 InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      //      DerControllerConf cc = DerController::getDefaultConf();
      cc.cInit=1.0;
      cc.useSD=true;
      //controller = new DerController(cc);
      controller = new InvertMotorNStep(cc);
      // controller = new SineController();
      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      controller->setParam("steps", 1);
      //    controller->setParam("adaptrate", 0.001);
      controller->setParam("adaptrate", 0.0);
      controller->setParam("nomupdate", 0.005);
      controller->setParam("epsC", 0.1);
      controller->setParam("epsA", 0.1);
      // controller->setParam("epsC", 0.001);
      // controller->setParam("epsA", 0.001);
      //    controller->setParam("rootE", 1);
      //    controller->setParam("logaE", 2);
      controller->setParam("rootE", 3);
      controller->setParam("logaE", 0);
      //     controller = new SineController();
      controller->setParam("sinerate", 15);
      controller->setParam("phaseshift", 0.45);
    

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    // the feedbackwiring feeds here 75% of the motor actions as inputs and only 25% of real inputs
//     AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
//                                                 FeedbackWiring::Motor, 0.75);
    //global.plotoptions.push_back(PlotOption(GuiLogger,Robot,5));
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    // add an operator to keep robot from falling over
    agent->addOperator(new LimitOrientationOperator(Axis(0,0,1), Axis(0,0,1), M_PI*0.5, 30));
    
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    //agent->startMotorBabblingMode(5000);

    // this->getHUDSM()->setColor(Color(1.0,1.0,0));
    // this->getHUDSM()->setFontsize(18);
    // this->getHUDSM()->addMeasure(teacher,"gamma_s",ID,1);

  }

  }

  /** is called if a key was pressed.
      For keycodes see: osgGA::GUIEventAdapter
      @return true if the key was handled
  */
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData,
                       int key, bool down) {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key ) {
      case 'X' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
      case 'x' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
      case 'T' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , 3 ); break;
      case 't' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , -3 ); break;
//       case 'S' : controller->setParam("sineRate", controller->getParam("sineRate")*1.2);
//         printf("sineRate : %g\n", controller->getParam("sineRate"));
//       break;
//       case 's' : controller->setParam("sineRate", controller->getParam("sineRate")/1.2);
//         printf("sineRate : %g\n", controller->getParam("sineRate"));
//         break;
      default:
        return false;
      }
      return true;
    } else return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: X","Push robot to right (positive x)");
    au.addKeyboardMouseBinding("Simulation: x","Push robot to left (negative x)");
    au.addKeyboardMouseBinding("Simulation: T","Spin robot counter-clockwise");
    au.addKeyboardMouseBinding("Simulation: t","Spin robot clockwise");
    //    au.addKeyboardMouseBinding("Controller: S","Increase sine frequency");
    //    au.addKeyboardMouseBinding("Controller: s","Decrease sine frequency");
  }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("Images/yellow_velour.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}


