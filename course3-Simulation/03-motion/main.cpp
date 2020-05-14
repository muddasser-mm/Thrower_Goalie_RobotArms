#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>

//===========================================================================

void using_KOMO_for_IK(){
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame* obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state", true);

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 1, 1., 1);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_eq, {1e2});

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getJointState(1.);

  C.setJointState(q); //set your working config into the optimized state
  V.setConfiguration(C, "optimized configuration", true); //display it


  //-- redoing the optimization with the same KOMO object!
  //   (Warning: In doubt, rather create a new KOMO instance for each optimization.)

  //let's change an objective:
  std::shared_ptr<Objective> ob = komo.objectives(1); //which is the positionDiff added above
  ob->feat->setTarget({0., 0., .1}); //new target in that feature space: 10cm height difference
  //optimize
  komo.optimize(0.); //don't add noise or reinitialize

  C.setJointState(komo.getJointState(1.)); //set your working config into the optimized state
  V.setConfiguration(C, "optimized configuration", true); //display it

}

//===========================================================================

void using_KOMO_for_PathPlanning(){
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame* obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state", true);

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 40, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2});

  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getJointState(1.);

  C.setJointState(q); //set your working config into the optimized state
  V.setConfiguration(C, "optimized configuration", true); //display it

  V.setPath(komo.getPath_frames(), "optimized path", true);
  V.playVideo(true);

  rai::wait(.1); //TODO: otherwise the opengl gets hung up?
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  using_KOMO_for_IK();
  using_KOMO_for_PathPlanning();

  return 0;
}
