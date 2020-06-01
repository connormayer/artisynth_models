package artisynth.models.quantalLips;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import maspack.interpolation.Interpolation.Order;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.tools.batchsim.SimpleTimedBatchWorker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker;
import artisynth.tools.batchsim.conditions.TimeChecker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker.EquilibriumCondition;
import artisynth.tools.batchsim.conditions.TimeChecker.TimeCondition;
import artisynth.models.face.BadinFaceDemoLipOpening;

public class QuantalLipsBatchWorker extends SimpleTimedBatchWorker{
   
   protected double mySettleTime;
   protected BadinFaceDemoLipOpening root;
   protected FemMuscleModel face;
   protected ComponentList<MuscleBundle> exciters;
   protected FemNode3d[] nodes;
   protected PrintWriter myExcitationFileWriter;
   protected PrintWriter myGeometryFileWriter;
   protected PrintWriter myFailedExcitationFileWriter;
   protected PrintWriter myFailedExcitationTargetFileWriter;
   ArrayList<Double> excitations = new ArrayList<Double>();
   
   public QuantalLipsBatchWorker (String[] args) throws IllegalStateException, IOException {
      super (args);
      myMaxTime = 1.00;
      mySettleTime = 0.40;
      root = (BadinFaceDemoLipOpening)Main.getMain ().getRootModel ();
      face = (FemMuscleModel)root.findComponent ("models/mech/models/badinface");
      exciters =
         (ComponentList<MuscleBundle>)root.findComponent ("models/mech/models/badinface/bundles");
      ComponentList<FemNode3d> nodesList =
         (ComponentList<FemNode3d>)root
            .findComponent ("models/mech/models/badinface/nodes");
      nodes = new FemNode3d[nodesList.size ()];
      Iterator<FemNode3d> iter = nodesList.iterator ();
      for (int i = 0; i < nodesList.size (); i++) {
         nodes[i] = iter.next ();
      }
      root.removeAllInputProbes ();
      myExcitationFileWriter =
      new PrintWriter (
         new BufferedWriter (
            new FileWriter (
               new File (
                  myOutputDirName, "excitations." + myName + ".txt"),
               true)));
      myGeometryFileWriter =
      new PrintWriter (
         new BufferedWriter (
            new FileWriter (
               new File (
                  myOutputDirName, "geometry." + myName + ".txt"),
               true)));
      myFailedExcitationFileWriter =
      new PrintWriter (
         new BufferedWriter (
            new FileWriter (
               new File (
                  myOutputDirName, "failedexcitations." + myName + ".txt"),
               true)));
      myFailedExcitationTargetFileWriter =
      new PrintWriter (
         new BufferedWriter (
            new FileWriter (
               new File (
                  myOutputDirName, "failedexcitationtargets." + myName + ".txt"),
               true)));
      root.removeAllInputProbes ();
   }
   
   @Override
   protected void preSim () {
      Main.getMain ().clearWayPoints ();
      //removeAllExciterProbes ();
      //addAllExciterProbes ();
      modifyAllExciterProbes();
      super.preSim ();
      //System.out.println("preSim finished");
   }

   /*protected void addAllExciterProbes () {
      for (MuscleBundle exc : exciters) {
         root.addExciterProbe (exc.getName (), exc.getExcitation ());
      }
   }*/
   
   protected void modifyAllExciterProbes(){
      excitations.clear ();
      for(MuscleBundle exc : exciters){
         //System.out.println(exc.getName()+ " "+exc.getExcitation());
         excitations.add(exc.getExcitation ());
         if (root.getInputProbes().get(exc.getName () + " exciter probe") != null){
            NumericInputProbe numIP= 
               (NumericInputProbe)root.getInputProbes().get(exc.getName () + " exciter probe");
            numIP.getNumericList ().clear ();
            numIP.addData (new double []{ 0.00, 0.0, 0.03, 0.0, 0.40, exc.getExcitation (), 1.00,
                                          exc.getExcitation ()}, NumericInputProbe.EXPLICIT_TIME);
            numIP.setInterpolationOrder (Order.CubicStep);
         }
         else{
            NumericInputProbe numIP= 
            new NumericInputProbe (root, "models/mech/models/badinface/bundles/"+ exc.getName () + ":excitation",0,1);
            numIP.addData (new double []{ 0.00, 0.0, 0.03, 0.0, 0.40, exc.getExcitation (), 1.00,
                                       exc.getExcitation ()}, NumericInputProbe.EXPLICIT_TIME);
            numIP.setName(exc.getName () + " exciter probe");
            numIP.setInterpolationOrder (Order.CubicStep);
            root.addInputProbe(numIP);
         }
      }
   }
   
//   protected void addLipAreaProbe() {
//      NumericOutputProbe op = new NumericOutputProbe(root, "lipOpeningArea", ));
//   }

   protected void removeAllExciterProbes () {
      for (MuscleBundle exc : exciters) {
         Probe eProbe = root.getInputProbes().get (exc.getName () + " exciter probe");
         if (eProbe != null) {
            root.removeInputProbe (eProbe);
         }
      }
   }

   @Override
   protected void recordSimResults () {
      recordBinaryWayPoints (true);
      /*for(MuscleBundle exc : face.getMuscleBundles ()){
         System.out.println(exc.getName()+ " "+exc.getExcitation());
      }*/
      /*for(Double excitation : excitations){
         System.out.println(excitation);
      }*/

      // Record position of nodes in ASCII.
      File dir = new File (myOutputDirName, "ascii");
      dir.mkdirs ();
      File file = new File (dir, myTaskCounter + ".txt");
      PrintWriter pw = null;
      try {
         pw = new PrintWriter (new BufferedWriter (new FileWriter (file)));
         for (FemNode3d node : nodes) {
            pw.println (node.getPosition ());
         }
         pw.close ();
      }
      catch (IOException e) {
         System.err.println ("Error writing file ascii/" + file.getName ());
      }
      if(myCurrentTaskSuccessful){
         Double area = root.getLipOpeningArea();
         StringBuilder excitationbuilder = new StringBuilder ();
         StringBuilder geometrybuilder = new StringBuilder ();
         for (MuscleBundle exc : face.getMuscleBundles ()) {
            excitationbuilder.append (exc.getExcitation ()).append (" ");
         }
         excitationbuilder.deleteCharAt (excitationbuilder.length () - 1);            
//         double Area=Double.POSITIVE_INFINITY;
//         //boolean zero = false;
//         for(Double area : areas){
//            if(area<Area && area>1e-7){
//               Area=area;
//            }
//            /*else if(area<1e-7){
//               zero=true;
//            }
//            if(zero && area>1e-7){
//               Area=0;
//               break;
//            }*/
//            geometrybuilder.append(area).append (" ");
//         }
         if(area != 0){
            myExcitationFileWriter.println (excitationbuilder);
            myExcitationFileWriter.flush ();
            geometrybuilder.append(area);
            myGeometryFileWriter.println(geometrybuilder);
            myGeometryFileWriter.flush ();
         }
         else{
            myFailedExcitationFileWriter.println (excitationbuilder);
            myFailedExcitationFileWriter.flush ();
         }
      }
      else{
         StringBuilder failedexcitationbuilder = new StringBuilder ();
         StringBuilder failedexcitationtargetbuilder = new StringBuilder ();
         for (MuscleBundle exc : face.getMuscleBundles ()) {
            failedexcitationbuilder.append (exc.getExcitation ()).append (" ");
         }
         for(Double excitation : excitations){
            failedexcitationtargetbuilder.append (excitation).append (" ");
         }
         failedexcitationbuilder.deleteCharAt (failedexcitationbuilder.length () - 1);
         myFailedExcitationFileWriter.println (failedexcitationbuilder);
         myFailedExcitationFileWriter.flush ();
         failedexcitationtargetbuilder.deleteCharAt (failedexcitationtargetbuilder.length () - 1);
         myFailedExcitationTargetFileWriter.println (failedexcitationtargetbuilder);
         myFailedExcitationTargetFileWriter.flush ();
      }
   }

   @Override
   protected void postSim () {
      //removeAllExciterProbes ();
      Main.getMain ().clearWayPoints ();
   }
   
   @Override
   protected void setUpStopConditionMonitor () {
      super.setUpStopConditionMonitor ();

      TimeChecker tchk =
         new TimeChecker (
            TimeCondition.IN_RANGE_INCLUSIVE,
            myMaxTime - mySettleTime + myRootModel.getMaxStepSize (),
            myMaxTime);

      List<ModelComponent> comps = new LinkedList<> ();
      for (FemNode3d node : nodes) {
         comps.add (node);
      }
      EquilibriumChecker echk =
         new EquilibriumChecker (EquilibriumCondition.STATIC, tchk, 1, comps);

      myStopConditionMonitor.addConditionChecker (echk);
      /*
      System.out.println ("sT = " + mySettleTime);
      System.out.println ("mT = " + myMaxTime);
      System.out.println ("mSS = " + myRootModel.getMaxStepSize ());
      System.out.println ("# nodes for Static = " + comps.size ());*/
      
   }

}

