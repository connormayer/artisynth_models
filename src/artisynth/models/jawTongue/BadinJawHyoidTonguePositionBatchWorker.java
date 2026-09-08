package artisynth.models.jawTongue;

import java.io.BufferedWriter;
import java.io.File;
import java.awt.Color;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList; 
import java.util.List;
import java.util.AbstractMap.SimpleEntry;

import maspack.interpolation.Interpolation.Order;
import maspack.util.PathFinder;
import maspack.properties.Property;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.driver.Main;
import artisynth.core.probes.*;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;
import artisynth.models.tongue3d.HexTongueDemo;
import artisynth.models.jawTongue.JawHyoidFemMuscleTonguePosition;
import maspack.interpolation.Interpolation.Order;
import maspack.properties.PropertyMode;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.Probe;
import artisynth.core.femmodels.FemMarker;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import artisynth.tools.batchsim.SimpleTimedBatchWorker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker;
import artisynth.tools.batchsim.conditions.TimeChecker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker.EquilibriumCondition;
import artisynth.tools.batchsim.conditions.TimeChecker.TimeCondition;
import java.util.Timer;
import java.util.TimerTask;

public class BadinJawHyoidTonguePositionBatchWorker extends SimpleTimedBatchWorker {
   
   protected double mySettleTime;
   protected double myMaxTime;
   //yjc714為了batchworker可以開多個 於是註解
   //protected String myName = "default";
   protected BadinJawHyoidTongue root;
   protected FemMuscleModel face;
   protected FemMuscleModel tongue;
   protected ComponentList<MuscleExciter> exciters;
   protected MuscleExciter jawOpenerExciter;
   protected MuscleExciter jawCloserExciter;
   protected PrintWriter myPositionFileWriter;
   protected PrintWriter myExcitationFileWriter;
   protected PrintWriter myFailedExcitationFileWriter;

   private boolean isPositionHeaderWritten = false;

   @SuppressWarnings("unchecked")
   public BadinJawHyoidTonguePositionBatchWorker(String[] args) throws IllegalStateException, IOException {
      
      super(args);
      myMaxTime = 1.00;
      mySettleTime = 0.40;
      
      root = (BadinJawHyoidTongue) Main.getMain().getRootModel();
      root.setMaxStepSize(0.003);
      
      exciters = (ComponentList<MuscleExciter>) root.findComponent("models/jawmodel/models/tongue/exciters");
      jawOpenerExciter = (MuscleExciter) root.findComponent("models/jawmodel/exciters/bi_open");
      jawCloserExciter = (MuscleExciter) root.findComponent("models/jawmodel/exciters/bi_close");
      
      myPositionFileWriter = initWriter(myOutputDirName, "position." + myName + ".txt");
      myExcitationFileWriter = initWriter(myOutputDirName, "excitations." + myName + ".txt");
      myFailedExcitationFileWriter = initWriter(myOutputDirName, "failedexcitations." + myName + ".txt");
            
      root.removeAllInputProbes();
   }
   
   protected PrintWriter initWriter (String outputDir, String filename)
   throws IOException {
      return new PrintWriter (
         new BufferedWriter (
            new FileWriter (new File (outputDir, filename), true)));
   }
   
   @Override
   protected void preSim() {
      root = (BadinJawHyoidTongue) Main.getMain().getRootModel();
      tongue = root.getTongue();
      exciters = (ComponentList<MuscleExciter>) root.findComponent("models/jawmodel/models/tongue/exciters");
      jawOpenerExciter = (MuscleExciter) root.findComponent("models/jawmodel/exciters/bi_open");
      jawCloserExciter = (MuscleExciter) root.findComponent("models/jawmodel/exciters/bi_close");

      root.removeAllInputProbes();
      addAllExciterProbes();
      super.preSim();
      for(MuscleExciter exc : exciters) {
          System.out.println(exc.getName() + " " + exc.getExcitation());
       }
      System.out.println(jawOpenerExciter.getName() + " " + jawOpenerExciter.getExcitation());
      System.out.println(jawCloserExciter.getName() + " " + jawCloserExciter.getExcitation());
      System.out.println("preSim finished");
   }

//   protected void addAllExciterProbes() {
//      for(MuscleExciter exc : exciters) {
//         root.addExciterProbe(exc.getName(), exc.getExcitation());
//      }
//   }
   
   protected void addAllExciterProbes() {
	      for (String[] compPropVal : myCurrentTask) {
	         String propPath = compPropVal[0];
	         Property prop = myRootModel.getProperty (propPath);
	         if (prop.getHost () instanceof MuscleExciter) {
	            MuscleExciter exc = (MuscleExciter) prop.getHost();
	            root.addExciterProbe(exc.getName(), exc.getExcitation());
	         }
	      }
	   }


   protected void removeAllExciterProbes() {
      for(MuscleExciter exc : exciters) {
         Probe eProbe = root.getInputProbes().get(exc.getName() + " exciter probe");
         if(eProbe != null) {
            root.removeInputProbe(eProbe);
         }
      }
      
      Probe eProbe = root.getInputProbes().get(jawOpenerExciter.getName() + " exciter probe");
      if(eProbe != null) {
         root.removeInputProbe(eProbe);
      }
      
      eProbe = root.getInputProbes().get(jawCloserExciter.getName() + " exciter probe");
      if(eProbe != null) {
         root.removeInputProbe(eProbe);
      }
   }
   
   protected void recordPosition() {
	   if (root == null) {
	      System.err.println("Error: root model is null in recordPosition()");
	      return;
	   }

	   // Get the tongue model
	   FemMuscleModel tongue = root.getTongue();
	   if (tongue == null) {
	      System.err.println("Error: tongue model is null in recordPosition()");
	      return;
	   }

      // Write all probe marker positions
      StringBuilder builder = new StringBuilder ();
      for (int i = 0; i < tongue.markers().size(); i++) {
         FemMarker marker = (FemMarker) tongue.markers().get(i);
         String name = marker.getName();

         Point3d pos = marker.getPosition();
         String point_string = String.format(
            "%d,%s,%.3f,%.3f,%.3f%n",
               myTaskCounter,
               name,
               pos.x, pos.y, pos.z
         );
         builder.append(point_string);
      }
      myPositionFileWriter.print(builder);
      myPositionFileWriter.flush();
   }
   
   @Override
   protected void recordSimResults() {
      if(myCurrentTaskSuccessful){
    	  recordPosition();
         // Record excitations
         StringBuilder builder = new StringBuilder ();
         builder.append (myTaskCounter).append (",");
         for (String[] propVal : myCurrentTask) {
            builder.append (propVal[1]).append (",");
         }
         builder.deleteCharAt (builder.length () - 1);
         myExcitationFileWriter.println (builder);
         myExcitationFileWriter.flush ();
      }
      else{
         StringBuilder failedexcitationbuilder = new StringBuilder();

         for(MuscleExciter exc : exciters) {
            failedexcitationbuilder.append(exc.getExcitation()).append(",");
         }
         failedexcitationbuilder.append(jawOpenerExciter.getExcitation ()).append (",");
         failedexcitationbuilder.append(jawCloserExciter.getExcitation ()).append (",");
         failedexcitationbuilder.deleteCharAt(failedexcitationbuilder.length() - 1);
         myFailedExcitationFileWriter.println(failedexcitationbuilder);
         myFailedExcitationFileWriter.flush();
      }
   }

   @Override
   protected void postSim() {
      removeAllExciterProbes();
   }
   
   @Override
   protected void setUpStopConditionMonitor() {
       myMaxTime = 1.00;
       super.setUpStopConditionMonitor();

       double maxStep = myRootModel.getMaxStepSize();
       TimeChecker tchk = new TimeChecker(
           TimeCondition.IN_RANGE_INCLUSIVE, 
           myMaxTime - maxStep, 
           myMaxTime + maxStep
       );

       myStopConditionMonitor.addConditionChecker(tchk);
   }


   
   @Override
   public void closeWriters () {
      super.closeWriters ();
      myPositionFileWriter.close ();
      myExcitationFileWriter.close ();
      myFailedExcitationFileWriter.close ();
   }

}
