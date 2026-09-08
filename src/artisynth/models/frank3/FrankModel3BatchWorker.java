package artisynth.models.frank3;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import maspack.interpolation.Interpolation.Order;
import maspack.matrix.Point3d;
import maspack.properties.Property;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.Monitor;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.Probe;
import artisynth.models.frank2.FrankActivations;
import artisynth.tools.batchsim.SimpleTimedBatchWorker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker.EquilibriumCondition;
import artisynth.tools.batchsim.conditions.TimeChecker;
import artisynth.tools.batchsim.conditions.TimeChecker.TimeCondition;

/**
 * BatchWorker for {@link FrankModel3}.
 *
 * The BatchManager sends this worker one task at a time. Each task is a set of
 * exciter values. For each task, the worker:
 *   1. adds a probe that holds each exciter at its task value,
 *   2. runs the sim until the FEMs settle or it times out,
 *   3. if it settled: writes the values to excitations.txt and the node
 *      positions to position.txt. If not: writes the values to
 *      failedexcitations.txt,
 *   4. removes the probes before the next task.
 *
 * The record* flags below choose which FEMs are saved and which must settle.
 *
 * Adapted from FrankModel2BatchWorker by Infinity Fu, which is adapted from JawHyoidFemMuscleTongueBatchWorker.
 * Must live in src/artisynth/models/frank3/ to compile.
 */
public class FrankModel3BatchWorker extends SimpleTimedBatchWorker {

   protected String myOutputDirName = "output/";
   protected FrankModel3 root;
   protected ArrayList<MuscleExciter> exciters;
   //20260830Cyd
   protected MuscleExciter jawCloseExciter;
   protected double myJawCloseExcitation = 0.004;
   //-----20260830Cyd
   //20260901Shitong - delay tongue-maxilla contact until jawSettleTime
   protected double myJawSettleTime;
   protected boolean myDelayTongueMaxillaContact;
   protected boolean myTongueMaxillaContactEnabled;
   protected Monitor myContactDelayMonitor;
   //--20260901 Shitong
   protected PrintWriter myWriter;
   protected PrintWriter myPositionFileWriter;
   protected PrintWriter myFailedExcitationFileWriter;

   /** How long each exciter is held, in seconds. Make this match the batch stop time. Newly added.*/
   //20260827Cyd
   //protected double myProbeDuration = 1.0;
   protected double myProbeDuration = 1.2 ; 
   //----20260827Cyd
   // Time window the FEMs must settle in. Set in setUpStopConditionMonitor().
   protected double mySettleTime;
   //protected double myMaxTime;//20260904 Cyd

   // Turn a FEM off (false) to skip saving its nodes.
   protected boolean recordTongue     = true;
   //20260817Cyd----turn off the checkers
   protected boolean recordFace       = false;
   protected boolean recordSoftPalate = false;
   protected boolean recordPharynx    = false;
   protected boolean recordLarynx     = false;
   //--------------20260817Cyd
   public FrankModel3BatchWorker(String[] args)
         throws IllegalStateException, IOException {
      super(args);

      root = (FrankModel3) Main.getMain().getRootModel();
      //20260819Cyd
      root.setMaxStepSize(0.003); 
      //----20260819Cyd
      // frank3 inherits frank2 exciters in several groups. getAllExciters() puts them in one list.
      exciters = root.getAllExciters();
      //20260830Cyd
      jawCloseExciter = FrankActivations.makeJawCloseExciter(exciters);
      root.mechModel.addMuscleExciter(jawCloseExciter);
      //---20260830Cyd
      myWriter = initWriter(myOutputDirName, "excitations.txt");
      myPositionFileWriter = initWriter(myOutputDirName, "position.txt");
      myFailedExcitationFileWriter =
         initWriter(myOutputDirName, "failedexcitations.txt");

      root.removeAllInputProbes();
   }

   protected PrintWriter initWriter(String outputDir, String filename)
         throws IOException {
      new File(outputDir).mkdirs();
      return new PrintWriter(
         new BufferedWriter(
            new FileWriter(new File(outputDir, filename), true)));
   }

   @Override
   protected void preSim() {
      // Get the model again and clear the last task's probes.
      root = (FrankModel3) Main.getMain().getRootModel();
      root.removeAllInputProbes();
      addAllExciterProbes();
      setupDelayedTongueMaxillaContact(); //20260901 Shitong
      super.preSim();
      System.out.println("preSim finished for task " + myTaskCounter);
   }

   /**
    * Add one probe per exciter in this task. The value ramps up like the
    * jaw-tongue worker: hold 0, rise to the task value by 0.8 of the run with a
    * cubic step, then hold. Gentler than a hard step from t=0, which stresses
    * the FEM and can invert elements.
    */
   /* 20260830Cyd
   protected void addAllExciterProbes() {
      double d = myProbeDuration;
      //20260827Cyd
      //double[] time = {0.0, 0.1 * d, 0.8 * d, d};
      double[] time = {0.0, 0.02 * d, 0.85 * d, d};
      //----20260827Cyd
      for (String[] compPropVal : myCurrentTask) {
         String propPath = compPropVal[0];
         Property prop = myRootModel.getProperty(propPath);
         if (prop.getHost() instanceof MuscleExciter) {
            MuscleExciter exc = (MuscleExciter) prop.getHost();
            double v = exc.getExcitation();   // the framework already set this value for the task
            NumericInputProbe p = FrankActivations.createMuscleProbe(
               exc, exc.getName(), time, new double[] {0.0, 0.0, v, v});
            p.setInterpolationOrder(Order.CubicStep);
            root.addInputProbe(p);
         }
      }
   }
      */
   /*20260830Cyd
   protected void addAllExciterProbes() {
      double d = myProbeDuration;
      double settleTime = 0.02 * d;   // jaw closers ramp to target during [0, settleTime]
      double peakTime    = 0.85 * d;   // tongue muscles ramp to target during [settleTime, peakTime]
      double maxTime     = d;

      // jaw closers (AT/MT/PT/DM/SM/MP combined): 0 -> target over [0, settleTime], then hold
      NumericInputProbe jp = FrankActivations.createMuscleProbe(
         jawCloseExciter, jawCloseExciter.getName(),
         new double[] {0.0, settleTime, maxTime},
         new double[] {0.0, myJawCloseExcitation, myJawCloseExcitation});
      jp.setInterpolationOrder(Order.CubicStep);
      root.addInputProbe(jp);

      // tongue muscles for this task: hold 0 until settleTime, ramp to v by peakTime, then hold
      double[] tongueTime = {0.0, settleTime, peakTime, maxTime};
      for (String[] compPropVal : myCurrentTask) {
         String propPath = compPropVal[0];
         Property prop = myRootModel.getProperty(propPath);
         if (prop.getHost() instanceof MuscleExciter) {
            MuscleExciter exc = (MuscleExciter) prop.getHost();
            double v = exc.getExcitation();
            NumericInputProbe p = FrankActivations.createMuscleProbe(
               exc, exc.getName(), tongueTime, new double[] {0.0, 0.0, v, v});
            p.setInterpolationOrder(Order.CubicStep);
            root.addInputProbe(p);
         }
      }
   }

   -----20260830Cyd*/
   //20260831 Cyd
   
   protected void addAllExciterProbes() {
      double d = myProbeDuration;
      double jawRampTime   = 0.02 * d;   
      //double jawSettleTime = 0.08 * d;

      double jawSettleTime = 0.15 * d;//20260901Shitong
      myJawSettleTime = jawSettleTime;

      double peakTime      = 0.85 * d;   
      //double maxTime        = d;
      double maxTime       = d; //20260904 Shitong

      // jaw closers (AT/MT/PT/DM/SM/MP combined): 0 -> target over [0, jawRampTime], then hold
      NumericInputProbe jp = FrankActivations.createMuscleProbe(
         jawCloseExciter, jawCloseExciter.getName(),
         new double[] {0.0, jawRampTime, maxTime},
         new double[] {0.0, myJawCloseExcitation, myJawCloseExcitation});
      jp.setInterpolationOrder(Order.CubicStep);
      root.addInputProbe(jp);

      // tongue muscles for this task: hold 0 until jawSettleTime（不是 jawRampTime），
      // ramp to v by peakTime, then hold
      double[] tongueTime = {0.0, jawSettleTime, peakTime, maxTime};
      for (String[] compPropVal : myCurrentTask) {
         String propPath = compPropVal[0];
         Property prop = myRootModel.getProperty(propPath);
         if (prop.getHost() instanceof MuscleExciter) {
            MuscleExciter exc = (MuscleExciter) prop.getHost();
            double v = exc.getExcitation();
            NumericInputProbe p = FrankActivations.createMuscleProbe(
               exc, exc.getName(), tongueTime, new double[] {0.0, 0.0, v, v});
            p.setInterpolationOrder(Order.CubicStep);
            root.addInputProbe(p);
         }
      }
   }

   //-----20260831Cyd

   /** Remove this task's probes so the next task starts clean. */
   /* 20260830Cyd
   protected void removeAllExciterProbes() {
      for (MuscleExciter exc : exciters) {
         // the probe has the same name as the exciter
         Probe p = root.getInputProbes().get(exc.getName());
         if (p != null) {
            root.removeInputProbe(p);
         }
      }
   }
      */
   //20260830Cyd
   protected void removeAllExciterProbes() {
      for (MuscleExciter exc : exciters) {
         // the probe has the same name as the exciter
         Probe p = root.getInputProbes().get(exc.getName());
         if (p != null) {
            root.removeInputProbe(p);
         }
      }
      Probe jp = root.getInputProbes().get(jawCloseExciter.getName());
      if (jp != null) {
         root.removeInputProbe(jp);
      }
   }
   //-----20260830Cyd

   /** Save the node positions of every FEM that is turned on. */
   protected void recordPosition() {
      recordFemPositions(recordTongue,     root.tongue);
      recordFemPositions(recordFace,       root.face);
      recordFemPositions(recordSoftPalate, root.softPalate);
      recordFemPositions(recordPharynx,    root.pharynx);
      recordFemPositions(recordLarynx,     root.larynx);
      myPositionFileWriter.flush();
   }

   /**
    * Write one line per node: taskCounter, femName, nodeNumber, x, y, z.
    * Uses the node number, not the name (FEM nodes usually have no name).
    * The FEM name is included because node numbers restart at 0 for each FEM.
    */
   protected void recordFemPositions(boolean enabled, FemMuscleModel fem) {
      if (!enabled) {
         return;
      }
      if (fem == null) {
         System.err.println("Error: FEM is null in recordFemPositions()");
         return;
      }
      Point3d pos = new Point3d();
      for (FemNode3d n : fem.getNodes()) {
         n.getPosition(pos);
         StringBuilder builder = new StringBuilder();
         builder.append(myTaskCounter).append(",");
         builder.append(fem.getName()).append(",");
         builder.append(n.getNumber()).append(",");
         builder.append(pos.x).append(",");
         builder.append(pos.y).append(",");
         builder.append(pos.z);
         myPositionFileWriter.println(builder.toString());
      }
   }

   @Override
   protected void recordSimResults() {
      if (!myCurrentTaskSuccessful) {
         // Task failed. Save the values (same columns as excitations.txt) so
         // we know which combination failed.
         StringBuilder failed = new StringBuilder();
         failed.append(myTaskCounter);
         for (String[] propVal : myCurrentTask) {
            failed.append(",").append(propVal[1]);
         }
         myFailedExcitationFileWriter.println(failed);
         myFailedExcitationFileWriter.flush();
         return;
      }
      recordPosition();
      StringBuilder sb = new StringBuilder();
      sb.append(myTaskCounter);
      for (String[] propVal : myCurrentTask) {
         sb.append(",").append(propVal[1]);
      }
      myWriter.println(sb);
      myWriter.flush();
   }

   @Override
   protected void postSim() {
      removeAllExciterProbes();
      //20260901Cyd
      removeContactDelayMonitor();
      //----20260901Cyd
   }

   //20260901Cyd
   /**
    * If tongue-maxilla contact is supposed to be on (Activated), keep it off
    * until jawSettleTime so the jaw can close first. Deactivated stays off.
    */
   protected void setupDelayedTongueMaxillaContact() {
      removeContactDelayMonitor();
      myDelayTongueMaxillaContact = false;
      if (root instanceof FrankModel3Position) {
         myDelayTongueMaxillaContact =
            ((FrankModel3Position) root).collideTongueMaxilla;
      }
      else {
         CollisionBehavior b =
            root.mechModel.getCollisionBehavior(root.tongue, root.maxilla);
         myDelayTongueMaxillaContact = (b != null && b.isEnabled());
      }
      myTongueMaxillaContactEnabled = false;
      if (myDelayTongueMaxillaContact) {
         root.mechModel.setCollisionBehavior(root.tongue, root.maxilla, false);
         myContactDelayMonitor = new TongueMaxillaContactDelayMonitor();
         root.addMonitor(myContactDelayMonitor);
      }
   }

   protected void removeContactDelayMonitor() {
      if (myContactDelayMonitor != null) {
         root.removeMonitor(myContactDelayMonitor);
         myContactDelayMonitor = null;
      }
   }

   protected class TongueMaxillaContactDelayMonitor extends MonitorBase {
      public void apply(double t0, double t1) {
         if (!myTongueMaxillaContactEnabled && t1 >= myJawSettleTime) {
            root.mechModel.setCollisionBehavior(
               root.tongue, root.maxilla, true, 0.0);
            myTongueMaxillaContactEnabled = true;
         }
      }
   }
   //----20260901Cyd

   /**
    * Sets the rule for success: every FEM that is on must stop moving (settle)
    * within the time window ending at myMaxTime. If it does not, the task fails.
    */
   @Override
   protected void setUpStopConditionMonitor() {
      root = (FrankModel3) Main.getMain().getRootModel();
      mySettleTime = 0.2;
      // myMaxTime = myProbeDuration - myRootModel.getMaxStepSize(); // stop a bit before the end
      myMaxTime = 0.5; //20260904 Shitong
      super.setUpStopConditionMonitor();
      /* 20260831Cyd 
      TimeChecker tchk = new TimeChecker(
         TimeCondition.IN_RANGE_INCLUSIVE,
         myMaxTime - mySettleTime + myRootModel.getMaxStepSize(), myMaxTime);
      ------20260831Cyd */
      //20260819Cyd----comment out equilibrium checker, only require sim to survive to myMaxTime (like Badin Position worker)
      //List<ModelComponent> comps = new LinkedList<>();
      //addFemNodes(comps, recordTongue,     root.tongue);
      //addFemNodes(comps, recordFace,       root.face);
      //addFemNodes(comps, recordSoftPalate, root.softPalate);
      //addFemNodes(comps, recordPharynx,    root.pharynx);
      //addFemNodes(comps, recordLarynx,     root.larynx);
      //EquilibriumChecker echk =
      //   new EquilibriumChecker(EquilibriumCondition.STATIC, tchk, 1, comps);
      //myStopConditionMonitor.addConditionChecker(echk);

      /*20260831cyd
      myStopConditionMonitor.addConditionChecker(tchk);
      ----20260831Cyd*/
      //-----20260819Cyd
      
   }

   private void addFemNodes(
      List<ModelComponent> comps, boolean enabled, FemMuscleModel fem) {
      if (enabled && fem != null) {
         for (FemNode3d n : fem.getNodes()) {
            comps.add(n);
         }
      }
   }

   @Override
   public void closeWriters() {
      super.closeWriters();
      myWriter.close();
      myPositionFileWriter.close();
      myFailedExcitationFileWriter.close();
   }
}
