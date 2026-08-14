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
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ModelComponent;
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
 * Adapted from FrankModel2BatchWorker by Infinity Fu, which is adapted from JawHyoidFemMuscleTongueBatchWorker by Conner Mayer.
 * Must live in src/artisynth/models/frank3/ to compile.
 */
public class FrankModel3BatchWorker extends SimpleTimedBatchWorker {

   protected String myOutputDirName = "output/";
   protected FrankModel3 root;
   protected ArrayList<MuscleExciter> exciters;
   protected PrintWriter myWriter;
   protected PrintWriter myPositionFileWriter;
   protected PrintWriter myFailedExcitationFileWriter;

   /** How long each exciter is held, in seconds. Make this match the batch stop time. Newly added.*/
   protected double myProbeDuration = 1.0;

   // Time window the FEMs must settle in. Set in setUpStopConditionMonitor().
   protected double mySettleTime;
   protected double myMaxTime;

   // Turn a FEM off (false) to skip saving its nodes.
   protected boolean recordTongue     = true;
   protected boolean recordFace       = true;
   protected boolean recordSoftPalate = true;
   protected boolean recordPharynx    = true;
   protected boolean recordLarynx     = true;

   public FrankModel3BatchWorker(String[] args)
         throws IllegalStateException, IOException {
      super(args);

      root = (FrankModel3) Main.getMain().getRootModel();
      // frank3 inherits frank2 exciters in several groups. getAllExciters() puts them in one list.
      exciters = root.getAllExciters();

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
      super.preSim();
      System.out.println("preSim finished for task " + myTaskCounter);
   }

   /**
    * Add one probe per exciter in this task. The value ramps up like the
    * jaw-tongue worker: hold 0, rise to the task value by 0.8 of the run with a
    * cubic step, then hold. Gentler than a hard step from t=0, which stresses
    * the FEM and can invert elements.
    */
   protected void addAllExciterProbes() {
      double d = myProbeDuration;
      double[] time = {0.0, 0.1 * d, 0.8 * d, d};
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

   /** Remove this task's probes so the next task starts clean. */
   protected void removeAllExciterProbes() {
      for (MuscleExciter exc : exciters) {
         // the probe has the same name as the exciter
         Probe p = root.getInputProbes().get(exc.getName());
         if (p != null) {
            root.removeInputProbe(p);
         }
      }
   }

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
      recordPosition();  // skipped to test whether skipping the node dump improves batch stability //now do not skip :)
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
   }

   /**
    * Sets the rule for success: every FEM that is on must stop moving (settle)
    * within the time window ending at myMaxTime. If it does not, the task fails.
    */
   @Override
   protected void setUpStopConditionMonitor() {
      root = (FrankModel3) Main.getMain().getRootModel();
      mySettleTime = 0.2;
      myMaxTime = myProbeDuration - myRootModel.getMaxStepSize(); // stop a bit before the end
      super.setUpStopConditionMonitor();
      // Nest equilibrium under the time gate so the cheap time check runs
      // first. ConditionCheckerBase evaluates outer checkCondition before the
      // nested checker; with the old order (equilibrium outer), every step
      // walked all FEM node velocities even outside the settle window.
      List<ModelComponent> comps = new LinkedList<>();
      addFemNodes(comps, recordTongue,     root.tongue);
      addFemNodes(comps, recordFace,       root.face);
      addFemNodes(comps, recordSoftPalate, root.softPalate);
      addFemNodes(comps, recordPharynx,    root.pharynx);
      addFemNodes(comps, recordLarynx,     root.larynx);
      EquilibriumChecker echk =
         new EquilibriumChecker(EquilibriumCondition.STATIC, 1, comps);
      TimeChecker tchk = new TimeChecker(
         TimeCondition.IN_RANGE_INCLUSIVE, echk,
         myMaxTime - mySettleTime + myRootModel.getMaxStepSize(), myMaxTime);
      myStopConditionMonitor.addConditionChecker(tchk);
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
