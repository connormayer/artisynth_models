package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.GL.GLViewer;
import maspack.util.ReaderTokenizer;
import artisynth.models.dynjaw.JawModel;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemMuscleStiffener;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;
import artisynth.models.tongue3d.HexTongueDemo;
import artisynth.models.tongue3d.TetTongueDemo;
import maspack.util.PathFinder;

import java.util.Timer;
import java.util.TimerTask;


public class BadinJawHyoidTonguePosition extends BadinJawHyoidTongue {

   private ArrayList<FemMarker> probeMarkers = new ArrayList<FemMarker>();

   public static PropertyList myProps =
      new PropertyList(BadinJawHyoidTonguePosition.class, BadinJawHyoidTongue.class);


   public BadinJawHyoidTonguePosition () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      class Probe {
         String name;
         double y, z;
         Probe(String name, double y, double z) {
            this.name = name;
            this.y = y;
            this.z = z;
         }
      }

      boolean enableMaximalCover = false;
      if (enableMaximalCover) {
         // TODO: enter important points here
      }

      // Define names and coordinates for probe mesh
      Probe[] probeGrid = new Probe[] {
         // Row 1
         new Probe("row1_1", 35, 125),
         new Probe("row1_2", 8, 97), // invisible
         new Probe("row1_3", -8, 97), // invisible
         new Probe("row1_4", -35, 125),
         // Row 2
         new Probe("row2_1", 16, 108),
         new Probe("row2_2", -8, 100), // invisible
         new Probe("row2_3", 0, 100), // invisible
         new Probe("row2_4", 8, 100), // invisible
         new Probe("row2_5", 16, 108),
         // Row 3
         new Probe("row3_1", 21, 100),
         new Probe("row3_2", 12, 136),
         new Probe("row3_3", 8, 136),
         new Probe("row3_4", 0, 136),
         new Probe("row3_5", -8, 136),
         new Probe("row3_6", -16, 136),
         new Probe("row3_7", -21, 100),
         // Row 4
         new Probe("row4_1", 12, 104), // invisible
         new Probe("row4_2", 8, 112),
         new Probe("row4_3", 4, 136),
         new Probe("row4_4", -4, 136),
         new Probe("row4_5", -8, 112),
         new Probe("row4_6", -12, 104), // invisible
         // Row 5
         new Probe("row5_1", 12, 104),
         new Probe("row5_2", 6, 128), // invisible
         new Probe("row5_3", 0, 104),
         new Probe("row5_4", 6, 128), // invisible
         new Probe("row5_5", -12, 104),
         // Row 6
         new Probe("row6_1", 0, 100),
      };

      // Add markers
      for (Probe probe : probeGrid) {
         FemNode3d best = null;
         double bestDist = Double.MAX_VALUE;
         for (int i = 0; i < tongue.numNodes(); i++) {
            FemNode3d node = tongue.getNode(i);
            Point3d pos = node.getPosition();
            double dist = Math.abs(pos.y - probe.y) + Math.abs(pos.z - probe.z);
            if (dist < bestDist) {
               best = node;
               bestDist = dist;
            }
         }
         if (best != null) {
            FemMarker mkr = new FemMarker(best.getPosition());
            mkr.setName(probe.name);
            RenderProps.setSphericalPoints(mkr, 2, Color.RED);
            tongue.addMarker(mkr);
            probeMarkers.add(mkr);
         }
      }

      // Tongue tip marker 
      FemMarker mkr = new FemMarker (-5,0, 145);
      mkr.setName("tongue tip");
      RenderProps.setSphericalPoints (mkr,  2,  Color.ORANGE);
      tongue.addMarker (mkr);
      probeMarkers.add (mkr);
      
      Timer timer = new Timer();
      
      TimerTask task = new TimerTask() {
         public void run() {
            for (FemMarker marker : probeMarkers) {
               Point3d pos = marker.getPosition();
               System.out.printf("%s: %.3f %.3f %.3f%n", marker.getName(), pos.x, pos.y, pos.z);
            }
         }
      };
      timer.scheduleAtFixedRate (task, 0, 10000);

      RenderProps.setVisible(myJawModel.frameMarkers(), true);
   }
   
   public FemMuscleModel getTongue() {
      return tongue;
   }
   
   public void addExciterProbe(String exciterName, double maxExcitation) {
      if (getInputProbes().get (exciterName + " exciter probe") == null) {
      NumericInputProbe nip =
         new NumericInputProbe(this, "models/jawmodel/models/tongue/exciters/" + exciterName
         + ":excitation", 0, 0.5);
      nip.addData (
         new double[] { 0.00, 0.0,
                        0.03, 0.0,
                        0.40, maxExcitation,
                        0.50, maxExcitation
                      }, NumericInputProbe.EXPLICIT_TIME);
      nip.setName (exciterName + " exciter probe");
      nip.setInterpolationOrder (Order.CubicStep);
      addInputProbe (nip);
      System.out.println("adding probe");
      System.out.println(exciterName + " " + maxExcitation);
   }
  }

     public void removeExciterProbe(String exciterName) {
        Probe p = getInputProbes().get(exciterName + " exciter probe");
        if (p != null) {
           removeInputProbe(p);
        }
     }
}
