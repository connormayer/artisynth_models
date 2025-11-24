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


public class BadinJawHyoidTonguePositionDeactivated extends BadinJawHyoidTongue {

   private ArrayList<FemMarker> probeMarkers = new ArrayList<FemMarker>();

   public static PropertyList myProps =
      new PropertyList(BadinJawHyoidTonguePositionDeactivated.class, BadinJawHyoidTongue.class);


   public BadinJawHyoidTonguePositionDeactivated () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      collideTongueMaxilla = false;
      collideTongueJaw     = false;
      super.build (args);

      // Define y and z coordinates for marker placement
      double[] yCoords = new double[] {
         35, 8, -8, -35, // row 1
         16, -8, 0, 8, 16, // row 2
         21, 12, 8, 0, -8, -16, -21, // row 3
         12, 8, 4, -4, -8, -12, // row 4
         12, 6, 0, 6, -12, // row 5
         0 // row 6
      };
      double[] zCoords = new double[] {
         125, 97, 97, 125, // row 1
         108, 100, 100, 100, 108, // row 2
         100, 136, 136, 136, 136, 136, 100, // row 3
         104, 112, 136, 136, 112, 104, // row 4
         104, 128, 104, 128, 104, // row 5
         100 // row 6
      };

      // Define x and y offsets to search for the best node placement given a (y, z) coordinate
      double[] xOffsets = new double[] { -25.0, -10.0, 0.0, 10.0, 25.0 };
      double[] yOffsets = new double[] { 0.0, -12.0, 12.0 };

      // This data structure ensures that each node is only used once.
      java.util.HashSet<Integer> usedNodeIndices = new java.util.HashSet<Integer>();

      // For each set of coordinates, build the local target grid using offsets.
      for (int idx = 0; idx < yCoords.length; idx++) {
         double y = yCoords[idx];
         double z = zCoords[idx];

         // Find base node for this (y, z)
         FemNode3d base = null;
         double baseDist = Double.MAX_VALUE;
         for (int i = 0; i < tongue.numNodes(); i++) {
            FemNode3d node = tongue.getNode(i);
            Point3d pos = node.getPosition();
            double dist = Math.abs(pos.y - y) + Math.abs(pos.z - z);
            if (dist < baseDist) {
               base = node;
               baseDist = dist;
            }
         }

         if (base == null) {
            continue;
         }

         double baseX = base.getPosition().x;

         // Choose a candidate node for this particular (y, z)
         for (double off : xOffsets) {
            for (double ly : yOffsets) {
               double targetX = baseX + off;
               double targetY = y + ly;
               FemNode3d best = null;
               int bestIndex = -1;
               double bestDist = Double.MAX_VALUE;

               double searchThreshold = 20.0;
               FemNode3d bestHigh = null;
               int bestHighIndex = -1;
               double bestHighZ = -Double.MAX_VALUE;
               for (int i = 0; i < tongue.numNodes(); i++) {
                  FemNode3d node = tongue.getNode(i);
                  Point3d pos = node.getPosition();
                  double xyDist = Math.abs(pos.y - targetY) + Math.abs(pos.x - targetX);
                  if (xyDist <= searchThreshold) {
                     if (pos.z > bestHighZ) {
                        bestHigh = node;
                        bestHighIndex = i;
                        bestHighZ = pos.z;
                     }
                  }
                  if (xyDist < bestDist) {
                     best = node;
                     bestIndex = i;
                     bestDist = xyDist;
                  }
               }

               // Check if the chosen node is valid or has already been used
               FemNode3d chosen = (bestHigh != null) ? bestHigh : best;
               int chosenIndex = (bestHigh != null) ? bestHighIndex : bestIndex;
               if (chosen != null && chosenIndex >= 0 && !usedNodeIndices.contains(chosenIndex)) {
                  
                  // Name each probe using its coordinates
                  Point3d bpos = chosen.getPosition();
                  int xi = (int)Math.round(targetX);
                  int yi = (int)Math.round(targetY);
                  int zi = (int)Math.round(bpos.z);
                  String xs = (xi < 0) ? ("n" + Math.abs(xi)) : ("p" + xi);
                  String ys = (yi < 0) ? ("n" + Math.abs(yi)) : ("p" + yi);
                  String zs = (zi < 0) ? ("n" + Math.abs(zi)) : ("p" + zi);
                  String name = String.format("marker%d_x%s_y%s_z%s", idx, xs, ys, zs);
                  
                  // Create the marker and add it to the model 
                  FemMarker xmkr = new FemMarker(bpos);
                  xmkr.setName(name);
                  RenderProps.setSphericalPoints(xmkr, 2, Color.BLUE);
                  tongue.addMarker(xmkr);
                  probeMarkers.add(xmkr);
                  usedNodeIndices.add(chosenIndex);
               }
            }
         }
      }
      
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
