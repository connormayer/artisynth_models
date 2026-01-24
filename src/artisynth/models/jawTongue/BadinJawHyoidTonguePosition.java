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
   
   // Enable/disable this flag to show the probe positions in the terminal while running
   // simulations.
   private boolean enableVerboseTracking = false; 

   // Declare midline visibility flag and threshold
   private boolean showMidline = true;

   public static PropertyList myProps =
      new PropertyList(BadinJawHyoidTonguePosition.class, BadinJawHyoidTongue.class);


   public BadinJawHyoidTonguePosition () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      // Hard coded probe coordinates (x, y, z, name)
      Object[][] probeCoordinates = {
         // Row 0
         {125.208, -22.326, 96.669, "row0_1"},   
         {127.715, 8.618, 97.165, "row0_2"},     
         {126.055, 19.467, 96.218, "row0_3"},    

         // Row 1
         {112.586, -21.622, 104.283, "row1_1"},  
         {116.284, -17.005, 108.140, "row1_2"},  
         {115.573, -7.582, 109.644, "row1_3"},   
         {114.390, 0.000, 110.271, "row1_4"},    
         {115.572, 7.582, 109.644, "row1_5"},    
         {116.284, 17.005, 108.140, "row1_6"},  
         {112.586, 21.622, 104.283, "row1_7"},   

         // Row 2
         {96.333, -18.824, 108.686, "row2_1"},  
         {96.753, -13.860, 113.322, "row2_2"},   
         {97.584, -6.610, 113.977, "row2_3"},   
         {96.811, 0.000, 113.659, "row2_4"},    
         {97.584, 6.610, 113.977, "row2_5"},  
         {96.753, 13.860, 113.322, "row2_6"},  
         {96.333, 18.824, 108.686, "row2_7"},  

         // Row 3
         {81.016, -16.360, 98.083, "row3_1"},   
         {78.589, -15.568, 106.734, "row3_2"},  
         {78.823, -9.747, 111.026, "row3_3"},   
         {79.228, -4.108, 111.550, "row3_4"},   
         {79.228, 4.108, 111.550, "row3_5"},   
         {78.823, 9.747, 111.026, "row3_6"},     
         {78.589, 15.568, 106.734, "row3_7"},    
         {81.016, 16.360, 98.083, "row3_8"},     

         // Row 4
         {71.403, -13.947, 97.069, "row4_1"},  
         {67.596, -12.053, 103.382, "row4_2"},  
         {66.528, -2.895, 106.215, "row4_3"},    
         {66.528, 2.895, 106.215, "row4_4"},   
         {67.596, 12.053, 103.382, "row4_5"},   
         {71.403, 13.947, 97.069, "row4_6"}, 
         
         // Row 5
         {60.962, -9.876, 99.233, "row5_1"},     
         {60.037, -5.952, 100.075, "row5_2"},   
         {59.829, -2.571, 100.222, "row5_3"},  
         {59.574, 0.000, 99.916, "row5_4"},    
         {59.829, 2.571, 100.222, "row5_5"},    
         {60.037, 5.952, 100.075, "row5_6"},    
         {60.962, 9.876, 99.233, "row5_7"},  

         // Row 6
         {59.023, -5.932, 95.639, "row6_1"},    
         {59.314, 0.000, 94.640, "row6_2"},     
         {59.023, 5.932, 95.639, "row6_3"},  
      };

      // Create markers using the data structure defined above
      for (int i = 0; i < probeCoordinates.length; i++) {
         double x = (Double) probeCoordinates[i][0];
         double y = (Double) probeCoordinates[i][1];
         double z = (Double) probeCoordinates[i][2];
         String name = (String) probeCoordinates[i][3];
         
         Point3d pos = new Point3d(x, y, z);
         FemMarker marker = new FemMarker(pos);
         marker.setName(name);
         RenderProps.setSphericalPoints(marker, 2, Color.ORANGE);
         tongue.addMarker(marker);
         probeMarkers.add(marker);
      }

      // Hide markers based on the showMidline flag
      if (!showMidline) {
         final double SIDE_Y = 12.0;       // approximate y of the two side columns (mm)
         final double SIDE_THRESH = 3.0;   // how close to SIDE_Y to treat as "that column"
         final double CENTER_KEEP = 3.0;   // keep any marker with |y| <= CENTER_KEEP (center column)
         for (FemMarker m : probeMarkers) {
            double y = m.getPosition().y;
            if (Math.abs(y) <= CENTER_KEEP) {
               // keep the center column visible
               RenderProps.setVisible(m, true);
            }
            else if (Math.abs(Math.abs(y) - SIDE_Y) <= SIDE_THRESH) {
               // hide the two side columns
               RenderProps.setVisible(m, false);
            }
         }
      }
      
      // Output the x, y, z coordinates of all probes once after placement
      for (FemMarker marker : probeMarkers) {
         Point3d pos = marker.getPosition();
         System.out.printf("%s: %.3f %.3f %.3f%n", marker.getName(), pos.x, pos.y, pos.z);
      }
      
      // This whole block periodically prints the probe positions to the terminal
      if (enableVerboseTracking) {
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
      }

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
