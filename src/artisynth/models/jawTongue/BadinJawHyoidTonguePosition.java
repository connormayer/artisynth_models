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

   public static PropertyList myProps =
      new PropertyList(BadinJawHyoidTonguePosition.class, BadinJawHyoidTongue.class);


   public BadinJawHyoidTonguePosition () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      FemMarker mkr = new FemMarker (-5,0, 145);
      RenderProps.setSphericalPoints (mkr,  2,  Color.BLUE);
      tongue.addMarker (mkr);
      
      Timer timer = new Timer();
      
      TimerTask task = new TimerTask() {
         public void run() {
            Point3d tipPos = mkr.getPosition ();
            System.out.println("pos: " + tipPos);
         }
      };
      timer.scheduleAtFixedRate (task, 0, 10000);
      
//      NumericOutputProbe mkrProbe =
//      new NumericOutputProbe ( //not sure how to format this
//        tongue, "markers/0:position", PathFinder.getSourceRelativePath (this, "PositionMkr.txt"), 0.01);
//      mkrProbe.setName("FemMarker Position");
//      mkrProbe.setDefaultDisplayRange (-4, 4);
//      mkrProbe.setStopTime (10);
//      addOutputProbe (mkrProbe);

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
