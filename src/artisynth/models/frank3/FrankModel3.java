package artisynth.models.frank3;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.L2RegularizationTerm;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearFrameMaterial;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.FrameFrameAttachment;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpringList;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.mechmodels.SphericalJoint;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.frank2.FrankModel2;
import artisynth.models.frank2.GenericModel;
import artisynth.models.jawTongue.AirwaySkin;
import artisynth.models.jawTongue.StaticJawHyoidTongue;
import artisynth.models.modelOrderReduction.PrintData;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.neckModel.InterveterbralJointFM;
import artisynth.models.neckModel.MasoudMillardLAMExt;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.ColorMapProps;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;

public class FrankModel3 extends FrankModel2 {

   // neck rigid body and joint
   RenderableComponentList<RigidBody> neckRbs = new RenderableComponentList<RigidBody> (RigidBody.class, "CervicalSpine", "NS");
   RenderableComponentList<RigidBody> rbs;
   RenderableComponentList<SphericalJoint> neckSjs = new RenderableComponentList<SphericalJoint> (SphericalJoint.class, "NeckSphericalJoints", "NSjs");
   RenderableComponentList<FrameSpring> neckFss = new RenderableComponentList<FrameSpring> (FrameSpring.class, "NeckFrameSprings", "NFss");;
   RenderableComponentList<MuscleBundle> externalMuscles;

   // neck muscles
   public ComponentList<MultiPointSpringList> muscleList = new ComponentList<MultiPointSpringList>(MultiPointSpringList.class, "NeckSpinalMuscles", "SMs");
   public MultiPointSpringList<MultiPointMuscle> longusColli = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "longusColli", "LCo");
   public MultiPointSpringList<MultiPointMuscle> longusCapitis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "longusCapitis", "LCa");
   public MultiPointSpringList<MultiPointMuscle> subOccipital = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "subOccipital", "SO");
   public MultiPointSpringList<MultiPointMuscle> semispinalisCapitis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "semispinalisCapitis", "SsCa");
   public MultiPointSpringList<MultiPointMuscle> semispinalisCervicis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "semispinalisCervicis", "SsCe");
   public MultiPointSpringList<MultiPointMuscle> levatorScapulae = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "levatorScapulae", "LS");
   public MultiPointSpringList<MultiPointMuscle> multifidus = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "multifidus", "M");
   public MultiPointSpringList<MultiPointMuscle> scalenus = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "scalenus", "S");
   public MultiPointSpringList<MultiPointMuscle> spleniusCapitis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "spleniusCapitis", "SCa");
   public MultiPointSpringList<MultiPointMuscle> spleniusCervicis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "spleniusCervicis", "SCe");
   public MultiPointSpringList<MultiPointMuscle> sternocleidomastoid = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "sternocleidomastoid", "Sc");
   public MultiPointSpringList<MultiPointMuscle> trapezius = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "trapezius", "T");
   public MultiPointSpringList<MultiPointMuscle> longissimusCervicis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "longissimusCervicis", "LCe");
   public MultiPointSpringList<MultiPointMuscle> longissimusCapitis = new MultiPointSpringList<MultiPointMuscle>(MultiPointMuscle.class, "longissimusCapitis", "LCa");
   int myMusNum = 116;
   int myMusGroupNum = 14;
   public double myMusForSca = 0.6;
   public ComponentList<MuscleExciter> muscleExciter = new ComponentList<MuscleExciter>(MuscleExciter.class, "Exciter", "Ex");
   boolean saveEx = false;
   boolean loadEx = false;

   // inverse simulation
   RenderableComponentList<FrameMarker> tMrList = new RenderableComponentList<FrameMarker>(FrameMarker.class, "targetMarkers", "tm");
   TrackingController myTrackingController = new TrackingController(mechModel, "tcon");
   public double simDuration = 5.0;

   // control panel
   public ControlPanel alignmentPanel;
   public boolean saveGeo = false;

   /** When true, skip GUI panels and textured rendering (set via -Dartisynth.batchsim=true). */
   protected boolean batchMode =
      Boolean.getBoolean ("artisynth.batchsim");

   String workingPath = ArtisynthPath.getSrcRelativePath (this.getClass(), "");
   String geometryPath = workingPath + "/geometry";
   String dataPath = workingPath + "/data";
   String neckDataPath = dataPath + "/neckModel";

   RigidBody jaw;
   RigidBody maxilla;
   RigidBody cranium;
   RenderableComponentList<FemMuscleModel> fems;
   AirwaySkin airwaySkin;


   @Override
   protected void buildControlPanels() {
      if (batchMode) {
         return;
      }
      super.buildControlPanels();
   }

   public void build (String [] args) throws IOException {
      super.build (args);
      rbs = (RenderableComponentList<RigidBody>)mechModel.get ("RigidBodies");
      externalMuscles = (RenderableComponentList<MuscleBundle>)mechModel.get ("ExternalMuscles");
      
      addHeadNeck();
      neckRbs.get ("C8").setDynamic (false);

      // control panel
      if (!batchMode) {
         alignmentPanel = new ControlPanel("Alignment");
         alignmentPanel.addWidget (this, "saveGeometry");
         alignmentPanel.addWidget (this, "saveMuscleExcitation");
         alignmentPanel.addWidget (this, "loadMuscleExictation");
         //this.addControlPanel(alignmentPanel);
      }

      // inverse simulation
      //createTargetList();
      //writeTargetList();
      //addTrackingController();
      //createInputProbeForNuetralPosture("/data/neckModel");

      // attach to frank
      jaw = rbs.get ("jaw");
      maxilla = rbs.get ("maxilla");
      cranium = rbs.get ("cranium");
      fems = (RenderableComponentList<FemMuscleModel>)mechModel.get("DeformableBodies");

      attachPharynxNeck ();
      //renderNonDynNodes();
      setNodeDym();
      setFrankSkull();
      //defineJawContraints();
      setCollisionBehavior();
      face.getRenderProps ().setVisible (true);
      if (!batchMode) {
         renderLipTexture();
      }
      renderNonDynNodes();
      //recursivelyPrintComponents (this);
   }

   /**
    * Recursively descends through a model, using depth first search, and
    * prints the types and path names of components.
    */
   public static void recursivelyPrintComponents (CompositeComponent comp) {
      for (int i=0; i<comp.numComponents(); i++) {
         ModelComponent mc = comp.get(i);
         String pathName = ComponentUtils.getPathName(mc);

         // First, explicitly check for FemModel3d, RigidBody, and SkinMeshBody
         // because these are also CompositeComponents that we *don't* want to
         // recurse into:
         if (mc instanceof FemModel3d) {
            System.out.println ("FemModel3d: " + pathName);
         }
         else if (mc instanceof RigidBody) {
            System.out.println ("RigidBody: " + pathName);
         }
         else if (mc instanceof SkinMeshBody) {
            System.out.println ("SkinMeshBody: " + pathName);
         }
         // Otherwise, if the component is a CompositeComponent, recursively
         // descend into it:
         else if (mc instanceof CompositeComponent) {
            recursivelyPrintComponents ((CompositeComponent)mc);
         }
         // Finally, if the component is not a composite, print its simple class
         // name and path name:
         else {
            System.out.println (mc.getClass().getSimpleName() + ": " + pathName);
         }
      }
   }         
   
   public void photoShoot(double time)
   {
      mechModel.setDynamicsEnabled(false);

      double tEnd = 10.0;
      double tRot = 0.25;
      double dist = 0.75;
      double tRotMod = (time % tRot) * 1.0/tRot;
      Point3d pCenter = new Point3d(0.12, 0.0, 0.07);
      Point3d pCamera = new Point3d(pCenter.x - dist*Math.cos( Math.PI/2.0 - tRotMod*Math.PI*2.0), pCenter.y - dist*Math.sin(Math.PI/2.0 - tRotMod*Math.PI*2.0), pCenter.z);
      if (time/tRot >= 5.25) {
         return;
      }
      this.setViewerCenter(pCenter);
      this.setViewerEye(pCamera);
      for (FemNode3d node : softPalate.getNodes ()) {
         if (node.numAdjacentElements () == 0) {
            RenderProps.setVisible (node, false);
         }
      }
      for (FemNode3d node : pharynx.getNodes ()) {
         RenderProps.setVisible (node, false);
      }
      
      double alpha = 1.0;
      if (time > 0.0) {
         double base = (time % tRot)/tRot;
         if ((base) < 0.15) {
            alpha = 1.0 - (base)/0.3;
         }
         else {
            alpha = 0.0;
         }
      }

      if (alpha <= 0) {
         if (time/tRot >= 1.0)
         {
            face.getRenderProps().setVisible(false);
            face.getMuscleBundles().get("OOP").getRenderProps().setVisible(false);
            cranium.getRenderProps ().setVisible(false);
            externalMuscles.getRenderProps().setVisible(false);
         }
         if (time/tRot >= 2.0)
         {
            //jaw.getRenderProps().setVisible(false);
            maxilla.getRenderProps().setVisible(false);
            for (RigidBody rb : neckRbs) {
               rb.getRenderProps ().setVisible(false);
            }
            for (MultiPointSpringList mus : muscleList) {
               mus.getRenderProps ().setVisible(false);
            }
         }
         if (time/tRot >= 3.0)
         {
            //jaw.getRenderProps().setAlpha (1.5-alpha);
            pharynx.getRenderProps().setVisible(false);
            airwaySkin.getRenderProps ().setVisible(false);
            softPalate.getMuscleBundles ().getRenderProps().setVisible(false);
         }
         if (time/tRot >= 4.0) {
            //tongue.getRenderProps ().setVisible (false);
            larynx.getRenderProps ().setVisible(false);
            softPalate.getRenderProps ().setVisible(false);
         }
         if (time/tRot >= 5.0) {
            face.getRenderProps().setVisible(true);
            face.getMuscleBundles().get("OOP").getRenderProps().setVisible(true);
            cranium.getRenderProps ().setVisible(true);
            externalMuscles.getRenderProps().setVisible(true);
            jaw.getRenderProps().setVisible(true);
            maxilla.getRenderProps().setVisible(true);
            for (RigidBody rb : neckRbs) {
               rb.getRenderProps ().setVisible(true);
            }
            for (MultiPointSpringList mus : muscleList) {
               mus.getRenderProps ().setVisible(true);
            }
            pharynx.getRenderProps().setVisible(true);
            softPalate.getMuscleBundles ().getRenderProps().setVisible(true);
            larynx.getRenderProps ().setVisible(true);
         }
      }
      else {
         if (time/tRot >= 0.0) {
            face.getRenderProps().setAlpha(1.5-alpha);
            face.getMuscleBundles().get("OOP").getRenderProps().setAlpha(1.5-alpha);
            cranium.getRenderProps ().setAlpha(1.5-alpha);
            externalMuscles.getRenderProps().setAlpha(1.5-alpha);
            //jaw.getRenderProps().setAlpha(1.5-alpha);
            maxilla.getRenderProps().setAlpha(1.5-alpha);
            for (RigidBody rb : neckRbs) {
               rb.getRenderProps ().setAlpha(1.5-alpha);
            }
            for (MultiPointSpringList mus : muscleList) {
               mus.getRenderProps ().setAlpha(1.5-alpha);
            }
            pharynx.getRenderProps().setAlpha(1.5-alpha);
            softPalate.getMuscleBundles ().getRenderProps().setAlpha(1.5-alpha);
            larynx.getRenderProps ().setAlpha(1.5-alpha);
            this.airwaySkin.getRenderProps ().setAlpha (1.5 - alpha);
            
         }
         if (time/tRot >= 1.0)
         {
            face.getRenderProps().setAlpha(alpha);
            face.getMuscleBundles().get("OOP").getRenderProps().setAlpha(alpha);
            cranium.getRenderProps ().setAlpha(alpha);
            externalMuscles.getRenderProps().setAlpha(alpha);
         }
         if (time/tRot >= 2.0)
         {
            //jaw.getRenderProps().setAlpha(alpha);
            maxilla.getRenderProps().setAlpha(alpha);
            for (RigidBody rb : neckRbs) {
               rb.getRenderProps ().setAlpha(alpha);
            }
            for (MultiPointSpringList mus : muscleList) {
               mus.getRenderProps ().setAlpha(alpha);
            }
         }
         if (time/tRot >= 3.0)
         {
            //jaw.getRenderProps().setAlpha (1.5-alpha);
            pharynx.getRenderProps().setAlpha(alpha);
            airwaySkin.getRenderProps ().setAlpha(alpha);
            softPalate.getMuscleBundles ().getRenderProps().setAlpha(alpha);
         }
         if (time/tRot >= 4.0) {
            //tongue.getRenderProps ().setVisible (false);
            larynx.getRenderProps ().setAlpha(alpha);
            softPalate.getRenderProps ().setAlpha(alpha);
         }
         if (time/tRot > 5.0) {
            face.getRenderProps().setAlpha(1.5-alpha);
            face.getMuscleBundles().get("OOP").getRenderProps().setAlpha(1.5-alpha);
            cranium.getRenderProps ().setAlpha(1.5-alpha);
            externalMuscles.getRenderProps().setAlpha(1.5-alpha);
            jaw.getRenderProps().setAlpha(1.5-alpha);
            maxilla.getRenderProps().setAlpha(1.5-alpha);
            for (RigidBody rb : neckRbs) {
               rb.getRenderProps ().setAlpha(1.5-alpha);
            }
            for (MultiPointSpringList mus : muscleList) {
               mus.getRenderProps ().setAlpha(1.5-alpha);
            }
            pharynx.getRenderProps().setAlpha(1.5-alpha);
            softPalate.getMuscleBundles ().getRenderProps().setAlpha(1.5-alpha);
            larynx.getRenderProps ().setAlpha(1.5-alpha);
         }
      }
      
         
   }

   //Property list
   public static PropertyList myProps = new PropertyList (
      FrankModel3.class, FrankModel2.class);

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add ("targetPosition", "The position of target frame markers", new double[99]);
      myProps.add ("saveGeometry", "save geometries", false);
      myProps.add ("saveMuscleExcitation", "save muscle excitations", false);
      myProps.add ("loadMuscleExictation", "load muscle excitations", false);
      myProps.add ("LRExcitation", "muscle excitations", new double [28]);
   }

   public void setTargetPosition(double [] value) {
      double [] temPos = value;
      int i = 0;
      for (FrameMarker tMr : tMrList) {
         tMr.setPosition(temPos[i*3], temPos[i*3+1], temPos[i*3+2]);
         i += 1;
      }
   }
   public double [] getTargetPosition() {
      double [] temPos = new double [99];
      int i = 0;
      for (FrameMarker tMr : tMrList){
         temPos[i*3] = tMr.getPosition ().x;
         temPos[i*3+1] = tMr.getPosition ().y;
         temPos[i*3+2] = tMr.getPosition ().z;
         i += 1;
      }
      return temPos;
   }

   public void setSaveGeometry(boolean listener) {
      saveGeo = listener;
      try {
         if (saveGeo) {
            String meshPath =
            ArtisynthPath.getSrcRelativePath (
               this.getClass (), "/data/");
            for (int i = 0; i < neckRbs.size (); i++) {
               File fp = new File(meshPath + "C" + i + ".obj");
               neckRbs.get (8-i).getMesh ().write (fp, "%.8g", false);
            }
            writeBoneInfo();
            //writeJointInfo();
            writeLinearJointInfo();
            writeMuscleInfo();
            writeMuscleName();
            writeMusclePoints();
            writeMuscleAttachName();

            saveGeo = false;
         }
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public boolean getSaveGeometry() {
      return saveGeo;
   }

   public void setSaveMuscleExcitation(boolean listener) {
      saveEx = listener;
      if (saveEx) {
         try {
            writeMuscleExcitations("/data/neckModel");
         }
         catch (IOException e) {
            e.printStackTrace();
         }
         saveEx = false;
      }
   }

   public boolean getSaveMuscleExcitation() {
      return saveEx;
   }

   public void setLoadMuscleExictation(boolean listener) {
      loadEx = listener;
      if (loadEx) {
         loadMuscleExcitation("/data/neckModel");
         loadEx = false;
      }
   }

   public boolean getLoadMuscleExictation() {
      return loadEx;
   }

   public void setLRExcitation (double [] Exs) {
      int idx = 0;
      for (MuscleExciter musEx : muscleExciter) {
         if (musEx.getName ().contains ("_l") || musEx.getName ().contains ("_r")) {
            musEx.setExcitation (Exs[idx++]);
         }
      }

   }

   public double [] getLRExcitation () {
      double [] Exs  = new double [myMusGroupNum * 2];
      int idx = 0;
      for (MuscleExciter musEx : muscleExciter) {
         if (musEx.getName ().contains ("_l") || musEx.getName ().contains ("_r")) {
            Exs[idx++] = musEx.getExcitation ();
         }
      }
      return Exs;
   }
   
   public void printMuscleLayout() throws FileNotFoundException {
      
      class MuscleInfo {
         String name;
         ArrayList<Point> points = new ArrayList<>();
         ArrayList<int[]> connections = new ArrayList<>();
      }
      
      ArrayList<MuscleInfo> muscles = new ArrayList<>(2*muscleList.size());
      
      for (MultiPointSpringList<MultiPointMuscle> muscle : muscleList) {
         String muscleName = muscle.getName();
         
         System.out.println("Muscle: " + muscleName);
         MuscleInfo left = new MuscleInfo();
         left.name = muscleName.substring(0,1).toUpperCase() + muscleName.substring(1) + "_L";
         MuscleInfo right = new MuscleInfo();
         right.name = muscleName.substring(0,1).toUpperCase() + muscleName.substring(1) + "_R";
         
         for (MultiPointMuscle mpm : muscle) {
            String mpmName = mpm.getName();
            String subName = mpmName.substring(muscleName.length()+1);
            
            System.out.println("  component: " + subName);
            MuscleInfo minfo = right;
            if (subName.startsWith("l")) {
               minfo = left;
            }
            
            // points and connections
            int pidx = minfo.points.size();
            int[] connections = new int[mpm.numPoints()];
            for (int i=0; i<mpm.numPoints(); ++i) {
               Point pnt = mpm.getPoint(i);
               minfo.points.add(pnt);
               connections[i] = pidx;
               ++pidx;
               DynamicAttachment attach = pnt.getAttachment();
               String attached = attach.getMasters()[0].getName();
               Point3d pos = pnt.getPosition();
               System.out.println("    " + pos.toString() + ", " + attached);
            }
            minfo.connections.add(connections);
         }
         
         muscles.add(left);
         muscles.add(right);
      }
      
      for (MuscleInfo minfo : muscles) {
         
         File outFile = ArtisynthPath.getSrcRelativeFile(this, "geometry/NeckMuscles/" + minfo.name + ".txt");
         PrintWriter writer = new PrintWriter(outFile);
         Date dNow = new Date();
         SimpleDateFormat ft = new SimpleDateFormat ("yyyy.MM.dd'_'HH:mm:ss zzz");
         writer.printf("# Description of *%s* muscle bundle. Created on %s. \n\n", minfo.name, ft.format(dNow));
         writer.println("points");
         int pidx = 0;
         for (Point p : minfo.points) {
            writer.printf("%d, %f, %f, %f", pidx, p.getPosition().x, p.getPosition().y, p.getPosition().z);
            
            DynamicAttachment attach = p.getAttachment();
            String attached = attach.getMasters()[0].getName();
            if (attached.equals("C0")) {
               attached = "cranium";
            } else if (attached.equals("C8")) {
               attached = "base";
            }
            writer.printf(", type=FrameMarker, attach=%s\n", attached);
            
            ++pidx;
         }
         writer.println("\nconnections");
         for (int[] conn : minfo.connections) {
            for (int i=0; i<conn.length; ++i) {
               if (i > 0) {
                  writer.print(", ");
               } 
               writer.print(conn[i]);
            }
            writer.println();
         }
         
         writer.close();
         
      }
      
   }


   //--------------------------------------------------
   // read model from file
   //--------------------------------------------------

   public void addHeadNeck() {
      try {
         addHeadNeckBone();
         addHeadNeckJoint();
         addHeadNeckMuscle();
         addMuscleExciter();
         
         // printMuscleLayout();
      }
      catch (IOException e) {
         System.out.println ("addHeadNeck: failed to load model from file!");
         e.printStackTrace();
      }
   }


   //--------------------------------------------------
   // Rigid body
   //--------------------------------------------------

   public void addHeadNeckBone() throws IOException {
      int idx = 8;
      MatrixNd rbInfo = new MatrixNd ();
      ReadWrite.readMatrix (rbInfo, this.getClass (), "/data/neckModel/neckRbInfo.txt"); 

      RigidTransform3d pose = new RigidTransform3d();
      for (int i = 0; i <= 8; i++) {

         int InfoIdx = 0; 
         RigidBody rb = loadRigidBody("C"+idx, "/C"+idx+".obj");

         // pose
         for (int j = 0; j < 3; j++) {
            pose.p.set (j, rbInfo.get (i, InfoIdx++));
         }
         Vector3d axis = new Vector3d();
         for (int j = 0; j< 3; j++) {
            axis.set (j, rbInfo.get (i, InfoIdx++));
         }
         AxisAngle aA = new AxisAngle();
         aA.set (axis, rbInfo.get (i, InfoIdx++));
         pose.R.setAxisAngle (aA);
         rb.setPose (pose);

         // center of mass
         Point3d com = new Point3d();
         for (int j = 0; j < 3; j++) {
            com.set (j, rbInfo.get (i, InfoIdx++));
         }
         rb.setCenterOfMass (com);

         // mass and inertia
         rb.setMass (rbInfo.get (i, InfoIdx++));
         SymmetricMatrix3d rJ = new SymmetricMatrix3d();
         for (int j = 0; j< 3; j++) {
            rJ.set (j, j, rbInfo.get (i, InfoIdx++));
         }
         rb.setRotationalInertia (rJ);

         neckRbs.add (rb);
         //mechModel.addRigidBody (rb);
         idx--;
      }

      // Bones
      RenderProps rbRenderProps = new RenderProps ();
      rbRenderProps.setFaceColor (new Color (238, 232, 170));
      rbRenderProps.setFaceStyle (Renderer.FaceStyle.FRONT_AND_BACK);
      rbRenderProps.setAlpha (1);
      rbRenderProps.setShading (Shading.SMOOTH);
      //rbRenderProps.setVisible (true);
      for (RigidBody rbb : neckRbs) {
         rbb.setRenderProps (rbRenderProps);
      }

      mechModel.add (neckRbs);
   }

   private RigidBody loadRigidBody(String name, String fileName) {
      PolygonalMesh geom = GenericModel.loadGeometry(geometryPath, fileName);
      RigidBody rb;
      if (geom == null)
         rb = null;
      else
      {
         rb = new RigidBody();
         rb.setName(name);
         rb.setMesh(geom, null);
         rb.setDensity(0);
      }

      return rb;
   }

   //--------------------------------------------------
   // Joint
   //--------------------------------------------------

   public void addHeadNeckJoint() throws IOException {
      MatrixNd jtInfo = new MatrixNd ();
      ReadWrite.readMatrix (jtInfo, this.getClass (), "/data/neckModel/neckLinearJointInfo.txt");

      int idx = 8;
      RigidTransform3d pose1 = new RigidTransform3d ();
      RigidTransform3d pose2 = new RigidTransform3d ();
      for (int i = 0; i < jtInfo.rowSize (); i++) {

         int InfoIdx = 0;
         FrameSpring spring; 
         SphericalJoint joint;
         //InterveterbralJointFM jointMat = new InterveterbralJointFM();
         LinearFrameMaterial jointMat = new LinearFrameMaterial();

         // pose
         for (int j = 0; j < 3; j++) {
            pose1.p.set (j, jtInfo.get (i, InfoIdx++));
         }
         Vector3d axis = new Vector3d();
         for (int j = 0; j< 3; j++) {
            axis.set (j, jtInfo.get (i, InfoIdx++));
         }
         AxisAngle aA = new AxisAngle();
         aA.set (axis, jtInfo.get (i, InfoIdx++));
         pose1.R.setAxisAngle (aA);

         for (int j = 0; j < 3; j++) {
            pose2.p.set (j, jtInfo.get (i, InfoIdx++));
         }
         for (int j = 0; j< 3; j++) {
            axis.set (j, jtInfo.get (i, InfoIdx++));
         }
         aA.set (axis, jtInfo.get (i, InfoIdx++));
         pose2.R.setAxisAngle (aA);

         // make joint
         joint = new SphericalJoint();
         joint.setName ("C"+idx+"C"+Integer.toString (idx-1));
         joint.setBodies (neckRbs.get (i+1), neckRbs.get (i), pose1, pose2);
         spring = new FrameSpring("C"+idx+"C"+Integer.toString (idx-1));
         //spring.setFrames (neckRbs.get (i+1), neckRbs.get (i), pose);
         pose1.mulInverseLeft (neckRbs.get (i+1).getPose (), pose1);
         pose2.mulInverseLeft (neckRbs.get (i).getPose (), pose2);
         spring.setFrames (neckRbs.get (i+1), pose1, neckRbs.get (i), pose2);

         InfoIdx = makeJointMat(jointMat, jtInfo, i, InfoIdx);

         // set material
         spring.setMaterial (jointMat);

         neckSjs.add (joint);
         neckFss.add (spring);
         mechModel.addBodyConnector (joint);
         mechModel.addFrameSpring (spring);

         idx--;
      }
   }

   public int makeJointMat(InterveterbralJointFM jointMat, MatrixNd jtInfo, int jointIdx, int InfoIdx) {

      // FEPoly
      double [] FE = new double [4];
      for (int j = 0; j < 4; j++) {
         FE[j] = jtInfo.get(jointIdx, InfoIdx++);
      }
      jointMat.setFEPoly (FE);
      // LBPoly
      double [] LB = new double [4];
      for (int j = 0; j < 4; j++) {
         LB[j] = jtInfo.get(jointIdx, InfoIdx++);
      }
      jointMat.setLBPoly (LB);
      // ARPoly
      double [] AR = new double [4];
      for (int j = 0; j < 4; j++) {
         AR[j] = jtInfo.get(jointIdx, InfoIdx++);
      }
      jointMat.setARPoly (AR);

      return InfoIdx;
   }

   public int makeJointMat(LinearFrameMaterial jointMat, MatrixNd jtInfo, int jointIdx, int InfoIdx) {

      Vector3d rK = new Vector3d ();
      for (int j = 0; j < 3; j++) {
         rK.set (j, jtInfo.get (jointIdx, InfoIdx++));
      }
      jointMat.setRotaryStiffness (rK);

      Vector3d rD = new Vector3d ();
      for (int j = 0; j < 3; j++) {
         rD.set (j, jtInfo.get (jointIdx, InfoIdx++));
      }
      jointMat.setRotaryDamping (rD);

      jointMat.setStiffness (0);
      jointMat.setDamping (0);

      return InfoIdx;
   }

   //--------------------------------------------------
   // Muscle
   //--------------------------------------------------
   public void addHeadNeckMuscle() throws IOException {
      MatrixNd musInfo = new MatrixNd ();
      ReadWrite.readMatrix (musInfo, getClass(), "/data/neckModel/neckMuscleInfo.txt");
      String [] musNames = ReadWrite.readStringArray (getClass(), "/data/neckModel/neckMuscleName.txt");
      double [][] musPoints = ReadWrite.readArray (getClass(), "/data/neckModel/neckMusclePoint.txt");
      String [][] musPntAttName = ReadWrite.readStringDoubleArray (getClass(), "/data/neckModel/neckMusclePointAttachName.txt");

      muscleList.add (longusColli);
      muscleList.add (longusCapitis);
      muscleList.add (subOccipital);
      muscleList.add (semispinalisCapitis);
      muscleList.add (semispinalisCervicis);
      muscleList.add (levatorScapulae);
      muscleList.add (multifidus);
      muscleList.add (scalenus);
      muscleList.add (spleniusCapitis);
      muscleList.add (spleniusCervicis);
      muscleList.add (sternocleidomastoid);
      muscleList.add (trapezius);
      muscleList.add (longissimusCervicis);
      muscleList.add (longissimusCapitis);
      mechModel.add (muscleList);

      for (int i = 0 /*mus idx*/; i < musInfo.rowSize (); i++) {
         // muscle Name
         MultiPointMuscle mus = new MultiPointMuscle();
         String musName = musNames[i];
         mus.setName (musName);
         String musGroupName = musName.split ("_")[0];


         // muscle material parameters
         double [] matPar = new double [musInfo.colSize ()-1];
         for (int j = 0; j < matPar.length; j++) {
            matPar[j] = musInfo.get (i, j);
         }
         MasoudMillardLAMExt musMat = new MasoudMillardLAMExt();
         musMat.setMatrialParameters (matPar);
         musMat.setForceScaling (myMusForSca);
         mus.setMaterial (musMat);


         // muscle point 
         int pntNo =  musPoints[i].length/3;
         int idx = 0; // pnt data idx
         for (int j/*pnt idx*/ = 0; j < pntNo; j++) {
            Point3d pnt3d = new Point3d();
            for (int k = 0; k < 3; k++) {
               pnt3d.set (k, musPoints[i][idx++]);
            }
            Point pnt = new Point(pnt3d);
            mus.addPoint (pnt);
         }

         // set attachment
         String [] attNames = musPntAttName[i];
         for (int j = 0; j < pntNo; j ++) {
            mus.getPoint (j).setName (attNames[j] + "_" + mus.getName ());
            mechModel.addPoint (mus.getPoint (j));
            mechModel.attachPoint (mus.getPoint (j), neckRbs.get (attNames[j]));
         }

         // set rest length
         double restLen = musInfo.get (i, musInfo.colSize ()-1);
         mus.setRestLength (restLen);

         // add into muscle group
         muscleList.get (musGroupName).add (mus);
      }

      // Muscles
      RenderProps musRenderProps = new RenderProps ();
      musRenderProps = new RenderProps ();
      musRenderProps.setLineStyle (Renderer.LineStyle.SPINDLE);
      musRenderProps.setLineColor (new Color (255,175,190));
      musRenderProps.setLineRadius (0.001);
      for ( MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         musList.setRenderProps (musRenderProps);
         for( MultiPointMuscle mus : musList) {
            mus.setExcitationColor (Color.RED);
         }
      }
   }

   public void addMuscleExciter() {
      MuscleExciter musExcL;
      MuscleExciter musExcR;
      MuscleExciter musEx;

      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         //left-right
         musExcL = new MuscleExciter(musList.getName()+"_l");
         musExcR = new MuscleExciter(musList.getName()+"_r");
         muscleExciter.add (musExcL);
         muscleExciter.add (musExcR);
         //kind
         musEx = new MuscleExciter(musList.getName());
         muscleExciter.add (musEx);

         for (MultiPointMuscle mus : musList) { 
            //left-right
            if(mus.getName ().contains ("_l")) {      
               musExcL.addTarget (mus, 1.0);
            }
            else if(mus.getName().contains ("_r")){
               musExcR.addTarget (mus, 1.0);
            }
            //kind
            musEx.addTarget (mus, 1.0);
         }
      }
      mechModel.add (muscleExciter);
   }

   public void loadMuscleExcitation(String filePath) {
      VectorNd musExVec = new VectorNd (myMusNum);
      VectorNd ExVec = new VectorNd(muscleList.size ());
      VectorNd ExLRVec = new VectorNd(muscleList.size ()*2);

      try {
         ReadWrite.readVector (musExVec, getClass(), filePath + "/muscleExcitation.txt");
         ReadWrite.readVector (ExVec, getClass(), filePath + "/groupExcitation.txt");
         ReadWrite.readVector (ExLRVec, getClass(), filePath + "/groupLRExcitation.txt");
      }
      catch (IOException e) {
         System.out.println ("failed to load muscle excitations from file !");
         e.printStackTrace();
      }

      int idx = 0;
      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         for (MultiPointMuscle mus : musList) { 
            mus.setExcitation (musExVec.get (idx++));
         }
      }

      idx = 0; 
      int exIdx = 0;
      for (MuscleExciter ex: muscleExciter) {
         if (ex.getName ().contains ("_l") || ex.getName ().contains ("_r")) {
            ex.setExcitation (ExLRVec.get (idx++));
         } else {
            ex.setExcitation (ExVec.get (exIdx++));
         }
      }

   }

   //----------------------------------------------------------------------------------------------
   // write rigidBody information
   //----------------------------------------------------------------------------------------------
   public void writeHeadNeck() {
      try {
         writeBoneInfo();
         //writeJointInfo();
         writeLinearJointInfo();
         writeMuscleInfo();
         writeMuscleName();
         writeMusclePoints();
         writeMuscleAttachName();
      }
      catch (IOException e) {
         System.out.println ("writeHeadNeck: failed to wirte head and neck model!");
         e.printStackTrace();
      }

   }


   //-------------------------------------------
   // write rigidBody information
   //-------------------------------------------

   /**
    * info for bones :
    * position - 3
    * axis and angle - 4
    * center of mass in local coordinate - 3
    * mass - 1
    * inertia -3
    * @throws IOException
    */
   public void writeBoneInfo() throws IOException {
      MatrixNd rbInfo = new MatrixNd (neckRbs.size (), 14);

      for (int i = 0; i < neckRbs.size (); i++) {
         int idx = 0;
         RigidBody rb = neckRbs.get (i);
         // pose
         for (int j = 0; j < 3; j++) {
            rbInfo.set (i, idx++, rb.getPose ().p.get (j));
         }

         for (int j = 0; j < 3; j++) {
            rbInfo.set (i, idx++, rb.getPose ().R.getAxisAngle ().axis.get (j));
         }
         rbInfo.set (i, idx++, rb.getPose ().R.getAxisAngle ().angle);

         // center of mass
         for (int j = 0; j < 3; j++) {
            rbInfo.set (i, idx++, rb.getCenterOfMass ().get (j));
         }

         // mass
         rbInfo.set (i, idx++, rb.getMass ());

         // inertia
         for (int j = 0; j < 3; j ++) {
            rbInfo.set (i, idx++, rb.getRotationalInertia ().get (j, j));
         }
      }
      ReadWrite.writeMatrixToFile (rbInfo, this.getClass (), "/data/neckModel/neckRbInfo.txt");
   }

   //--------------------------------------------------
   // write joint information
   //--------------------------------------------------
   /**
    * info for joint
    * CW - position - 3
    * CW - axis and angle - 4
    * DW - position - 3
    * DW - axis and angle - 4
    * FEPoly - 4
    * LBPoly - 4
    * ARPoly - 4
    * @throws IOException
    */
   public void writeJointInfo() throws IOException {
      MatrixNd jointInfo = new MatrixNd (mechModel.bodyConnectors ().size (), 26);

      for (int i = 0; i < jointInfo.rowSize (); i ++) {
         RigidTransform3d X = new RigidTransform3d ();
         RigidTransform3d CW = new RigidTransform3d ();
         mechModel.bodyConnectors ().get (i).getCurrentTCW (CW);
         RigidTransform3d DW = new RigidTransform3d ();
         mechModel.bodyConnectors ().get (i).getCurrentTDW (DW);
         //mechModel.bodyConnectors ().get (i).getPose (X);

         int idx = 0;
         // pose
         X.set (CW);
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.p.get (j));
         }
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.R.getAxisAngle ().axis.get (j));
         }
         jointInfo.set (i, idx++, X.R.getAxisAngle().angle);

         X.set (DW);
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.p.get (j));
         }
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.R.getAxisAngle ().axis.get (j));
         }
         jointInfo.set (i, idx++, X.R.getAxisAngle().angle);


         InterveterbralJointFM jointMat = (InterveterbralJointFM)mechModel.frameSprings ().get (i).getMaterial ();
         // FEPoly
         for (int j = 0; j < 4; j ++) {
            jointInfo.set (i, idx++, jointMat.getFEPoly ()[j]);
         }
         // LBPoly
         for (int j = 0; j < 4; j++) {
            jointInfo.set (i, idx++, jointMat.getLBPoly ()[j]);
         }
         // ARPoly
         for (int j = 0; j < 4; j++) {
            jointInfo.set (i, idx++, jointMat.getARPoly ()[j]);
         }
      }

      ReadWrite.writeMatrixToFile (jointInfo, this.getClass (), "/data/neckModel/neckJointInfo.txt");
   }


   /**
    * info for linear joint
    * CW - position - 3
    * CW - axis and angle - 4
    * DW - position - 3
    * DW - axis and angle - 4
    * rotary stiffness - 3
    * rotatry damping - 3
    * 
    * @throws IOException
    */
   public void writeLinearJointInfo() throws IOException {
      MatrixNd jointInfo = new MatrixNd (mechModel.bodyConnectors ().size (), 20);

      for (int i = 0; i < jointInfo.rowSize (); i ++) {
         RigidTransform3d X = new RigidTransform3d ();
         RigidTransform3d CW = new RigidTransform3d ();
         mechModel.bodyConnectors ().get (i).getCurrentTCW (CW);
         RigidTransform3d DW = new RigidTransform3d ();
         mechModel.bodyConnectors ().get (i).getCurrentTDW (DW);
         //mechModel.bodyConnectors ().get (i).getPose (X);

         int idx = 0;
         // pose
         X.set (CW);
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.p.get (j));
         }
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.R.getAxisAngle ().axis.get (j));
         }
         jointInfo.set (i, idx++, X.R.getAxisAngle().angle);

         X.set (DW);
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.p.get (j));
         }
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, X.R.getAxisAngle ().axis.get (j));
         }
         jointInfo.set (i, idx++, X.R.getAxisAngle().angle);

         LinearFrameMaterial jointMat = (LinearFrameMaterial)mechModel.frameSprings ().get (i).getMaterial ();
         // rotary stiffness
         for (int j = 0; j < 3; j ++) {
            jointInfo.set (i, idx++, jointMat.getRotaryStiffness ().get (j));
         }
         // rotary damping
         for (int j = 0; j < 3; j++) {
            jointInfo.set (i, idx++, jointMat.getRotaryDamping ().get (j));
         }
      }

      ReadWrite.writeMatrixToFile (jointInfo, this.getClass (), "/data/neckModel/neckLinearJointInfo.txt");
   }

   //--------------------------------------------------
   // write muscle information
   //--------------------------------------------------
   public void writeMuscleInfo() throws IOException {
      MatrixNd musInfo = new MatrixNd (myMusNum, 9);

      int idx = 0;
      int dataIdx = 0;
      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         for (MultiPointMuscle mus : musList) {

            MasoudMillardLAMExt musMat = (MasoudMillardLAMExt)mus.getMaterial ();
            double [] matPar = musMat.getMaterialParameters ();
            for (int i = 0; i < matPar.length; i++) {
               musInfo.set (idx, i, matPar[i]);
            }
            dataIdx = matPar.length;
            musInfo.set (idx, dataIdx++, mus.getRestLength ());

            idx++;
         }
      }

      ReadWrite.writeMatrixToFile (musInfo, getClass(), "/data/neckModel/neckMuscleInfo.txt");
   }

   public void writeMuscleName () throws IOException {

      String [] musName = new String [myMusNum];
      int idx = 0;

      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         for (MultiPointMuscle mus : musList) {

            String [] musNameSplit = mus.getName().split ("_");
            musName[idx++] = musList.getName () + "_" + musNameSplit[musNameSplit.length-1];

         }
      }

      ReadWrite.writeArrayToFile (musName, this.getClass (), "/data/neckModel/neckMuscleName.txt");
   }

   public void writeMusclePoints () throws IOException {
      double [][] musPoint = new double [myMusNum][];
      int idx = 0;

      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         for (MultiPointMuscle mus : musList) {

            musPoint[idx] = new double [mus.numPoints ()*3];   
            for (int i = 0;  i < mus.numPoints (); i++) {
               for (int j = 0; j < 3; j++) {
                  musPoint[idx][i*3+j] = mus.getPoint (i).getPosition ().get (j);
               }
            }
            idx++;
         }
      }

      ReadWrite.writeArrayToFile (musPoint, getClass(), "/data/neckModel/neckMusclePoint.txt");
   }

   public void writeMuscleAttachName() throws IOException {
      String [][] attName = new String [myMusNum][];
      int idx = 0;

      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         for (MultiPointMuscle mus : musList) {

            attName[idx] = new String [mus.numPoints ()];
            for (int i = 0;  i < mus.numPoints (); i++) {
               Point pnt = mus.getPoint (i);
               attName[idx][i] = pnt.getAttachment ().getMasters ()[0].getName ();
               if (attName[idx][i].equalsIgnoreCase ("Base")) {
                  attName[idx][i] = "C8";
               }
            }
            idx++;
         }
      }

      ReadWrite.writeArrayToFile (attName, getClass(), "/data/neckModel/neckMusclePointAttachName.txt");
   }

   public void writeMuscleExcitations(String filePath) throws IOException {
      VectorNd musExVec = new VectorNd (myMusNum);
      VectorNd musNetExVec = new VectorNd(myMusNum);
      VectorNd ExVec = new VectorNd(muscleList.size ());
      VectorNd ExLRVec = new VectorNd(muscleList.size ()*2);

      int Idx = 0;
      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         for (MultiPointMuscle mus : musList) {
            musExVec.set (Idx, mus.getExcitation ());
            musNetExVec.set (Idx, mus.getNetExcitation ());
            Idx++;
         }
      }

      Idx = 0;
      int exIdx = 0;
      for (MuscleExciter musEx : muscleExciter) {
         if (musEx.getName ().contains ("_l") || musEx.getName ().contains ("_r")) {
            ExLRVec.set (Idx++, musEx.getExcitation ());
         } else {
            ExVec.set (exIdx++, musEx.getExcitation ());
         }
      }

      ReadWrite.writeArrayToFile (musExVec.getBuffer (), getClass(), filePath + "/muscleExcitation.txt");
      ReadWrite.writeArrayToFile (musNetExVec.getBuffer (), getClass(), filePath + "/muscleNetExcitation.txt");
      ReadWrite.writeArrayToFile (ExVec.getBuffer (), getClass(), filePath + "/groupExcitation.txt");
      ReadWrite.writeArrayToFile (ExLRVec.getBuffer (), getClass(), filePath+"/groupLRExcitation.txt");
   }


   //----------------------------------------------------------------------------------------------
   // for inverse simulation
   //----------------------------------------------------------------------------------------------
   public void createTargetList() throws IOException {
      FrameMarker tMr;
      MatrixNd wPntMat = new MatrixNd ();
      ReadWrite.readMatrix (wPntMat, getClass(), "/data/inverse/targetPositions.txt");
      String [] pntNames = new String [wPntMat.rowSize ()];
      pntNames = ReadWrite.readStringArray (getClass(), "/data/inverse/targetNames.txt");

      for(int i = 0; i < pntNames.length; i++) {
         Point3d pnt = new Point3d();
         wPntMat.getRow (i, pnt);
         RigidTransform3d rbPose = new RigidTransform3d(neckRbs.get (pntNames[i]).getPose ());
         rbPose.invert ();
         // target point in rigid body coordinate
         pnt.transform (rbPose);

         tMr = new FrameMarker(neckRbs.get(pntNames[i]), pnt);
         tMr.setName ("targetMarker"+Integer.toString (i));
         RenderProps.setVisible (tMr, false);
         RenderProps.setPointColor (tMr, Color.GREEN);
         RenderProps.setPointStyle (tMr, Renderer.PointStyle.SPHERE);
         RenderProps.setPointRadius (tMr, 0.002);
         tMrList.add (tMr);
      }
      mechModel.add (tMrList);
   }


   public void writeTargetList() throws IOException {
      MatrixNd targetPntMat = new MatrixNd (tMrList.size (), 3);
      MatrixNd locPntMat = new MatrixNd(tMrList.size (), 3);
      String [] pntName = new String [tMrList.size ()];
      MatrixNd probeMat = new MatrixNd (2, tMrList.size ()*3);

      int idx = 0;
      for (int i = 0; i < targetPntMat.rowSize(); i ++) {
         for (int j = 0; j < 3; j++) {
            targetPntMat.set (i, j, tMrList.get (i).getPosition ().get (j));
            locPntMat.set (i, j, tMrList.get (i).getPosition ().get (j));
            pntName[i] = tMrList.get (i).getAttachment ().getMasters ()[0].getName ();
            // make probe file
            probeMat.set (0, idx, targetPntMat.get (i, j));
            probeMat.set (1, idx, targetPntMat.get (i, j));
            idx++;
         }
      }

      ReadWrite.writeMatrixToFile (targetPntMat, getClass(), "/data/inverse/targetPositions.txt");
      ReadWrite.writeMatrixToFile (locPntMat, getClass(), "/data/inverse/targetLocalPositions.txt");
      ReadWrite.writeArrayToFile (pntName, getClass(), "/data/inverse/targetNames.txt");

      double [] times = {0.0, simDuration};
      ReadWrite.writeProbeFile (probeMat, times, 0.0, times[times.length-1], getClass(), "/data/inverse/neutralPositionProbe.txt");

   }

   public void addTrackingController() {

      for (FrameMarker tMr : tMrList) {
         myTrackingController.addMotionTarget(tMr);
         myTrackingController.setTargetsPointRadius (0.002);
         myTrackingController.setTargetsVisible (true);
      }

      for (MuscleExciter musEx : muscleExciter) {
         if (musEx.getName().contains ("_r") || musEx.getName ().contains ("_l")) {
            myTrackingController.addExciter(musEx);
         }
         else {
            continue;
         }
      }

      myTrackingController.addL2RegularizationTerm (0.1);
      //myTrackingController.addTerm(new DampingTerm(TrackingController));
      //DampingTerm trackerDampingTerm = new DampingTerm (myTrackingController, 0.05);
      //myTrackingController.addCostTerm (trackerDampingTerm);


      myTrackingController.setProbeDuration (simDuration);

      myTrackingController.createProbesAndPanel (this);
      addController(myTrackingController);

      try {
         createInputProbeForInverse();
      }
      catch (IOException e) {
         System.out.print("adTranckingController: failed to load probe for inverse simulation!");
         e.printStackTrace();
      }
   }

   public void createInputProbeForInverse() throws IOException { 
      NumericInputProbe p1probe = (NumericInputProbe)getInputProbes ().get (0);
      //NumericInputProbe p1probe = new NumericInputProbe(this, "targetPosition", 0, simDuration);

      p1probe.setAttachedFileName (ArtisynthPath.getSrcRelativePath (this, 
      "/data/inverse/") + "neutralPositionProbe.txt");
      p1probe.setName("Posture");
      p1probe.load ();
      p1probe.setActive (true);
      //addInputProbe(p1probe);
   }

   //----------------------------------------------------------------------------------------------
   // input probe
   //----------------------------------------------------------------------------------------------
   public void createInputProbeForNuetralPosture(String filePath) throws IOException { 
      NumericInputProbe p1probe = new NumericInputProbe(this, "LRExcitation", 0, simDuration);
      p1probe.setAttachedFileName (ArtisynthPath.getSrcRelativePath (this, 
         filePath) + "/inputExcitationProbe.txt");
      p1probe.setName("Input LR Excitation");
      p1probe.load ();
      p1probe.setActive (true);
      addInputProbe(p1probe);
   }
   
   

   //----------------------------------------------------------------------------------------------
   // attach frank to neck model
   //----------------------------------------------------------------------------------------------
   public void attachPharynxNeck () {

      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d> ();

      for (FemNode3d node : GenericModel.findNodesNearSurface(pharynx, GenericModel.loadGeometry(geometryDir, "pharynx_posterior_surf.ply"), 0.0005, true))
      {
         if ( (node.getPosition().z > 0.128) || (node.getPosition().z < 0.017) || (Math.abs(node.getPosition().y) < 0.001) ) {
            node.setDynamic (true);
            nodes.add (node);
         }
      }


      for (int i = 0; i < neckRbs.size (); i++) {
         if (neckRbs.get (i).getName ().equalsIgnoreCase ("C8") || neckRbs.get (i).getName ().endsWith ("C7")) {
            continue;  
         }
         ArrayList<FemNode3d> attNodes = attachFemToRigidBody(pharynx, nodes, neckRbs.get (i), 5, mechModel);

         for (FemNode3d attNode : attNodes) {
            RenderProps.setVisible (attNode, true);
            RenderProps.setPointColor (attNode, Color.RED);
            RenderProps.setPointStyle (attNode, Renderer.PointStyle.SPHERE);
            RenderProps.setPointRadius (attNode, 0.0005);
         }

         System.out.println("attachPharynxNeck: attachment number for " + neckRbs.get(i).getName() + " : " + attNodes.size ());
      }

      for (MultiPointSpringList<MultiPointMuscle> musList : muscleList) {
         //musList.getRenderProps ().setVisible (false);
      }
      for (RigidBody rb : neckRbs) {
         //rb.getRenderProps ().setVisible (false);
      }

   }


   public static ArrayList<FemNode3d> attachFemToRigidBody(FemModel3d fem, RigidBody rb, double distance, boolean surfaceOnly, MechModel mechModel) 
   {
      ArrayList<FemNode3d> nodesToAttach;
      if (surfaceOnly == true)
         nodesToAttach = GenericModel.findNodesNearSurface(fem, rb.getSurfaceMesh(), distance, true);
      else
         nodesToAttach = GenericModel.findNodesInRegion(fem, rb.getSurfaceMesh(), distance, false);

      GenericModel.attachFemNodesToRigidBody(fem, nodesToAttach, rb, mechModel);
      return nodesToAttach;
   }

   public static ArrayList<FemNode3d> attachFemToRigidBody(FemModel3d fem, ArrayList<FemNode3d> nodeList, RigidBody rb, int num, MechModel mechModel) 
   {
      ArrayList<FemNode3d> nodesToAttach;
      nodesToAttach = findNodesNearSurface(nodeList, rb.getSurfaceMesh(), num);
      GenericModel.attachFemNodesToRigidBody(fem, nodesToAttach, rb, mechModel);
      return nodesToAttach;
   }

   public static ArrayList<FemNode3d> findNodesNearSurface(ArrayList<FemNode3d> nodeList, PolygonalMesh surface, int num)
   {
      ArrayList<FemNode3d> nearNodes = new ArrayList<FemNode3d>();
      double [] dsts = new double [num];
      int [] indexs = new int [num];

      for (int i = 0; i < dsts.length; i++) {
         dsts[i] = Double.MAX_VALUE;
         indexs[i] = -1;
      }


      BVFeatureQuery query = new BVFeatureQuery();
      Point3d nearPnt = new Point3d();


      for(FemNode3d node: nodeList)
      {
         Face nearFace = query.nearestFaceToPoint (nearPnt, null, surface, node.getPosition());
         double distance  = node.getPosition().distance (nearPnt);

         for (int i = 0; i < dsts.length; i ++) {
            if (dsts[i] < distance && i > 0) {
               int idx = i;
               double tmp = dsts[idx-1];
               dsts[idx-1] = distance;

               int tmpIn = indexs[idx-1];
               indexs[idx-1] = nodeList.indexOf (node);
               idx--;
               while (idx > 0) {
                  double tmp1 = dsts[idx-1];
                  dsts[idx-1] = tmp;
                  tmp = tmp1;
                  int tmpIn1 = indexs[idx-1];
                  indexs[idx-1] = tmpIn;
                  tmpIn = tmpIn1;
                  idx--;
               }
               break;
            } else if (dsts[i] > distance && i >= num-1) {
               int idx = i;
               double tmp = dsts[idx];
               dsts[idx] = distance;
               int tmpIn = indexs[idx];
               dsts[i] = nodeList.indexOf (node);
               idx--;
               while (idx > 0) {
                  double tmp1 = dsts[idx];
                  dsts[idx] = tmp;
                  tmp = tmp1;
                  int tmpIn1 = indexs[idx];
                  indexs[idx] = tmpIn;
                  tmpIn = tmpIn1;
                  idx--;
               }
            }

         }
      }

      for (int i = 0; i < num; i++) {
         if (indexs[i] != -1) {
            nearNodes.add (nodeList.get (indexs[i]));
         }
      }

      PrintData.printVector (new VectorNd(dsts));

      return nearNodes;
   }

   public void setFrankSkull() {
	   
	   RenderProps.setVisible(cranium, true);
      
      RigidBody skull = neckRbs.get ("C0");
      // attach skull to c0
      FrameFrameAttachment  maxattach = skull.createFrameAttachment (maxilla, maxilla.getPose ());
      FrameFrameAttachment  craattach = skull.createFrameAttachment (cranium, cranium.getPose ());
      mechModel.addAttachment(maxattach);
      mechModel.addAttachment(craattach);
      maxilla.setDynamic(true);
      cranium.setDynamic(true);
      RenderProps.setVisible(skull, false);
      
      // adjust skull mass
      double mass = skull.getMass()-jaw.getMass()-maxilla.getMass()-cranium.getMass();
      if (mass < 0) {
    	  mass = 0;
      }
      skull.setMass(mass);
      
      //      mechModel.addAttachment (ffa);
      

      //      cranium.getRenderProps ().setVisible (true);
      //
      //      /*
      //      RigidTransform3d Tsw = new RigidTransform3d(skull.getPose ());
      //      Tsw.invert ();
      //      maxilla.getMesh ().transform (Tsw);
      //
      //      // save maxilla mesh
      //      File file = new File(this.geometryPath+"/maxilla.obj");
      //      try {
      //         maxilla.getMesh ().write (file, null);
      //      }
      //      catch (IOException e) {
      //         System.out.println ("setFrankSkull: failed to write maxilla mesh!");
      //         e.printStackTrace();
      //      } */
      //
      //      maxilla.setMass (0);
      //      cranium.setMass (0);
      //      skull.setMass (skull.getMass () - jaw.getMass ());
      //      //maxilla.setMesh (null);
      //      maxilla.setDynamic (true);
      //      cranium.setDynamic (true);
      //      FrameFrameAttachment  ffa = skull.createFrameAttachment (maxilla, maxilla.getPose ());
      //      mechModel.addAttachment (ffa);
      //      ffa = neckRbs.get ("C0").createFrameAttachment (cranium, cranium.getPose ());
      //      mechModel.addAttachment (ffa);
      //
      //
      //      //skull.getMasterAttachments ();
      //      PolygonalMesh tmpMesh = loadRigidBody ("tmp", "/maxilla.obj").getMesh ();
      //      neckRbs.get ("C0").setMesh (null);
      //
      //      for (RigidBody rb : neckRbs) {
      //         if (! rb.getName ().contains ("C0")) {
      //            rb.setMass (rb.getMass () * 0.8);
      //         }
      //      }
   }

   public void defineJawContraints()
   {

      double penetrationTol = -1; // 0.002 or 0.002/1000

      // Define the constraints for the jaw motion
      Point3d jawPointL_rel = new Point3d(37.448215, -48.773806, 45.996159);
      Point3d jawPointR_rel = new Point3d(37.448215,  48.773806, 45.996159);
      Point3d jawPointL = new Point3d(137.44308/1000.0, -48.773806/1000.0, 132.34603/1000.0);
      Point3d jawPointR = new Point3d(137.44308/1000.0,  48.773806/1000.0, 132.34603/1000.0);
      jawPointL_rel = jawPointL;
      jawPointR_rel = jawPointR;
      AxisAngle aa0 = new AxisAngle(1.0, 0.0, 0.0, 0.0); 

      /*/
       RigidTransform3d transPoint2World_l = new RigidTransform3d(jawPointL, new AxisAngle(1.0, 0.0, 0.0, 0.0));
       RigidTransform3d transBody2World_l = new RigidTransform3d();
       transBody2World_l.mulInverseLeft(jaw.getPose(), transPoint2World_l );
       SphericalJoint sj_l = new SphericalJoint();
       sj_l.setBodies(jaw, transBody2World_l, null, transPoint2World_l);
       mechModel.addBodyConnector(sj_l);

       RigidTransform3d transPoint2World_r = new RigidTransform3d(jawPointR, new AxisAngle(1.0, 0.0, 0.0, 0.0));
       RigidTransform3d transBody2World_r = new RigidTransform3d();
       transBody2World_r.mulInverseLeft(jaw.getPose(), transPoint2World_r );
       SphericalJoint sj_r = new SphericalJoint();
       sj_r.setBodies(jaw, transBody2World_r, null, transPoint2World_r);
       mechModel.addBodyConnector(sj_r);
       //*/

      //*/
      PlanarConnector pc1 = new PlanarConnector();
      pc1.setName("LPOST"); 
      pc1.setPlaneSize(25.0/1000.0);
      pc1.setUnilateral(false); // TODO: true? why?
      pc1.setPenetrationTol(penetrationTol);
      pc1.setBodies(jaw, new RigidTransform3d(jawPointL_rel, aa0), 
              maxilla, new RigidTransform3d(jawPointL,  new AxisAngle(0.6715429342378181, -0.6715429342378179, -0.3131456130149732, 2.5346467848844263)) );
      BodyConnector tmpBC = mechModel.bodyConnectors ().get (pc1.getName ());
      mechModel.removeBodyConnector (tmpBC);
      mechModel.addBodyConnector(pc1);

      PlanarConnector pc2 = new PlanarConnector();
      pc2.setName("RPOST");
      pc2.setPlaneSize(25.0/1000.0);
      pc2.setUnilateral(false); // true? why?
      pc2.setPenetrationTol(penetrationTol);
      pc2.setBodies(
         jaw,  new RigidTransform3d(jawPointR_rel, aa0), 
         maxilla, new RigidTransform3d(jawPointR,  new AxisAngle(0.6715429342378181, -0.6715429342378179, -0.3131456130149732, 2.5346467848844263) ) );
      tmpBC = mechModel.bodyConnectors ().get (pc2.getName ());
      mechModel.removeBodyConnector (tmpBC);
      mechModel.addBodyConnector(pc2);

      PlanarConnector pc3 = new PlanarConnector();
      pc3.setName("LLTRL");
      pc3.setPlaneSize(25.0/1000.0);
      pc3.setUnilateral(false); // true? why?
      pc3.setPenetrationTol(penetrationTol);
      pc3.setBodies(
         jaw,  new RigidTransform3d(jawPointL_rel, aa0), 
         maxilla, new RigidTransform3d(jawPointL, new AxisAngle(-0.31066696,-0.67795798,-0.66622745,144.91265*Math.PI/180.0) ));
      tmpBC = mechModel.bodyConnectors ().get (pc3.getName ());
      mechModel.removeBodyConnector (tmpBC);
      mechModel.addBodyConnector(pc3);

      PlanarConnector pc4 = new PlanarConnector();
      pc4.setName("RLTRL");
      pc4.setPlaneSize(25.0/1000.0);
      pc4.setUnilateral(false); // true? why?
      pc4.setPenetrationTol(penetrationTol);
      pc4.setBodies(
         jaw,  new RigidTransform3d(jawPointR_rel, aa0), 
         maxilla, new RigidTransform3d(jawPointR, new AxisAngle(0.83699563,0.38354426,-0.39029747,101.1245*Math.PI/180.0)) );
      tmpBC = mechModel.bodyConnectors ().get (pc4.getName ());
      mechModel.removeBodyConnector (tmpBC);
      mechModel.addBodyConnector(pc4);

      // the SegmentedPlanarConnector describes a curved surface constraint, where the curve is given by the segments
      SegmentedPlanarConnector pc5 = new SegmentedPlanarConnector();
      pc5.setName("LTMJ");
      pc5.setPlaneSize(25.0/1000.0);
      pc5.setUnilateral(false);
      pc5.setPenetrationTol(penetrationTol);
      double[] segs5 = {0,0,  -0.0006,-0.00040931413,  -0.0012,-0.00079973446,  -0.0018,-0.001171615,  -0.0024,-0.0015253096,  
                        -0.003,-0.0018611725,  -0.0036,-0.0021795574,  -0.0042,-0.0024808185,  -0.0048,-0.0027653096,  
                        -0.0054,-0.0030333849,  -0.006,-0.0032853982,  -0.0066,-0.0035217035,  -0.0072,-0.0037426548,  
                        -0.0078,-0.0039486062,  -0.0084,-0.0041399115,  -0.009,-0.0043169248,  -0.0096,-0.00448,  
                        -0.0102,-0.0046294912,  -0.0108,-0.0047657522,  -0.0114,-0.0048891372,  -0.012,-0.005};
      pc5.set(
         jaw,  jawPointL_rel, 
         maxilla, new RigidTransform3d(jawPointL, aa0), segs5);
      tmpBC = mechModel.bodyConnectors ().get (pc5.getName ());
      mechModel.removeBodyConnector (tmpBC);
      mechModel.addBodyConnector(pc5);

      SegmentedPlanarConnector pc6 = new SegmentedPlanarConnector();
      pc6.setName("RTMJ");
      pc6.setPlaneSize(25.0/1000.0);
      pc6.setUnilateral(false);
      pc6.setPenetrationTol(penetrationTol);
      pc6.set(
         jaw,  jawPointR_rel, 
         maxilla, new RigidTransform3d(jawPointR, aa0), segs5 );
      tmpBC = mechModel.bodyConnectors ().get (pc6.getName ());
      mechModel.removeBodyConnector (tmpBC);
      mechModel.addBodyConnector(pc6);
      //*/
   }

   //----------------------------------------------------------------------------------------------
   // render non-dynamic components
   //----------------------------------------------------------------------------------------------
   void renderNonDynNodes() {
      for (FemMuscleModel fem: fems) {
         for (FemNode3d node : fem.getNodes ()) {
            if (!node.isDynamic ()) {
               RenderProps.setVisible (node, true);
               RenderProps.setPointColor (node, Color.BLACK);
               RenderProps.setPointStyle (node, Renderer.PointStyle.SPHERE);
               RenderProps.setPointRadius (node, 0.001);
            }
         }
      }
   }

   void setNodeDym() {
      for (FemMuscleModel fem:fems) {
         for (FemNode3d node : fem.getNodes ()) {
            if (!node.isDynamic ()) {
               node.setDynamic (true);
               mechModel.addAttachment (neckRbs.get ("C0").createPointAttachment (node));
            }
         }
      }
   }

   void setCollisionBehavior() {
      mechModel.setCollisionBehavior(pharynx, larynx, true);
   }

   void renderLipTexture() throws IOException {

      String meshFilename = ArtisynthPath.getSrcRelativePath (StaticJawHyoidTongue.class, "geometry/badinairwaylips_morphed_n.obj");
      String textureFilename = ArtisynthPath.getSrcRelativePath (StaticJawHyoidTongue.class, "geometry/lips.jpg");
      PolygonalMesh mesh = new PolygonalMesh (new File (meshFilename));
      //PolygonalMesh mesh = GenericModel.loadGeometry(meshFilename, "airway_complete_tube.ply");
      mesh.triangulate ();
      airwaySkin = new AirwaySkin(mesh);
      airwaySkin.setName ("texturedAirway");
      

      RenderProps props = airwaySkin.getRenderProps ();
      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
      props.setFaceColor (Color.WHITE);
      props.setShading(Renderer.Shading.SMOOTH);
      ColorMapProps tprops = new ColorMapProps();
      tprops.setFileName(textureFilename);
      tprops.setEnabled(true);
      tprops.setColorMixing(Renderer.ColorMixing.MODULATE);
      props.setColorMap(tprops);

      // define the boundary bodies/weights
      for (FemModel3d fem : new FemModel3d[]{face, tongue, softPalate, larynx, pharynx} )
         airwaySkin.addFemModel(fem);
      for (RigidBody rb : new RigidBody[]{maxilla, jaw} )
         airwaySkin.addFrame(rb);
      airwaySkin.computeWeights();

      mechModel.addMeshBody (airwaySkin);
      
      meshFilename = ArtisynthPath.getSrcRelativePath (FrankModel2.class, "geometry/");
      PolygonalMesh faceSurfaceMesh = GenericModel.loadGeometry(meshFilename, "face_surface.vtk");
      faceSurfaceMesh.setName ("SurfaceMesh");
      face.addMesh (faceSurfaceMesh);
   }
   


}
