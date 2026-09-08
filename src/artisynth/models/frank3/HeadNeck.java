package artisynth.models.frank3;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.DampingTerm;
import artisynth.core.inverse.L2RegularizationTerm;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearFrameMaterial;
import artisynth.core.mechmodels.AxialSpringList;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpringList;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SphericalJoint;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank2.GenericModel;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.neckModel.InterveterbralJointFM;
import artisynth.models.neckModel.MasoudMillardLAMExt;
import artisynth.models.neckModel.NeckWithMultiPointMuscle;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;

public class HeadNeck extends RootModel{


   String workingPath = ArtisynthPath.getSrcRelativePath (this.getClass(), "");
   String geometryPath = workingPath + "/geometry";
   String dataPath = workingPath + "/data";
   String neckDataPath = dataPath + "/neckModel";

   public MechModel mechModel = new MechModel("HeadNeck");
   public MechModel referenceModel = new MechModel("Reference");

   // neck rigid body and joint
   RenderableComponentList<RigidBody> neckRbs = new RenderableComponentList<RigidBody> (RigidBody.class, "neckRigidBodys", "nRbs");
   RenderableComponentList<RigidBody> rbs;
   RenderableComponentList<SphericalJoint> neckSjs = new RenderableComponentList<SphericalJoint> (SphericalJoint.class, "neckSphericalJoints", "nSjs");
   RenderableComponentList<FrameSpring> neckFss = new RenderableComponentList<FrameSpring> (FrameSpring.class, "neckFrameSprings", "nFss");;

   // neck muscles
   public ComponentList<MultiPointSpringList> muscleList = new ComponentList<MultiPointSpringList>(MultiPointSpringList.class, "MultiPointMuscles", "Ms");
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
   public double simDuration = 10.0;

   // control panel
   public ControlPanel alignmentPanel;
   public boolean saveGeo = false;

   // render frank components
   RenderableComponentList<RenderableComponentBase> softComponents = new RenderableComponentList<RenderableComponentBase> (RenderableComponentBase.class, "softTissues", "sTs");

   public void build (String [] args) throws IOException {
      super.build (args);
      addModel(mechModel);
      addHeadNeck();
      mechModel.rigidBodies ().get ("C8").setDynamic (false);

      // control panel
      alignmentPanel = new ControlPanel("Alignment");
      alignmentPanel.addWidget (this, "saveGeometry");
      alignmentPanel.addWidget (this, "saveMuscleExcitation");
      alignmentPanel.addWidget (this, "loadMuscleExictation");
      this.addControlPanel(alignmentPanel);
      
      // inverse simulation
      createTargetList();
      writeTargetList();
      //addTrackingController();
      //createInputProbeForNuetralPosture("/data/neckModel");

      // render frank components
      addModel(referenceModel);
      referenceModel.add (softComponents);
      ReadWrite.addMesh (softComponents, getClass(), "pharynx_surface.ply");
      ModelAligner2.setRenderProperties (softComponents, Color.CYAN);
      referenceModel.setGravity (0, 0, 0);
   }

   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);
   }

   //Property list
   public static PropertyList myProps = new PropertyList (
      HeadNeck.class, RootModel.class);

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


   //--------------------------------------------------
   // read model from file
   //--------------------------------------------------

   public void addHeadNeck() {
      try {
         addHeadNeckBone();
         addHeadNeckJoint();
         addHeadNeckMuscle();
         addMuscleExciter();
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
         mechModel.addRigidBody (rb);
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
      musRenderProps.setLineColor (Color.WHITE);
      musRenderProps.setLineRadius (0.002);
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
         RenderProps.setVisible (tMr, true);
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

   public void addTrackingController() throws IOException  {

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
      
      createInputProbeForInverse();
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


}
