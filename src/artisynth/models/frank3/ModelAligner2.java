package artisynth.models.frank3;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.DampingTerm;
import artisynth.core.inverse.L2RegularizationTerm;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearFrameMaterial;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpringList;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.frank2.FrankModel2;
import artisynth.models.modelOrderReduction.PrintData;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.neckModel.InterveterbralJointFM;
import artisynth.models.neckModel.MasoudMillardLAMExt;
import artisynth.models.neckModel.NeckWithMultiPointMuscle;
import maspack.geometry.CPD;
import maspack.geometry.ICPRegistration;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;

public class ModelAligner2 extends NeckWithMultiPointMuscle{

   RenderableComponentList<RenderableComponentBase> neckComponents = new RenderableComponentList<RenderableComponentBase> (RenderableComponentBase.class, "neckModelComponents", "nMCs");
   RenderableComponentList<RenderableComponentBase> frankComponents = new RenderableComponentList<RenderableComponentBase> (RenderableComponentBase.class, "frankModelComponents", "fMCs");
   double simDuration = 1.0;
   ArrayList<FrameMarker> tMrList = new ArrayList<FrameMarker> ();
   ComponentList<RigidBody> neckRbs;
   //NeckWithMultiPointMuscle neck ;
   FrankModel2 frank;
   MechModel mechModel;
   TrackingController myTrackingController = new TrackingController(mech, "tcon");
   public AffineTransform3d T = new AffineTransform3d();



   public ControlPanel alignmentPanel;
   public boolean traGeo = false;
   public boolean saveGeo = false;


   public ModelAligner2 (String name) throws IOException  {


      super(name);
      Boolean implement = true;
      // import neck model
      //neck = new NeckWithMultiPointMuscle("neck model");
      frank = new FrankModel2();
      //frank.build (new String[1]);
      mechModel = frank.mechModel;
      neckRbs = (ComponentList<RigidBody>)mech.rigidBodies ();
      addModel(frank);



      MatrixNd rMat = new MatrixNd ();

      if (!implement) {
         AffineTransform3d X = new AffineTransform3d();
         ReadWrite.readMatrix (rMat, this.getClass (), "data/AffineTransform_neckToFrank.txt");
         X = new AffineTransform3d();
         X.set (rMat);
         //mech.transformGeometry (X);

         Point3d pnt0 = neckRbs.get ("C7").getPosition ();
         Point3d pnt1 = new Point3d (0.187022, -0.00150266, 0.0153714);
         pnt1.sub (pnt0);
         T.A.setIdentity ();
         T.p.set (pnt1);

         T.mul (T,X);

      } else {
         ReadWrite.readMatrix (rMat, this.getClass (), "data/AffineTransform_neckToFrank1.txt");
         T.set (rMat);
      }

      mech.transformGeometry (T);

      //mech.setGravity (0, 0, 0);
      //mechModel.setGravity (0, 0, 0);

      //addTargetPoints();

      if (implement) {
         T.invert ();
         mech.transformGeometry (T);
         T.invert ();
      }



      alignmentPanel = new ControlPanel("Alignment");
      alignmentPanel.addWidget (this, "transform");
      alignmentPanel.addWidget (this, "saveGeometry");
      alignmentPanel.addWidget (neckRbs.get(0).getName()+"Visible", this, "rigidBodies/"+neckRbs.get (0).getName()+":renderProps.visible");
      this.addControlPanel(alignmentPanel);
      this.mergeAllControlPanels(true);

      //writePnts();
      addTargetPoints1();
      addTrackingController();
      //PntController pntCnt = new PntController(tMrList);
      //addController(pntCnt);
      createInputProbeForPosture();
   }

   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);
   }


   //Property list
   public static PropertyList myProps = new PropertyList (
      ModelAligner2.class, NeckWithMultiPointMuscle.class);

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add ("targetPosition", "The position of target frame markers", new double[24]);
      myProps.add ("transform", "transform geometry", false);
      myProps.add ("saveGeometry", "save geometries", false);
   }

   public void setTargetPosition(double [] psts) {

      for (int i = 0; i < tMrList.size (); i++) {
         FrameMarker fm = tMrList.get (i);
         double [] pst = new double [3];
         pst [0] = psts[i*3];
         pst [1] = psts[i*3+1];
         pst [2] = psts[i*3+2];
         Point3d pnt = new Point3d (pst);
         //pnt.scale (0.01);
         //fm.setPosition (pnt);
         fm.setTargetPosition (pnt);
      }

   }

   public double [] getTargetPosition() {

      double [] psts = new double [24];

      for (int i = 0; i < tMrList.size (); i++) {
         FrameMarker fm = tMrList.get (i);
         double [] pst = new double [3];
         //fm.getTargetPosition ().get (pst);

         //Point3d pnt = new Point3d (fm.getPosition ());
         Point3d pnt = new Point3d (fm.getTargetPosition ());
         //pnt.scale (0.01);
         pnt.get (pst);

         psts[i*3] = pst[0];
         psts[i*3+1] = pst [1];
         psts[i*3+2] = pst [2];
      }

      return psts;
   }

   public void setTransform(boolean listener) {
      traGeo = listener;
      if (traGeo) {
         mech.transformGeometry (T);
         traGeo = false;
      }
   }

   public boolean getTransform() {
      return traGeo;
   }

   public void setSaveGeometry(boolean listener) {
      saveGeo = listener;
      if (saveGeo) {
         String meshPath =
         ArtisynthPath.getSrcRelativePath (
            this.getClass (), "/data/");
         for (int i = 0; i < neckRbs.size (); i++) {
            File fp = new File(meshPath + "C" + i + ".obj");
            try {
               neckRbs.get (8-i).getMesh ().write (fp, "%.8g", false);
               writeBoneInfo();
               //writeJointInfo();
               writeLinearJointInfo();
               writeMuscleInfo();
               writeMuscleName();
               writeMusclePoints();
               writeMuscleAttachName();
            }
            catch (IOException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
         }
         saveGeo = false;
      }
   }

   public boolean getSaveGeometry() {
      return saveGeo;
   }



   /*
    * inverse simulation
    */
   public void addTrackingController()  {

      for (FrameMarker tMr : tMrList) {
         myTrackingController.addMotionTarget(tMr);
         myTrackingController.setTargetsPointRadius (0.002);
         myTrackingController.setTargetsVisible (true);
      }

      for (MuscleExciter musEx : muscleExciter) {
         if (musEx.getName().contains ("_r") || musEx.getName ().contains ("_l")) {
            continue;
         }
         else {
            myTrackingController.addExciter(musEx);
         }
      }

      myTrackingController.addL2RegularizationTerm (0.6);
      //myTrackingController.addTerm(new DampingTerm(TrackingController));
      myTrackingController.addDampingTerm (0.05);


      myTrackingController.setProbeDuration (simDuration);

      myTrackingController.createProbesAndPanel (this);
      addController(myTrackingController);

   }

   public void createInputProbeForPosture() throws IOException { 
      NumericInputProbe p1probe = (NumericInputProbe)getInputProbes ().get (0);
      //NumericInputProbe p1probe = new NumericInputProbe(this, "targetPosition", 0, simDuration);

      p1probe.setAttachedFileName (ArtisynthPath.getSrcRelativePath (this, 
      "/data/") + "ChangePosture.txt");
      p1probe.setName("Posture");
      p1probe.load ();
      p1probe.setActive (true);
      //addInputProbe(p1probe);
   }

   //--------------------------------------------------
   // write rigidBody information
   //--------------------------------------------------

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
   public void writeJointInfo() throws IOException {
      MatrixNd jointInfo = new MatrixNd (mech.bodyConnectors ().size (), 26);

      for (int i = 0; i < jointInfo.rowSize (); i ++) {
         RigidTransform3d X = new RigidTransform3d ();
         RigidTransform3d CW = new RigidTransform3d ();
         mech.bodyConnectors ().get (i).getCurrentTCW (CW);
         RigidTransform3d DW = new RigidTransform3d ();
         mech.bodyConnectors ().get (i).getCurrentTDW (DW);
         //mech.bodyConnectors ().get (i).getPose (X);

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


         InterveterbralJointFM jointMat = (InterveterbralJointFM)mech.frameSprings ().get (i).getMaterial ();
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

   public void writeLinearJointInfo() throws IOException {
      MatrixNd jointInfo = new MatrixNd (mech.bodyConnectors ().size (), 20);

      for (int i = 0; i < jointInfo.rowSize (); i ++) {
         RigidTransform3d X = new RigidTransform3d ();
         RigidTransform3d CW = new RigidTransform3d ();
         mech.bodyConnectors ().get (i).getCurrentTCW (CW);
         RigidTransform3d DW = new RigidTransform3d ();
         mech.bodyConnectors ().get (i).getCurrentTDW (DW);
         //mech.bodyConnectors ().get (i).getPose (X);

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

         LinearFrameMaterial jointMat = (LinearFrameMaterial)mech.frameSprings ().get (i).getMaterial ();
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
      MatrixNd musInfo = new MatrixNd (116, 9);

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

      String [] musName = new String [116];
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
      double [][] musPoint = new double [116][];
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
      String [][] attName = new String [116][];
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


   //--------------------------------------------------
   // set render properties
   //--------------------------------------------------

   public static void setRenderProperties (RenderableComponentList<RenderableComponentBase> argCom, Color argFaceColor) {
      for (RenderableComponentBase temMeshCom : argCom) {
         RenderProps.setFaceStyle (temMeshCom, Renderer.FaceStyle.FRONT_AND_BACK);
         RenderProps.setFaceColor (temMeshCom, argFaceColor);
         RenderProps.setShading (temMeshCom, Renderer.Shading.SMOOTH);
      }
   }

   public static void setRenderProperties (RenderableComponentList<RenderableComponentBase> argCom, Color argFaceColor, Color argEdgeColor) {
      for (RenderableComponentBase temMeshCom : argCom) {
         RenderProps.setFaceStyle (temMeshCom, Renderer.FaceStyle.FRONT_AND_BACK);
         RenderProps.setFaceColor (temMeshCom, argFaceColor);
         RenderProps.setShading (temMeshCom, Renderer.Shading.SMOOTH);
         RenderProps.setDrawEdges (temMeshCom, true);
         RenderProps.setLineColor (temMeshCom, argEdgeColor);
      }
   }

   public static void setRenderProperties (RenderableComponentList<RenderableComponentBase> argCom, String name, Color argFaceColor) {
      for (RenderableComponentBase temMeshCom : argCom) {
         if (temMeshCom.getName ().contains (name)) {
            RenderProps.setFaceStyle (temMeshCom, Renderer.FaceStyle.FRONT_AND_BACK);
            RenderProps.setFaceColor (temMeshCom, argFaceColor);
            RenderProps.setShading (temMeshCom, Renderer.Shading.SMOOTH);
            RenderProps.setDrawEdges (temMeshCom, false);
         }
      }
   }

   public static void setRenderProperties (RenderableComponentList<RenderableComponentBase> argCom, String name, Color argFaceColor, Color argEdgeColor) {
      for (RenderableComponentBase temMeshCom : argCom) {
         if (temMeshCom.getName ().contains (name)) {
            RenderProps.setFaceStyle (temMeshCom, Renderer.FaceStyle.FRONT_AND_BACK);
            RenderProps.setFaceColor (temMeshCom, argFaceColor);
            RenderProps.setShading (temMeshCom, Renderer.Shading.SMOOTH);
            RenderProps.setDrawEdges (temMeshCom, true);
            RenderProps.setEdgeColor (temMeshCom, argEdgeColor);
         }
      }
   }

   //--------------------------------------------------
   // make ICP registration
   //--------------------------------------------------

   public AffineTransform3d myICPRegistration(RenderableComponentList<RenderableComponentBase> tarComs, String tarName, RenderableComponentList<RenderableComponentBase> refComs, String refName) {

      // get target mesh 
      RenderableComponentBase tarMeshBody = null;
      for (RenderableComponentBase temFixedMesh : tarComs) {
         if (temFixedMesh.getName ().contains (tarName)) {
            tarMeshBody = temFixedMesh;
         }
      } 

      // get reference mesh
      RenderableComponentBase refMeshBody = null;
      for (RenderableComponentBase temFixedMesh : refComs) {
         if (temFixedMesh.getName ().contains (refName)) {
            refMeshBody = temFixedMesh;
         }
      }

      // get transform for target mesh
      //AffineTransform3d X = getFixedMeshTransform(refMeshBody, tarMeshBody);
      AffineTransform3d X = getFixedMeshTransform(refMeshBody, tarMeshBody);

      return X;
   }


   public static AffineTransform3d getFixedMeshTransform(RenderableComponentBase refMeshBody, RenderableComponentBase tarMeshBody) {

      RigidBody ref = (RigidBody)refMeshBody;
      RigidBody tar = (RigidBody)tarMeshBody;
      PolygonalMesh refMesh = (PolygonalMesh) ref.getMesh();
      PolygonalMesh tarMesh = (PolygonalMesh) tar.getMesh();

      return getFixedMeshTransform(refMesh, tarMesh);
   }

   public static AffineTransform3d getFixedMeshTransform(PolygonalMesh refMesh, PolygonalMesh tarMesh) {

      ICPRegistration myBoneRegister = new ICPRegistration();
      AffineTransform3d X = new AffineTransform3d();
      //AffineTransform3d X;

      System.out.println ("ICP registering " + " ...");
      myBoneRegister.registerICP (X, refMesh, tarMesh, 3, 7);

      return X;
   }

   //--------------------------------------------------
   // make rigid CPD registration
   //--------------------------------------------------

   public AffineTransform3d myCPDRegistration(RenderableComponentList<RenderableComponentBase> tarComs, String tarName, RenderableComponentList<RenderableComponentBase> refComs, String refName) {
      // get target mesh 

      RenderableComponentBase tarMeshBody = null;
      for (RenderableComponentBase temFixedMesh : tarComs) {
         if (temFixedMesh.getName ().contains (tarName)) {
            tarMeshBody = temFixedMesh;
         }
      } 

      // get reference mesh
      RenderableComponentBase refMeshBody = null;
      for (RenderableComponentBase temFixedMesh : refComs) {
         if (temFixedMesh.getName ().contains (refName)) {
            refMeshBody = temFixedMesh;
         }
      }

      // get transform for target mesh
      AffineTransform3d X = getFixedMeshCPDTransform(refMeshBody, tarMeshBody);

      return X;
   }

   public static AffineTransform3d getFixedMeshCPDTransform(RenderableComponentBase refMeshBody, RenderableComponentBase tarMeshBody) {

      RigidBody ref = (RigidBody)refMeshBody;
      RigidBody tar = (RigidBody)tarMeshBody;
      PolygonalMesh refMesh = (PolygonalMesh) ref.getMesh();
      PolygonalMesh tarMesh = (PolygonalMesh) tar.getMesh();;

      return getFixedMeshCPDTransform(refMesh, tarMesh);
   }

   public static AffineTransform3d getFixedMeshCPDTransform(PolygonalMesh refMesh, PolygonalMesh tarMesh) {


      AffineTransform3d X = new AffineTransform3d();
      //ScaledRigidTransform3d temX = new ScaledRigidTransform3d ();

      System.out.println ("CPD registering " + " ...");
      X = CPD.affine(refMesh, tarMesh, 0.1, 1, 100);
      //X.set (temX);

      return X;
   }

   //--------------------------------------------------
   // make affine transformation
   //--------------------------------------------------
   public void makeAffineTransform(RenderableComponentList<RenderableComponentBase> refComs, AffineTransform3d argX) {
      // get reference mesh
      for (RenderableComponentBase temFixedMesh : refComs) {
         RigidBody tmp = (RigidBody)temFixedMesh;
         tmp.getMesh ().transform (argX);
      }
   }

   public void makeAffineTransform(RenderableComponentList<RenderableComponentBase> refComs, AffineTransform3d argX, String argName) {
      // get reference mesh
      for (RenderableComponentBase temFixedMesh : refComs) {
         if (temFixedMesh.getName ().contains (argName)) {
            System.out.println("Transform Component " + temFixedMesh.getName ());
            RigidBody tmp = (RigidBody)temFixedMesh;
            tmp.getMesh ().transform (argX);
         }
      }
   }

   //--------------------------------------------------
   // add target points
   //--------------------------------------------------

   public void addTargetPoints() throws IOException {
      MatrixNd pnts = new MatrixNd ();
      ReadWrite.readMatrix (pnts, this.getClass (), "/data/TargetPoints_SpineAlign.txt");


      Point3d  tmpPnt3d;
      FemElement3d elem;
      Point3d locPnt = new Point3d();
      MatrixNd pnts1 = new MatrixNd (pnts.rowSize (), 3);
      ReadWrite.readMatrix (pnts1, this.getClass (), "/data/TargetPoints_SpineAlign1.txt");
      //SkinMeshBody tmpSkin = (SkinMeshBody)mechModel.meshBodies().get ("airway_tube");
      for (int i = 0; i < pnts.rowSize (); i ++) {

         // position
         System.out.println ("C"+i);
         RigidTransform3d rT = new RigidTransform3d(neckRbs.get ("C"+ i).getPose ());
         rT.invert ();

         tmpPnt3d = new Point3d (pnts.get (i, 0), pnts.get (i, 1), pnts.get (i, 2));
         if (i != 0) {
            frank.pharynx.findNearestSurfaceElement (locPnt, tmpPnt3d);
         }
         else {
            pnts1.getRow (i, locPnt);
         }

         tmpPnt3d.transform (rT);
         PrintData.printVector (tmpPnt3d);
         FrameMarker fM = new FrameMarker(tmpPnt3d);
         fM.setFrame (neckRbs.get ("C" +i));
         tMrList.add (fM);
         tMrList.get (i).setName ("C" + i +"prx");
         pnts1.setRow (i, locPnt);
         Point lp = new Point(locPnt);
         lp.setName ("destC"+i);
         RenderProps.setVisible (lp, true);
         RenderProps.setPointColor (lp, Color.GREEN);
         RenderProps.setPointStyle (lp, Renderer.PointStyle.SPHERE);
         RenderProps.setPointRadius (lp, 0.002);
         mech.addPoint (lp);

      }

      for(int i = 0; i < tMrList.size (); i++) {
         RenderProps.setVisible (tMrList.get (i), true);
         RenderProps.setPointColor (tMrList.get (i), Color.GREEN);
         RenderProps.setPointStyle (tMrList.get (i), Renderer.PointStyle.SPHERE);
         RenderProps.setPointRadius (tMrList.get (i), 0.002);
         mech.addFrameMarker (tMrList.get (i));
      }

   }


   public void addTargetPoints1() throws IOException {
      MatrixNd pnts = new MatrixNd ();
      ReadWrite.readMatrix (pnts, this.getClass (), "/data/TargetPoints_SpineAlign2.txt");


      Point3d  tmpPnt3d;
      FemElement3d elem;
      Point3d locPnt = new Point3d();
      MatrixNd pnts1 = new MatrixNd (pnts.rowSize (), 3);
      //SkinMeshBody tmpSkin = (SkinMeshBody)mechModel.meshBodies().get ("airway_tube");
      for (int i = 0; i < pnts.rowSize (); i ++) {
         // position
         System.out.println ("C"+i);
         RigidTransform3d rT = new RigidTransform3d(neckRbs.get ("C"+ i).getPose ());
         rT.invert ();
         tmpPnt3d = new Point3d (pnts.get (i, 0), pnts.get (i, 1), pnts.get (i, 2));
         //frank.pharynx.findNearestSurfaceElement (locPnt, tmpPnt3d);
         tmpPnt3d.transform (rT);
         PrintData.printVector (tmpPnt3d);
         FrameMarker fM = new FrameMarker(tmpPnt3d);
         fM.setFrame (neckRbs.get ("C" +i));
         tMrList.add (fM);
         tMrList.get (i).setName ("C" + i +"prx");
         ReadWrite.readMatrix (pnts1, this.getClass (), "/data/TargetPoints_SpineAlign3.txt");
         pnts1.getRow (i, locPnt);
         Point lp = new Point(locPnt);
         lp.setName ("destC"+i);
         RenderProps.setVisible (lp, true);
         RenderProps.setPointColor (lp, Color.GREEN);
         RenderProps.setPointStyle (lp, Renderer.PointStyle.SPHERE);
         RenderProps.setPointRadius (lp, 0.002);
         mech.addPoint (lp);

         //fM.setPosition (locPnt);
      }

      for(int i = 0; i < tMrList.size (); i++) {
         RenderProps.setVisible (tMrList.get (i), true);
         RenderProps.setPointColor (tMrList.get (i), Color.GREEN);
         RenderProps.setPointStyle (tMrList.get (i), Renderer.PointStyle.SPHERE);
         RenderProps.setPointRadius (tMrList.get (i), 0.002);
         mech.addFrameMarker (tMrList.get (i));
      }

      //ReadWrite.writeMatrixToFile (pnts1, this.getClass (), "/data/TargetPoints_SpineAlign1.txt");
   }

   //--------------------------------------------------
   // add target points
   //--------------------------------------------------
   public void writePnts() throws IOException {
      MatrixNd pnt0 = new MatrixNd(8, 3);
      for (int i = 0; i < pnt0.rowSize (); i++) {
         for (int j = 0; j < pnt0.colSize (); j++) {
            pnt0.set (i, j, mech.frameMarkers ().get ("C"+i+"prx").getPosition ().get (j));
         }
      }
      ReadWrite.writeMatrixToFile (pnt0, this.getClass (), "/data/TargetPoints_SpineAlign2.txt");

      MatrixNd pnt1 = new MatrixNd (8, 3);
      for (int i = 0; i < pnt1.rowSize (); i++) {
         for (int j = 0; j < pnt1.colSize (); j++) {
            pnt1.set (i, j, mech.points ().get ("destC"+i).getPosition ().get (j));
         }
      }
      ReadWrite.writeMatrixToFile (pnt1, this.getClass (), "/data/TargetPoints_SpineAlign3.txt");

   }


   //--------------------------------------------------
   // add target points
   //--------------------------------------------------
   public class PntController extends ControllerBase {
      ArrayList<FrameMarker> fmList;
      MatrixNd mat0 = new MatrixNd ();
      MatrixNd mat1 = new MatrixNd();
      double duration = 1.0;
      MatrixNd steps;
      double time = 0;

      public PntController (ArrayList<FrameMarker> fmList) throws IOException {
         this.fmList = fmList;
         ReadWrite.readMatrix (mat0, FrankModel3.class, "/data/TargetPoints_SpineAlign2.txt");
         ReadWrite.readMatrix (mat1, FrankModel3.class, "/data/TargetPoints_SpineAlign3.txt");

         steps = new MatrixNd (mat0.rowSize (), mat0.colSize ());
         for (int i = 0; i < steps.rowSize (); i++) {
            for (int j = 0; j < steps.colSize (); j++) {
               double step = 0;
               step = (mat1.get (i, j)-mat0.get (i, j))/duration;
               steps.set (i, j, step);
            }
         }
      }
      @Override
      public void apply (double t0, double t1) {
         // TODO Auto-generated method stub
         Point3d pnt = new Point3d();
         if (time < duration) {
            for (int i = 0; i < mat0.rowSize (); i++) {
               for (int j = 0; j < 3; j++) {
                  pnt.set (j, mat0.get (i, j) + steps.get (i, j)*time);
               }
               PrintData.printVector (pnt);
               fmList.get (i).setPosition (pnt);
            }
         }

         time = time + t1 - t0;
      }




   }
}
