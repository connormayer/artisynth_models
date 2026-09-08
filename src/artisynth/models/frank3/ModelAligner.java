package artisynth.models.frank3;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank2.FrankModel2;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.neckModel.Neck;
import artisynth.models.swollowingFrank.MeshExperiment;
import maspack.geometry.CPD;
import maspack.geometry.ICPRegistration;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;

public class ModelAligner extends RootModel{

   RenderableComponentList<RenderableComponentBase> neckComponents = new RenderableComponentList<RenderableComponentBase> (RenderableComponentBase.class, "neckModelComponents", "nMCs");
   RenderableComponentList<RenderableComponentBase> frankComponents = new RenderableComponentList<RenderableComponentBase> (RenderableComponentBase.class, "frankModelComponents", "fMCs");
   MechModel mechModel = new MechModel("frank3");


   public void build (String [] args) throws IOException {


      // import neck model
      Neck neck = new Neck("neck model");
      MechModel neckMech = neck.getMech();
      ComponentList<RigidBody> neckRbs = (ComponentList<RigidBody>)neckMech.rigidBodies ();

      // add mesh 
      ReadWrite.addMesh (neckComponents, this.getClass (), "jawNeckModel.ply");
      ReadWrite.addMesh (neckComponents, this.getClass (), "baseNeckModel.obj");
      ReadWrite.addMesh (frankComponents, MeshExperiment.class, "jaw_surface.vtk");

      setRenderProperties(neckComponents, Color.CYAN);
      setRenderProperties(frankComponents, Color.GREEN);
      mechModel.add (neckComponents);
      mechModel.add (frankComponents);

      AffineTransform3d X1, X2, X3;
      
      if (false) {
      // transform neck model to data
      RigidBody tmpRb;
      try {
         tmpRb = (RigidBody)neckRbs.get ("Base").clone ();
         tmpRb.setName ("Base0");
         neckComponents.add (tmpRb);
      }
      catch (CloneNotSupportedException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      
      X1 = myICPRegistration(neckComponents, "Base0", neckComponents, "baseNeckModel");
      neckMech.transformGeometry (X1);
      //X = myCPDRegistration(neckComponents, "Base0", neckComponents, "baseNeckModel");
      MatrixNd rMat = new MatrixNd ();
      ReadWrite.readMatrix (rMat, this.getClass (), "data/CPDTransform.txt");
      X2 = new AffineTransform3d();
      X2.set (rMat);
      neckMech.transformGeometry (X2);
      //ReadWrite.writeMatrixToFile (X2, this.getClass (), "data/CPDTransform.txt");
      
      // transform neck model to frank
      X3 = myICPRegistration(frankComponents, "jaw", neckComponents, "jaw");
      X3.invert ();
      makeAffineTransform(neckComponents, X3, "NeckModel");
      neckMech.transformGeometry (X3);
      ReadWrite.writeMatrixToFile (X3, this.getClass (), "data/AffineTransform_dataToFrank.txt");
      
      X2.mul (X1);
      X3.mul (X2);
      
      ReadWrite.writeMatrixToFile (X2, this.getClass (), "data/AffineTransform_neckToData.txt");
      ReadWrite.writeMatrixToFile (X3, this.getClass (), "data/AffineTransform_neckToFrank.txt");
      }
      else {
         MatrixNd rMat = new MatrixNd ();
         ReadWrite.readMatrix (rMat, this.getClass (), "data/AffineTransform_dataToFrank.txt");
         X3 = new AffineTransform3d();
         X3.set (rMat);
         ReadWrite.readMatrix (rMat, this.getClass (), "data/AffineTransform_neckToFrank.txt");
         X2 = new AffineTransform3d();
         X2.set (rMat);
         neckMech.transformGeometry (X2);
         makeAffineTransform(neckComponents, X3, "NeckModel");
      }
      
      

      

      addModel(mechModel);
      addModel(neck);
   }


   public void attach (DriverInterface driver)
   {
      //this.getMainViewer().setBackgroundColor(Color.white);
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


}
