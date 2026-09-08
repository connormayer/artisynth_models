package artisynth.models.frank3;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.io.MayaAsciiReader;
import maspack.geometry.io.MayaAsciiReader.AngleUnit;
import maspack.geometry.io.MayaAsciiReader.LengthUnit;
import maspack.geometry.io.MayaAsciiReader.TimeUnit;
import maspack.geometry.io.MayaAsciiReader.UnitInfo;
import maspack.matrix.Point3d;
import maspack.render.MeshRenderProps;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;

public class DigitizedPharynx extends RootModel {

   protected MechModel mech;
   public RigidBody maxilla=new RigidBody();
   public RigidBody hyoid=new RigidBody();
   public RigidBody thyroid_R=new RigidBody();
   public RigidBody thyroid_L=new RigidBody();
   public RigidBody cricothyroid=new RigidBody();
   ArrayList<MotionTargetComponent> larynxMarkers;
   protected static String[] fiberGroupNames = {
            "Left_constrictor/L_constrictorA1",
            "Digitized_pharyngeal_muscles/pharynx_2:Muscle1",
            "Digitized_pharyngeal_muscles/pharynx_2:Muscle2",
            "Digitized_pharyngeal_muscles/vessels",
            "Digitized_pharyngeal_muscles/thyroid_cartilage",
            "Digitized_pharyngeal_muscles/LconstrictorA",
            "Digitized_pharyngeal_muscles/LconstrictorB",
            "Digitized_pharyngeal_muscles/Rconstrictor",
            "Digitized_pharyngeal_muscles/Rhyoglossus",
            "Digitized_pharyngeal_muscles/Lhyoglossus",
            "Digitized_pharyngeal_muscles/Rstylopharyngeus",
            "Digitized_pharyngeal_muscles/Rstyloglossus",
            "Digitized_pharyngeal_muscles/Lstylopharyngeus",
            "Digitized_pharyngeal_muscles/Lstyloglossus",
            "Digitized_pharyngeal_muscles/Rsalpingopharyngeus",
            "Digitized_pharyngeal_muscles/Lsalpingopharyngeus",
            "Digitized_pharyngeal_muscles/Lpalatopharyngeus",
            "Digitized_pharyngeal_muscles/Rpalatopharyngeus",
            //"Digitized_pharyngeal_muscles/nasopharyngeal_tube",
            //"Digitized_pharyngeal_muscles/LmedialPterygoid",
            //"Digitized_pharyngeal_muscles/L_LVP",
            //"Digitized_pharyngeal_muscles/R_LVP",
            //"Digitized_pharyngeal_muscles/pharynx_2:Muscle22",
            //"Digitized_pharyngeal_muscles/R_TVP",
            //"Digitized_pharyngeal_muscles/L_TVP",
            // "Digitized_pharyngeal_muscles/Lcoricothyroid",
            //"Digitized_pharyngeal_muscles/Stylohyoid_right",
            "skull12" // skull4:Mesh does not exist?
            };
   protected static Color[] colors = new Color[] {new Color(255,255,240),
                                                  new Color(0, 0, 255), new Color(0, 0, 255), //new Color(0, 128, 0),
                                                  new Color(0, 0, 128), new Color(255, 0, 0),
                                                  new Color(0, 128, 128), new Color(0, 128, 128), //new Color(128, 0, 255),
                                                  new Color(0, 128, 128),//new Color(0, 255, 255), 
                                                  new Color(255, 0, 255),new Color(255, 0, 255),
                                                  //new Color(0, 255, 128), 
                                                  new Color(255, 0, 128),new Color(0, 128, 255),
                                                  new Color(255, 0, 128),new Color(0, 128, 255),
                                                  new Color(128, 0, 255),new Color(128, 0, 255),
                                                  new Color(255, 140, 0), new Color(255, 140, 0), 
                                                  //new Color(255, 150, 150),
                                                  new Color(138, 43, 226), new Color(255, 222, 173),
                                                  new Color(85, 107, 47), new Color(205, 92, 92),
                                                  new Color(255, 218, 185), new Color(45,128,75)};
   public void build(String[] args) throws IOException {
 
      mech = new MechModel("mech");
      mech.setGravity(0,0,0);
      mech.setIntegrator(Integrator.ConstrainedBackwardEuler);
      mech.setMaxStepSize(0.1);
      addModel(mech);
      
      addRigidBodies();
      File fiberFile = new File("C:/Users/Negar/Desktop/Anne's Data/pharyngeal muscles with skull model.ma");
      ArrayList<PolylineMesh> fiberMeshes = new ArrayList<PolylineMesh>(fiberGroupNames.length);
      loadMayaFiberMeshes(fiberGroupNames,fiberFile, fiberMeshes);
      addDigitizedFibers(fiberMeshes);
      
   }
   
   private void addRigidBodies () {
      // Note to myself: For exporting rigidBodies from Maya into the ArtiSynth use the OBJ plugin.
      // That does not need any scaling afterwards.
      RenderProps boneRenderProps=setupBoneRenderProps();
      
      
      
      try {
         maxilla.setMesh (new PolygonalMesh ("C:/Users/Negar/Desktop/Anne's Data/polySurfaceShape2.obj"));
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      maxilla.setName ("polySurfaceShape2");    
      maxilla.setDynamic (false);
      RenderProps.setDrawEdges (maxilla, true);
      maxilla.setRenderProps (boneRenderProps);
      mech.addRigidBody(maxilla);

      try {
         hyoid.setMesh (new PolygonalMesh ("C:/Users/Negar/Desktop/Anne's Data/skull2-polySurfaceShape14.obj"));
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      hyoid.setName ("skull2-polySurfaceShape14");    
      hyoid.setDynamic (false);
      RenderProps.setDrawEdges (hyoid, true);
      hyoid.setRenderProps (boneRenderProps);
      mech.addRigidBody(hyoid);
      
      
      try {
         cricothyroid.setMesh (new PolygonalMesh ("C:/Users/Negar/Desktop/Anne's Data/loftedSurfaceShape25.obj"));
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      cricothyroid.setName ("loftedSurfaceShape25");    
      cricothyroid.setDynamic (false);
      RenderProps.setDrawEdges (cricothyroid, true);
      cricothyroid.setRenderProps (boneRenderProps);
      mech.addRigidBody(cricothyroid);
      
      try {
         thyroid_R.setMesh (new PolygonalMesh ("C:/Users/Negar/Desktop/Anne's Data/loftedSurfaceShape23.obj"));
         thyroid_L.setMesh (new PolygonalMesh ("C:/Users/Negar/Desktop/Anne's Data/loftedSurfaceShape24.obj"));
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      thyroid_R.setName ("loftedSurfaceShape23");    
      thyroid_L.setName ("loftedSurfaceShape24");
      thyroid_R.setDynamic (false);
      thyroid_L.setDynamic (false);
      RenderProps.setDrawEdges (thyroid_R, true);
      RenderProps.setDrawEdges (thyroid_L, true);
      thyroid_R.setRenderProps (boneRenderProps);
      thyroid_L.setRenderProps (boneRenderProps);
      mech.addRigidBody(thyroid_R);
      mech.addRigidBody(thyroid_L);
   }

   public RenderProps setupBoneRenderProps() {
      
      RenderProps boneRenderProps = new MeshRenderProps ();
      boneRenderProps.setAlpha (1);
      boneRenderProps.setShading (Shading.SMOOTH);
      boneRenderProps.setFaceColor (new Color(0.882f,0.831f,0.753f));
      boneRenderProps.setFaceStyle (FaceStyle.FRONT_AND_BACK);
    
      return boneRenderProps;
   }

   private void addDigitizedFibers (ArrayList<PolylineMesh> fiberMeshes){ 
      System.out.println( "\nSize of fiberMeshes is:"+fiberMeshes.size());
      int i=0;
      for (PolylineMesh mesh : fiberMeshes) {
        
         System.out.println( "\nName of mesh is:"+mesh.getName());
         RenderProps.setLineColor(mesh,colors[i]);
         RenderProps.setLineWidth (mesh, 3);
         addRenderable(mesh);
//        MuscleBundle bundle = new MuscleBundle(mesh.getName());
//        pharynx.addMuscleBundle(bundle);
//            
//            //double r = pharynx.updateVolume();
//            //r = Math.pow(3.0 * r / pharynx.numElements() / 8.0 / Math.PI / 4.0, 0.3);
//            
//        bundle.addFiberMeshElements(r, mesh); 
         i++;
        }
        
      }
   private static void loadMayaFiberMeshes(String[] groupNames, File fiberFile,
      ArrayList<PolylineMesh> fiberMeshes) {

      MayaAsciiReader mar = null;
      try {
         mar = new MayaAsciiReader(fiberFile);
         //PrintWriter out = new PrintWriter("C:/Users/Negar/workspace/FilesForLarynx/hierarchy.txt");
         //for (String s: mar.getGroupHierarchy ("-")){out.println(s);}
         //out.close();
         
      } catch (IOException e) {
         e.printStackTrace();
         return;
      }

      for (String name : groupNames) {
         PolylineMesh mesh = mar.getPolylineMesh(name,
            new UnitInfo(LengthUnit.CENTIMETER, AngleUnit.RADIAN, TimeUnit.SECOND));
            //mesh.transform(myTransform);

         mesh.setName(name.replace('/', '_').replace (':', '_'));
         fiberMeshes.add(mesh);
      }
   }
}
