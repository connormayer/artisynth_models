package artisynth.models.frank2;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.VtkAsciiReader;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank3.FrankModel3;
import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.correspondences.ICPMeshCorrespondence;
import artisynth.models.registration.weights.GaussianWeightFunction;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.WavefrontReader;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class SoftPalate extends RootModel {

   public String modelDir   = ArtisynthPath.getSrcRelativePath (SoftPalate.class, "");
   public String geometryDir = ArtisynthPath.getSrcRelativePath (FrankModel3.class, "geometry/softPalate/");
   public String muscleDir = ArtisynthPath.getSrcRelativePath (FrankModel2.class, "muscles/PalateMuscles/");

   MechModel mechModel = new MechModel("SoftPalate");
   
   PolygonalMesh srcPalate;
   PolygonalMesh targetPalate;
   FemModel3d srcBoundingFem;
   
   MuscleBundleData[] mbd;
   LinkedHashMap<MusclePoint,FemMarker> muscleMap;
   
   public void build (String [] args) throws IOException {
      super.build (args);
      addModel(mechModel);

      loadMuscleInfo();
      addSoftPalate();
      
      setupRegistration();
   }

   void addSoftPalate() throws IOException {

      String palateMeshName = "palate_src";
      FemModel3d softPalate = new FemModel3d();
      GenericModel.loadFemMesh(softPalate, geometryDir, palateMeshName);
      VtkAsciiReader.read(softPalate, geometryDir + "/" + palateMeshName + ".vtk");
      softPalate.setName("softPalate");
      srcPalate = softPalate.getSurfaceMesh().clone();
      FixedMeshBody src = new FixedMeshBody(srcPalate);
      RenderProps.setFaceColor(src, Color.RED);
      RenderProps.setAlpha(src, 0.2);
      addRenderable(src);
      
      // target
      targetPalate = (PolygonalMesh)(WavefrontReader.read(geometryDir + "/palateSurface_rst.obj"));
      FixedMeshBody target = new FixedMeshBody(targetPalate);
      target.setName("target");
      RenderProps.setFaceColor(target, Color.CYAN);
      RenderProps.setAlpha(target, 0.5);
      addRenderable(target);
   }
   
   void setupRegistration() {
      
      // bounding FEM
      srcBoundingFem = createVoxelFem(null, srcPalate, 5, 0.002);
      FemMeshComp defPalate = srcBoundingFem.addMesh(srcPalate.clone());
      
      // muscle points
      muscleMap = new LinkedHashMap<>();
      for (MuscleBundleData m : mbd) {
         for (MusclePoint mp : m.musclePoints) {
            // inside soft palate
            if (mp.modelName.equals("softPalate")) {
               FemMarker fm = srcBoundingFem.addMarker(mp.loc);
               muscleMap.put(mp, fm);
            }
         }
      }
      
      RenderProps.setPointColor(srcBoundingFem.markers(), Color.GREEN);
      RenderProps.setPointRadius(srcBoundingFem.markers(), 0.0005);
      RenderProps.setPointStyle(srcBoundingFem.markers(), PointStyle.SPHERE);
      
      // affine transform
      // AffineTransform3d afft = CPD.affine(targetPalate, srcPalate, 0.01, 1e-6, 100);
      // System.out.println("Computed affine transform: ");
      // System.out.println(afft);
      
      // computed using CPD:
      AffineTransform3d afft = new AffineTransform3d( 0.6401502873449065, 0.024893147195155025, -0.11089839564812509, 0.049557232959660885,
         -0.03228654637510117, 0.9017153165799615, 0.014383239040616613, 0.0022767785414011528,
         0.1484616038837665, -0.06814493589692877, 0.8382888679319394, -0.0033379114066521415);
     
      // transform palate
      srcBoundingFem.transformGeometry(afft);
      
      // deformable
      mechModel.setGravity(0, 0, 0);
      mechModel.addModel(srcBoundingFem);
      RenderProps.setFaceColor(defPalate, Color.BLUE);
      RenderProps.setAlpha(defPalate, 0.5);
      RenderProps.setVisible(srcBoundingFem.getElements(), false);
      
      // parameters
      final double REG_YOUNGS = 20;
      final double REG_POISSON = 0.1;
      final double REG_DENSITY = 1000;
      final double REG_SIGMA = 0.2;
      final boolean REG_PERP = true;
      final double REG_FORCE = 1000;
      
      DynamicRegistrationController reg = new DynamicRegistrationController (mechModel);
      GaussianWeightFunction gwf = new GaussianWeightFunction (1, REG_SIGMA, REG_PERP);
      ICPMeshCorrespondence icp = new ICPMeshCorrespondence ();
      icp.setWeightFunction (gwf);
      reg.addRegistrationTarget (defPalate, targetPalate, 1.0, icp);
      reg.setForceScaling (REG_FORCE);
      reg.setName("registration");
      addController(reg);
      
      //      PressureMeshRegistrationForce prf = new PressureMeshRegistrationForce ();
      //      GaussianPressureFunction gpf = new GaussianPressureFunction (REG_PRESSURE, REG_SIGMA, REG_SNAP, REG_PERP);
      //      prf.setPressureFunction (gpf);
      //      MeshRegistrationController comp = new MeshRegistrationController(defPalate, targetPalate, prf);
         
      srcBoundingFem.setDensity(REG_DENSITY);
      srcBoundingFem.setLinearMaterial(REG_YOUNGS, REG_POISSON, true);
      
   }

   void loadMuscleInfo() {
      mbd = new MuscleBundleData[6];
      String[] muscles = {"LevatorVeliPalitini", "MusculusUvulae", "PalatoglossusAnterior", "PalatoglossusPosterior", "Palatopharyngeus", "TensorVeliPalitini"};

      for (int i=0; i<muscles.length; ++i) {
         mbd[i] = FrankMuscles.readMuscleBundle(muscleDir + muscles[i] + "_R.txt");
      }
   }
   
   public static FemModel3d createVoxelFem(
      FemModel3d fem, PolygonalMesh mesh, int minRes, double maxElemWidth) {
      if (fem == null) {
         fem = new FemModel3d();
      }

      OBB obb = new OBB(mesh);
      // fem from OBB
      Vector3d hw = new Vector3d(obb.getHalfWidths());
      double dz = 2 * hw.z / minRes;
      if (dz > maxElemWidth) {
         dz = maxElemWidth;
      }
      // add a little bit
      // hw.add(dz / 8, dz / 8, dz / 8);

      int[] res = new int[3];
      res[0] = (int)(Math.round(2 * hw.x / dz));
      res[1] = (int)(Math.round(2 * hw.y / dz));
      res[2] = (int)(Math.round(2 * hw.z / dz));

      FemFactory.createHexGrid(
         fem, 2 * hw.x, 2 * hw.y, 2 * hw.z, res[0], res[1], res[2]);
      fem.transformGeometry(obb.getTransform());

      double dx, dy;
      dx = 2 * hw.x / res[0];
      dy = 2 * hw.y / res[1];
      dz = 2 * hw.z / res[2];
      double r = 1.05 * Math.sqrt(dx * dx + dy * dy + dz * dz);

      // outside and farther than r
      BVFeatureQuery query = new BVFeatureQuery();

      HashSet<FemNode3d> deleteThese = new HashSet<FemNode3d>();
      for (FemNode3d node : fem.getNodes()) {
         boolean inside =
            query.isInsideOrientedMesh(mesh, node.getPosition(), r);
         if (!inside) {
            deleteThese.add(node);
         }
      }

      // remove elements/nodes
      for (FemNode3d node : deleteThese) {
         // remove element dependencies
         ArrayList<FemElement3d> elems =
            new ArrayList<>(node.getElementDependencies());
         for (FemElement3d elem : elems) {
            fem.removeElement(elem);
         }
         // remove node
         fem.removeNode(node);
      }

      // remove un-necessary nodes
      deleteThese.clear();
      for (FemNode3d node : fem.getNodes()) {
         if (node.getElementDependencies().size() < 1) {
            deleteThese.add(node);
         }
      }
      for (FemNode3d node : deleteThese) {
         fem.removeNode(node);
      }

      return fem;
   }

}
