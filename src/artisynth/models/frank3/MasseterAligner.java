package artisynth.models.frank3;

import java.io.IOException;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import maspack.geometry.ICPRegistration;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;

public class MasseterAligner extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      String jawTargetFileName = ArtisynthPath.getSrcRelativePath(this, "geometry/masseter/jaw_frank.obj");
      String jawSourceFileName = ArtisynthPath.getSrcRelativePath(this, "geometry/masseter/jaw_smooth_5000.obj");
      
      String maxillaTargetFileName = ArtisynthPath.getSrcRelativePath(this, "geometry/masseter/maxilla_frank.obj");
      String maxillaSourceFileName = ArtisynthPath.getSrcRelativePath(this, "geometry/masseter/maxilla_smooth_5000.obj");
      
      PolygonalMesh jawTarget =  new PolygonalMesh(jawTargetFileName);
      PolygonalMesh jawSource =  new PolygonalMesh(jawSourceFileName);
   
      FixedMeshBody jawTargetBody = new FixedMeshBody("jaw target", jawTarget);
      FixedMeshBody jawSourceBody = new FixedMeshBody("jaw source", jawSource);
      
      PolygonalMesh maxillaTarget =  new PolygonalMesh(maxillaTargetFileName);
      PolygonalMesh maxillaSource =  new PolygonalMesh(maxillaSourceFileName);
   
      FixedMeshBody maxillaTargetBody = new FixedMeshBody("maxilla target", maxillaTarget);
      FixedMeshBody maxillaSourceBody = new FixedMeshBody("maxilla source", maxillaSource);
      
      addRenderable(jawTargetBody);
      addRenderable(jawSourceBody);
      
      addRenderable(maxillaTargetBody);
      addRenderable(maxillaSourceBody);
      
      // affine registration of jaw
      // AffineTransform3d jawTrans = CPD.affine (jawTarget, jawSource, 0.01, 1e-12, 100);
      // System.out.println("Original: " + jawTrans.toString());
      
      ICPRegistration icp = new ICPRegistration();
      AffineTransform3d jtrans = new AffineTransform3d();
      icp.registerICP(jtrans, jawSource, jawTarget, 12);
      jtrans.invert();
      System.out.println("Jaw transform: ");
      System.out.println(jtrans);
      
      PolygonalMesh jawRegistered = jawSource.copy();
      jawRegistered.transform(jtrans);
      FixedMeshBody jawRegisteredBody = new FixedMeshBody("jaw registered", jawRegistered);
      addRenderable(jawRegisteredBody);
      
      
      // transform by jaw transform first for better initial position
      PolygonalMesh maxillaRegistered = maxillaSource.copy();
      
      // jaw transform:
      AffineTransform3d jawTransform = new AffineTransform3d(
      0.002495409373738498, 1.0433426405018236, -0.10340864144558273, 0.11151730641954215,
        -0.9710849277815373, 0.009060351596940392, -0.01259028768718018, 7.130175836106193E-4,
        -4.744934681457862E-4, 0.028839862397976635, 1.096312265454904, 0.04873678936407018);
      
      //      ScaledRigidTransform3d mtrans2 = CPD.rigid(maxillaTarget, maxillaRegistered, 0.1, 1e-2, 100, false);
      //      maxillaRegistered.transform(mtrans2);
      //      System.out.println(mtrans2);
      
//      AffineTransform3d mtrans2 = MeshICP.align(maxillaTarget, maxillaRegistered, AlignmentType.RIGID);
//      maxillaRegistered.transform(mtrans2);
//      System.out.println(mtrans2);

      Point3d t = new Point3d(-0.00302142, 0, 0.00450815);
      AxisAngle r = new AxisAngle(0, -1, 0, Math.toRadians(0.7223));
      AffineTransform3d maxillaTransform = new AffineTransform3d();
      maxillaTransform.setTranslation(t);
      maxillaTransform.setRotation(r);
      maxillaTransform.mul(jawTransform);
      maxillaRegistered.transform(maxillaTransform);
      
      FixedMeshBody maxillaRegisteredBody = new FixedMeshBody("maxilla registered", maxillaRegistered);
      addRenderable(maxillaRegisteredBody);
    
      // cranium
      String craniumTargetFileName = ArtisynthPath.getSrcRelativePath(this, "geometry/masseter/cranium_frank.obj");
      String craniumSourceFileName = ArtisynthPath.getSrcRelativePath(this, "geometry/masseter/cranium_smooth.obj");
      
      PolygonalMesh craniumTarget =  new PolygonalMesh(craniumTargetFileName);
      PolygonalMesh craniumSource =  new PolygonalMesh(craniumSourceFileName);
   
      FixedMeshBody craniumTargetBody = new FixedMeshBody("cranium target", craniumTarget);
      FixedMeshBody craniumSourceBody = new FixedMeshBody("cranium source", craniumSource);
      
      addRenderable(craniumTargetBody);
      addRenderable(craniumSourceBody);
      
      AffineTransform3d ctrans = new AffineTransform3d();
      icp.registerICP(ctrans, craniumSource, craniumTarget, 12);
      ctrans.invert();
      System.out.println("Cranium transform: ");
      System.out.println(ctrans);
      
      // transform by jaw transform first for better initial position
      PolygonalMesh craniumRegistered = craniumSource.copy();
      craniumRegistered.transform(ctrans);
      FixedMeshBody craniumRegisteredBody = new FixedMeshBody("cranium registered", craniumRegistered);
      addRenderable(craniumRegisteredBody);
      
   }
}
