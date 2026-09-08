package artisynth.models.frank3;

import java.io.IOException;
import java.util.LinkedList;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.editorManager.RemoveComponentsCommand;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import maspack.matrix.AxisAngle;
import maspack.matrix.DualQuaternion;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.StlRenderer;

public class Frank3NeckBrace extends FrankModel3 {

   RigidTransform3d craniumInitialPose;  // for computing cranium transform
   RigidTransform3d skullInitialPose;
   RigidTransform3d maxillaInitialPose;
   RigidTransform3d jawInitialPose;


   @Override
   public void build(String[] args) throws IOException {
      super.build(args);


      // disable all inverted element aborts
      for (FemModel3d fem : super.fems) {
         fem.setAbortOnInvertedElements(false);
         fem.setMaterial(new LinearMaterial(30000,0.4));
         RenderProps.setVisible(fem.getNodes(), false);
         RenderProps.setVisible(fem.getElements(), false);
      }
      
      mechModel.getCollisionManager().clear();

      // fix head components so we can move them around
      maxilla.setDynamic(false);
      cranium.setDynamic(false);
      RigidBody skull = neckRbs.get ("C0");
      skull.setDynamic(false);
       jaw.setDynamic(false);

      // make face follow default visibility
      RenderProps.setVisibleMode(face, PropertyMode.Inherited);

      addEyes();
      
      // remove all fems
      // RenderProps.setVisible(fems, false);
      // delete(fems);

      craniumInitialPose = new RigidTransform3d(cranium.getPose());
      skullInitialPose = new RigidTransform3d(skull.getPose());
      maxillaInitialPose = new RigidTransform3d(maxilla.getPose());
      jawInitialPose = new RigidTransform3d(jaw.getPose());

   }

  

   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);

      for (double t=0; t<1.0; t += 0.1) {
         super.addWayPoint(t);
      }
      super.addBreakPoint(1.0);
   }

   @Override
   public StepAdjustment advance(double t0, double t1, int flags) {

      // move head up slightly
      AxisAngle r = new AxisAngle(0, 1, 0, 0.1803292241045657);
      Vector3d v = new Vector3d(0.0122017768656664707, 0, 0.04239944377638194);
      RigidTransform3d rtrans = new RigidTransform3d(v, r);

      // interpolate transform, apply to skull
      double dt = Math.min(t1,  1);
      DualQuaternion dq = new DualQuaternion(rtrans);
      DualQuaternion dq0 = new DualQuaternion();
      DualQuaternion q = new DualQuaternion();
      q.dualQuaternionLinearBlending(dq0, dt, dq);
      q.getRigidTransform3d(rtrans);

      RigidTransform3d pose = new RigidTransform3d();

      RigidBody skull = neckRbs.get ("C0");
      pose.mul(rtrans, skullInitialPose);
      skull.setPose(pose);

      pose.mul(rtrans,maxillaInitialPose);
      maxilla.setPose(pose);

      pose.mul(rtrans,craniumInitialPose);
      cranium.setPose(pose);
      pose = new RigidTransform3d(cranium.getPose());

      pose.mul(rtrans,jawInitialPose);
      jaw.setPose(pose);

      return super.advance(t0, t1, flags);
   }


   void delete(ModelComponent components) {

      LinkedList<ModelComponent> comps = new LinkedList<>();
      comps.add(components);

      LinkedList<ModelComponent> update = new LinkedList<ModelComponent>();
      LinkedList<ModelComponent> delete = 
         ComponentUtils.findDependentComponents (update, comps);

      if (delete.size() > comps.size()) {
         // first, see if we can actually delete:
         System.out.println ("delete size=" + delete.size());
      }
      // components are deselected before they are removed. This
      // will not affect dependentSelection because it is a (possibly
      // extended) copy of the selection.
      RemoveComponentsCommand cmd = new RemoveComponentsCommand ("delete", delete, update);
      Main.getMain().getUndoManager().saveStateAndExecute (cmd);
   }

   @Override
   public void render(Renderer renderer, int flags) {
      super.render(renderer, flags);

      // get left/right eye poses
      //            RigidBody leye = mechModel.rigidBodies().get("left_eye");
      //            System.out.println("Left eye: " + leye.getPose().R.getAxisAngle() + ", " + leye.getPose().p);
      //            RigidBody reye = mechModel.rigidBodies().get("right_eye");
      //            System.out.println("Right eye: " + reye.getPose().R.getAxisAngle() + ", " + reye.getPose().p);
   }
   
   public void saveAsSTL() {
      
      // create a new render list
      RenderList rlist = new RenderList();
      rlist.addIfVisible(this);
      
      StlRenderer renderer = new StlRenderer();
      renderer.begin(ArtisynthPath.getTempDir() + "/frank.stl", true);
      int qid = 0;
      qid = rlist.renderOpaque(renderer, qid, 0);
      qid = rlist.renderTransparent(renderer, qid, 0);
      renderer.end();
      
   }

}
