package artisynth.models.neckModel;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.properties.PropertyList;
import maspack.spatialmotion.Twist;
import maspack.spatialmotion.Wrench;
import artisynth.core.materials.*;

public class InterveterbralJointFM extends FrameMaterial{
   protected Vector3d myK = new Vector3d();
   protected Vector3d myD = new Vector3d();
   protected Vector3d myRotK = new Vector3d();
   protected Vector3d myRotD = new Vector3d();
   protected double [] myFEPoly = new double [] {0, 0, 1, 0};
   protected double [] myLBPoly = new double [] {0, 0, 1, 0};
   protected double [] myARPoly = new double [] {0, 0, 1, 0};

   public static PropertyList myProps =
      new PropertyList (OffsetLinearFrameMaterial.class, FrameMaterial.class);

   static {
      myProps.add ("stiffness * *", "offset linear spring stiffness", Vector3d.ZERO);
      myProps.add ("damping * *", "offset linear spring damping", Vector3d.ZERO);
      myProps.add ("rotaryDamping * *", "offset linear spring damping", Vector3d.ZERO);
   }   

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }  

   public Vector3d getStiffness () {
      return myK;
   }

   public void setStiffness (double k) {
      myK.set (k, k, k);
   }

   public void setStiffness (double kx, double ky, double kz) {
      myK.set (kx, ky, kz);
   }

   public void setStiffness (Vector3d kvec) {
      myK.set (kvec);
   }

   public Vector3d getDamping() {
      return myD;
   }

   public void setDamping (double d) {
      myD.set (d, d, d);
   }

   public void setDamping (double dx, double dy, double dz) {
      myD.set (dx, dy, dz);
   }

   public void setDamping (Vector3d dvec) {
      myD.set (dvec);
   }

   public Vector3d getRotaryDamping() {
      return myRotD;
   }

   public void setRotaryDamping (double d) {
      myRotD.set (d, d, d);
   }

   public void setRotaryDamping (double dx, double dy, double dz) {
      myRotD.set (dx, dy, dz);
   }

   public void setRotaryDamping (Vector3d dvec) {
      myRotD.set (dvec);
   }
   
   public double [] getFEPoly () {
      return myFEPoly;
   }
   
   public void setFEPoly (double [] argPoly) {
      myFEPoly = argPoly;
   }
   
   public void setFEPoly (double arg3, double arg2, double arg1, double arg0) {
         myFEPoly[0] = arg3;
         myFEPoly[1] = arg2;
         myFEPoly[2] = arg1;
         myFEPoly[3] = arg0;
   }
   
   public double [] getLBPoly() {
      return myLBPoly;
   }
   
   public void setLBPoly (double [] argPoly) {
      myLBPoly = argPoly;
   }
   
   public void setLBPoly (double arg3, double arg2, double arg1, double arg0) {
         myLBPoly[0] = arg3;
         myLBPoly[1] = arg2;
         myLBPoly[2] = arg1;
         myLBPoly[3] = arg0;
   }
   
   public double [] getARPoly () {
      return myARPoly;
   }
   
   public void setARPoly (double [] argPoly) {
      myARPoly = argPoly;
   }
   
   public void setARPoly (double arg3, double arg2, double arg1, double arg0) {
         myARPoly[0] = arg3;
         myARPoly[1] = arg2;
         myARPoly[2] = arg1;
         myARPoly[3] = arg0;
   }

   public InterveterbralJointFM () {
      this (0, 0, 0);
   }

   public InterveterbralJointFM (double k, double d, double dr) {
      setStiffness (k);
      setDamping (d);
      setRotaryDamping (dr);
   }

   public void computeF (
      Wrench wr, RigidTransform3d X21, Twist vel21, 
      RigidTransform3d initialX21) {

      Vector3d p = X21.p;
      Vector3d initialp = initialX21.p;

      // use these matrix entries as small angle approximations to 
      // the rotations about x, y, and z
      double sx =  X21.R.m21;
      double sy = -X21.R.m20;
      double sz =  X21.R.m10;
      double absx = Math.abs (sx);
      double absy = Math.abs (sy);
      double absz = Math.abs (sz);

      wr.f.x = myK.x*(p.x - initialp.x );
      wr.f.y = myK.y*(p.y - initialp.y );
      wr.f.z = myK.z*(p.z - initialp.z );

      wr.m.x = calculateWr(Math.toDegrees (absx), myFEPoly);
      wr.m.y = calculateWr(Math.toDegrees (absy), myLBPoly);
      wr.m.z = calculateWr(Math.toDegrees (absz), myARPoly);

      Vector3d v = vel21.v;
      Vector3d w = vel21.w;

      wr.f.x += myD.x*v.x;
      wr.f.y += myD.y*v.y;
      wr.f.z += myD.z*v.z;

      wr.m.x += myRotD.x*w.x;
      wr.m.y += myRotD.y*w.y;
      wr.m.z += myRotD.z*w.z;
   }

   public void computeDFdq (
      Matrix6d Jq, RigidTransform3d X21, Twist vel21, 
      RigidTransform3d initialX21, boolean symmetric) {

      Jq.setZero();

      RotationMatrix3d R = X21.R;
      // use these matrix entries as small angle approximations to 
      // the rotations about x, y, and z
      double sx =  R.m21;
      double sy = -R.m20;
      double sz =  R.m10;
      double absx = Math.abs (sx);
      double absy = Math.abs (sy);
      double absz = Math.abs (sz);

      Jq.m00 = myK.x;
      Jq.m11 = myK.y;
      Jq.m22 = myK.z;
      
      Matrix3d RJq = new Matrix3d();
      Jq.getSubMatrix33 (RJq);
      
      RJq.set (0, 0, calculateDWrdq (Math.toDegrees (absx), myFEPoly));
      RJq.set (1, 1, calculateDWrdq (Math.toDegrees (absy), myLBPoly));
      RJq.set (2, 2, calculateDWrdq (Math.toDegrees (absz), myARPoly));
      
      Matrix3d temR = new Matrix3d();
      temR.m00 = R.m11;
      temR.m11 = R.m00;
      temR.m22 = R.m00;
      if (!symmetric) {
         temR.m10 = -R.m10;
         temR.m01 = -R.m01;
         temR.m20 = -R.m20;
      }
      RJq.mul (temR);
      Jq.setSubMatrix33 (RJq);
   }

   public void computeDFdu (
      Matrix6d Ju, RigidTransform3d X21, Twist vel21, 
      RigidTransform3d initialX21, boolean symmetric) {

      Ju.setZero();

      Ju.m00 = myD.x;
      Ju.m11 = myD.y;
      Ju.m22 = myD.z;
      
      Ju.m33 = myRotD.x;
      Ju.m44 = myRotD.y;
      Ju.m55 = myRotD.z;
   }

   public boolean equals (FrameMaterial mat) {
      return true;
   }
   
   public double calculateWr(double argMotion, double [] argPoly) {
      Vector4d temPoly = new Vector4d ();
      Vector4d temMotion = new Vector4d ();
      double temWr = 0;
      
      temPoly.set(argPoly);
      temMotion.set (Math.pow (argMotion, 3), Math.pow (argMotion, 2), argMotion, 1);
      temWr = temMotion.dot (temPoly);
      if (argMotion < 0) {
         temWr = -temWr;
      }
      
      return temWr;
   }
   
   public double calculateDWrdq(double argMotion, double [] argPoly) {
      Vector3d temPoly = new Vector3d ();
      Vector3d temMotion = new Vector3d();
      double temDWrdq = 0;
      
      temPoly.set (argPoly[0], argPoly[1], argPoly[2]);
      temMotion.set (3*Math.pow (argMotion, 2), 2*argMotion, 1);
      temDWrdq = temMotion.dot (temPoly);
      if (argMotion < 0) {
         temDWrdq = - temDWrdq;
      }
      temDWrdq = temDWrdq * Math.toDegrees (1);
      
      return temDWrdq;
   }
   
   public InterveterbralJointFM clone() {
      InterveterbralJointFM mat = (InterveterbralJointFM)super.clone();
      mat.myK = new Vector3d (myK);
      mat.myRotK = new Vector3d (myRotK);
      mat.myD = new Vector3d (myD);
      mat.myRotD = new Vector3d (myRotD);
      return mat;
   }
}
