package artisynth.models.neckModel;

import artisynth.core.materials.MasoudMillardLAM;
import maspack.properties.PropertyList;

public class MasoudMillardLAMExt extends MasoudMillardLAM{

   public MasoudMillardLAMExt () {
      super ();
   }

   public MasoudMillardLAMExt (double penAngleLit, double sarcomereLenLit,
   double fiberRatio) {
      super (penAngleLit, sarcomereLenLit, fiberRatio);
   }

   public static PropertyList myProps = new PropertyList (
      MasoudMillardLAMExt.class, MasoudMillardLAM.class);
   
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
   public void setOptimalSarcomereLen (double optSarLen) {
      optimalSarcomereLen = optSarLen;
   }
   
   public double getOptimalSarcomereLen () {
      return optimalSarcomereLen;
   }
   
   public void setMatrialParameters(double [] args) {
      setMaxForce (args[0]);
      setMyFMTratioLit(args[1]);
      setPassiveFraction(args[2]);
      setPenAngleLit (args[3]);
      setMySarcomereLenLit (args[4]);
      setOptimalSarcomereLen (args[5]);
      setDamping (args[6]);
      setForceScaling (args[7]);
   }
   
   public double [] getMaterialParameters() {
      
      double [] args = new double [8];
      
      args[0] = getMaxForce ();
      args[1] = getMyFMTratioLit();
      args[2] = getPassiveFraction();
      args[3] = getPenAngleLit ();
      args[4] = getMySarcomereLenLit ();
      args[5] = getOptimalSarcomereLen ();
      args[6] = getDamping ();
      args[7] = getForceScaling ();
      
      return args;
   }
   

   
   
}
