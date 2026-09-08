//20260803 Cyd - package artisynth.models.jawTongue; (from Badin)
package artisynth.models.frank3;

import java.io.IOException;


//20260803 Cyd - public class BadinJawHyoidTonguePositionDeactivated extends BadinJawHyoidTonguePosition {

//20280829 Shitong - Modified according to BadinJawHyoidTongue,
// now directly extending FrankModel3Positiojn
public class FrankModel3PositionDeactivated extends FrankModel3Position {

   public FrankModel3PositionDeactivated () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      collideTongueMaxilla = false;
      collideTongueJaw = false;
      super.build (args);
   }
}
