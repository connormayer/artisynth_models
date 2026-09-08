//20260803 Cyd - package artisynth.models.jawTongue; (from Badin)
package artisynth.models.frank3;

import java.io.IOException;

//20260803 Cyd - public class BadinJawHyoidTonguePositionActivated extends BadinJawHyoidTonguePosition {

//20280829 Shitong - Modified according to BadinJawHyoidTongue,
// now directly extending FrankModel3Position
public class FrankModel3PositionActivated extends FrankModel3Position {

   public FrankModel3PositionActivated () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      collideTongueMaxilla = true;
      collideTongueJaw = true;
      super.build (args);
   }
}
