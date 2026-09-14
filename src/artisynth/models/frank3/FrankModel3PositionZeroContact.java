/*
20260911 Shitong
Added as a new class for a condition
without any contact / collision between tongue and structures
including soft palate
 */
package artisynth.models.frank3;

import java.io.IOException;


//20260803 Cyd - public class BadinJawHyoidTonguePositionDeactivated extends BadinJawHyoidTonguePosition {

//20280829 Shitong - Modified according to BadinJawHyoidTongue,
// now directly extending FrankModel3Positiojn
public class FrankModel3PositionZeroContact extends FrankModel3Position {

   public FrankModel3PositionZeroContact () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      collideTongueMaxilla = false;
      collideTongueJaw = false;
      collideTongueSoftPalate = false; //All are off
      super.build (args);
   }
}
