package artisynth.models.modelOrderReduction;

import java.io.IOException;

import maspack.matrix.Matrix;
import maspack.matrix.Vector;

public class PrintData {

   public static void printMatrix(Matrix M) {
      for (int i = 0; i < M.rowSize (); i ++) {
         for (int j = 0; j < M.colSize (); j ++) {
            System.out.printf ("%.8f ", M.get (i, j));
         }
         System.out.print("\n");
      }
   }

   public static void printVector(Vector vec)  {
      for (int i = 0; i < vec.size (); i ++) {
         System.out.printf ("%.8f ", vec.get (i));
      }
      System.out.println ("");
   }

}
