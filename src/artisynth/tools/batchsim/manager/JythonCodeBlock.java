package artisynth.tools.batchsim.manager;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.concurrent.Semaphore;

import artisynth.core.gui.jythonconsole.ArtisynthJythonConsole;
import artisynth.tools.batchsim.manager.PropertySpecification.PhonyPropValue;
import maspack.util.IndentingPrintWriter;

/**
 * See the official documentation in artisynth_models/doc/batchsim for details.
 * 
 * @author Francois Roewer-Despres
 */
public class JythonCodeBlock implements Printable {

   protected String myCode;
   protected BatchManager myManager;
   protected Boolean myReturnValue = null;
   protected ArtisynthJythonConsole myConsole;
   protected List<PhonyPropValue> myCurrentTask;
   protected Semaphore mySem = new Semaphore (0);

   /**
    * Creates a new {@link JythonCodeBlock} with the given code, and the Jython
    * console in which to later execute the code.
    * 
    * @param code
    * the Jython code
    * @param console
    * the Jython console
    */
   public JythonCodeBlock (BatchManager manager, String code,
   ArtisynthJythonConsole console) {
      myCode = code;
      myManager = manager;
      myConsole = console;
   }

   /**
    * Executes the Jython code, taking the current task into account, and
    * returns the return value of the code.
    * 
    * @param task
    * the current (partial) task
    * @return the value of the Jython code
    */
   public boolean check (List<PhonyPropValue> task) {
      synchronized (this) {
         myCurrentTask = task;
      }
      myConsole.getConsole ().set ("supervisor", this);
      String script =
         "_interpreter_.set('get', supervisor.get)\n"
         + "_interpreter_.set('return_value', supervisor.returnValue)\n"
         + myCode;
      InputStream input =
         new ByteArrayInputStream (script.getBytes (StandardCharsets.UTF_8));
      myConsole.execfile (input, "Jython Code Block");
      try {
         mySem.acquire ();
      }
      catch (InterruptedException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
      boolean returnValue = myReturnValue;
      myReturnValue = null;
      return returnValue;
   }

   /**
    * Returns the current value of the given property path as a string.
    * 
    * @param propPath
    * the property path
    * @return the current value of the property path
    * @throws IllegalArgumentException
    * if the given path does not correspond to a known
    * {@link PropertySpecification} or the value is not set because the given
    * path corresponds to a {@code PropertySpecification} that was not defined
    * before those in the corresponding redef block
    */
   synchronized public String get (String propPath)
      throws IllegalArgumentException {
      for (PhonyPropValue ppv : myCurrentTask) {
         if (ppv.propPath.equals (propPath)) {
            return ppv.value;
         }
      }
      for (PropertySpecification propSpec : myManager.myPropertySpecifications) {
         if (propSpec.getPropertyPath ().equals (propPath)) {
            throw new IllegalArgumentException (
               "property path \"" + propPath + "\" cannot appear in the when "
               + "block of a redef statement if it was not defined *before* all"
               + " the property paths listed in the corresponding redef block");
         }
      }
      throw new IllegalArgumentException (
         "property path \"" + propPath + "\" was not previously defined");
   }

   /**
    * Sets the return value to be returned at a later time to the given value.
    * 
    * @param value
    * the new return value
    * @throws IllegalStateException
    * if called multiple times within a single instance of code execution
    */
   public void returnValue (boolean value) throws IllegalStateException {
      if (myReturnValue == null) {
         myReturnValue = value;
         mySem.release ();
      }
      else {
         throw new IllegalStateException (
            "`returnValue()' called multiple times");
      }
   }

   @Override
   public void print (IndentingPrintWriter writer) {
      writer.println ("jython");
      writer.addIndentation (2);
      for (String line : myCode.split ("\n")) {
         writer.println ("$" + line);
      }
      writer.removeIndentation (2);
   }

   @Override
   public String toString () {
      return Utils.printableToString (this);
   }

}
