package artisynth.models.frank2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Resolves a batch {@code activationOrder} pattern and per-muscle excitations
 * into absolute probe onset times. Only the relative order among muscles with
 * excitation &gt; {@code epsilon} matters; inactive muscles always receive onset 0.
 */
public class ActivationScheduleResolver {

   public static final String SIMULTANEOUS = "SIMULTANEOUS";

   /**
    * @param pattern activation order string from {@link FrankModel2#getActivationOrder()}
    * @param excitations muscle name to excitation level for muscles in the task
    * @param staggerInterval seconds between successive activations in a chain
    * @param epsilon excitation values at or below this are treated as inactive
    * @return muscle name to onset time (seconds)
    */
   public static Map<String, Double> resolve (
      String pattern,
      Map<String, Double> excitations,
      double staggerInterval,
      double epsilon) {

      List<String> active = activeMuscles (excitations, epsilon);
      Map<String, Double> onsets = zeroOnsets (excitations.keySet());

      if (active.isEmpty()) {
         return onsets;
      }
      if (active.size() == 1) {
         onsets.put (active.get (0), 0.0);
         return onsets;
      }

      String normalized = normalizePattern (pattern);
      if (isSimultaneous (normalized)) {
         for (String name : active) {
            onsets.put (name, 0.0);
         }
         return onsets;
      }

      List<String> order = parseOrder (normalized);
      validateOrderMatchesActive (order, active);

      for (int i = 0; i < order.size(); i++) {
         onsets.put (order.get (i), i * staggerInterval);
      }
      return onsets;
   }

   protected static List<String> activeMuscles (
      Map<String, Double> excitations, double epsilon) {
      List<String> active = new ArrayList<>();
      for (Map.Entry<String, Double> e : excitations.entrySet()) {
         if (e.getValue() > epsilon) {
            active.add (e.getKey());
         }
      }
      Collections.sort (active);
      return active;
   }

   protected static Map<String, Double> zeroOnsets (Set<String> muscleNames) {
      Map<String, Double> onsets = new LinkedHashMap<>();
      for (String name : muscleNames) {
         onsets.put (name, 0.0);
      }
      return onsets;
   }

   protected static String normalizePattern (String pattern) {
      if (pattern == null || pattern.trim().isEmpty()) {
         return SIMULTANEOUS;
      }
      return pattern.trim();
   }

   protected static boolean isSimultaneous (String pattern) {
      return SIMULTANEOUS.equalsIgnoreCase (pattern);
   }

   /**
    * Accepts {@code SL>GGP}, {@code SL_THEN_GGP}, or {@code HG_THEN_MH_THEN_STY}.
    */
   protected static List<String> parseOrder (String pattern) {
      if (pattern.contains (">")) {
         return splitMuscleNames (pattern, ">");
      }
      if (pattern.contains ("_THEN_")) {
         return splitMuscleNames (pattern, "_THEN_");
      }
      throw new IllegalArgumentException (
         "activation order pattern \"" + pattern
         + "\" must be SIMULTANEOUS or use '>' or '_THEN_' separators");
   }

   protected static List<String> splitMuscleNames (String pattern, String sep) {
      String[] parts = pattern.split (java.util.regex.Pattern.quote (sep));
      List<String> names = new ArrayList<>();
      for (String part : parts) {
         String name = part.trim();
         if (name.isEmpty()) {
            throw new IllegalArgumentException (
               "empty muscle name in activation order pattern \"" + pattern + "\"");
         }
         names.add (name);
      }
      if (names.size() < 2) {
         throw new IllegalArgumentException (
            "activation order pattern \"" + pattern
            + "\" must list at least two muscles");
      }
      return names;
   }

   protected static void validateOrderMatchesActive (
      List<String> order, List<String> active) {
      Set<String> orderSet = new HashSet<>(order);
      Set<String> activeSet = new HashSet<>(active);

      if (orderSet.size() != order.size()) {
         throw new IllegalArgumentException (
            "activation order lists muscle \"" + duplicateName (order)
            + "\" more than once");
      }
      if (!orderSet.equals (activeSet)) {
         throw new IllegalArgumentException (
            "activation order " + order + " does not match active muscles "
            + active);
      }
   }

   protected static String duplicateName (List<String> order) {
      Set<String> seen = new HashSet<>();
      for (String name : order) {
         if (!seen.add (name)) {
            return name;
         }
      }
      return order.get (0);
   }
}
