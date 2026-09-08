# iu_blend_demo.py
# ---------------------------------------------------------------------------
# Adds COMBINED excitation probes to an ALREADY-LOADED FrankModel2, at runtime.
# Nothing is recompiled and no Java source is modified.
#
# HOW TO RUN
#   1. Start ArtiSynth, load FrankModel2.
#   2. File > Run Script...  ->  pick this file  (or paste into Jython console).
#   3. Press Play.
#
# HOW TO UNDO
#   Reload the model, re-run this script, or in the console:
#       root.removeAllInputProbes(); root.removeAllWayPoints()
#
# TIMELINE   time = [0.0, 0.10, 0.50, 1.00],  values = [0, 0, tgt, tgt]
#   0.00-0.10  rest
#   0.10-0.50  ramp up
#   0.50       full blended target reached  <-- peak load; inversion shows here
#   0.50-1.00  hold (tongue settles into the combined posture)
#
# TWO TARGET SETS -- switch with the `targets = ...` line below.
#   TARGETS_CODE : /i/ from probe_snd_i, /u/ from probe_snd_u (as-shipped).
#   TARGETS_LIT  : /i/ from probe_snd_i_partially_stavness_0720Cyd (Stavness2012-
#                  based), /u/ still from probe_snd_u.
#                  Adds GGA + VERT (absent from probe_snd_i), keeps the face
#                  (RIS/ZYG) and palate (LVP) drives, and lowers GGP 0.35->0.25
#                  and STY 0.18->0.09.
#                  Gentler main driver => you can usually raise SCALE.
# ---------------------------------------------------------------------------

from jarray import array
from artisynth.core.driver import Main
from artisynth.models.frank2 import FrankActivations
from maspack.interpolation import Interpolation
Order = Interpolation.Order

W = 0.5     # blend weight (method 1): 1.0=/i/, 0.0=/u/


SCALE = 0.5   # global multiplier on every target (lower = safer). 0.3-0.6 is a safer interval.


# ============================================================================
# COMBINE METHODS  --  uncomment EXACTLY ONE return block, comment the rest.
# ============================================================================
def combine(name, ai, au):

    # --- (1) WEIGHTED BLEND -------------------------------------------------
    return min(1.0, W * ai + (1.0 - W) * au)

    # --- (2) SUM (literal superposition) ------------------------------------
    #return min(1.0, ai + au)

    # --- (3) MAX (envelope) -------------------------------------------------
    #     Whichever vowel wants the muscle more.
    # return max(ai, au)
# ============================================================================


# ============================================================================
# TARGET SETS      muscle : (a_i, a_u)
# ============================================================================

# As-shipped from Frank2: /i/ from probe_snd_i, /u/ from probe_snd_u.
TARGETS_CODE = {
    "GGP":  (0.35, 0.10),   
    "GGM":  (0.08, 0.03),
    "STY":  (0.18, 0.15),   
    "TRANS":(0.14, 0.10),
    "SL":   (0.05, 0.07),
    "IL":   (0.20, 0.00),   
    "MH":   (0.08, 0.00),   
    "RIS":  (0.05, 0.00),   
    "ZYG":  (0.05, 0.00),
    "OOP":  (0.00, 0.40),  
    "LVP":  (0.10, 0.15),
}

# probe_snd_i_partially_stavness_0720Cyd /i/  +  as-shipped probe_snd_u /u/.
# Changes vs TARGETS_CODE on the /i/ side:
#   GGP  0.35 -> 0.25
#   GGM  0.08 -> 0.115
#   GGA   --  -> 0.025
#   SL   0.05 -> 0.125
#   MH   0.08 -> 0.04
#   STY  0.18 -> 0.09
#   TRANS0.14 -> 0.095
#   VERT  --  -> 0.05
#   IL   0.20 -> 0.20   (unchanged)
#   RIS  0.05 -> 0.05   (unchanged)
#   ZYG  0.05 -> 0.05   (unchanged)
#   LVP  0.10 -> 0.10   (unchanged)
TARGETS_LIT = {
    "GGP":  (0.250, 0.10),
    "GGM":  (0.115, 0.03),
    "GGA":  (0.025, 0.00),
    "SL":   (0.125, 0.07),
    "IL":   (0.200, 0.00),
    "MH":   (0.040, 0.00),
    "STY":  (0.090, 0.15),
    "TRANS":(0.095, 0.10),
    "VERT": (0.050, 0.00),
    "RIS":  (0.050, 0.00),
    "ZYG":  (0.050, 0.00),
    "OOP":  (0.000, 0.40),
    "LVP":  (0.100, 0.15),
}

# ---- PICK ONE ----
targets = TARGETS_LIT
# targets = TARGETS_CODE

ADD_JC = True          # set False to drop JC
JC_i, JC_u = 0.02, 0.00
# ============================================================================


root     = Main.getMain().getRootModel()
exciters = root.getAllExciters()

# clean slate (reversible)
root.removeAllInputProbes()
root.removeAllWayPoints()

time = array([0.0, 0.10, 0.50, 1.00], 'd')   # rest, rest, reach, hold (ONE plateau)

added = 0
for name in targets:
    ai, au = targets[name]
    exc = FrankActivations.findExciter(exciters, name)
    if exc is None:
        print("skip (exciter not found): %s" % name)
        continue
    tgt = min(1.0, SCALE * combine(name, ai, au))
    vals = array([0.0, 0.0, tgt, tgt], 'd')
    p = FrankActivations.createMuscleProbe(exc, name, time, vals)
    p.setInterpolationOrder(Order.CubicStep)      # gentle onset -> avoids inversion
    root.addInputProbe(p)
    added += 1
    print("%-6s i=%.3f u=%.2f -> %.3f" % (name, ai, au, tgt))

# --- JC (jaw close): synthesized meta-exciter, added/reused separately ---
if ADD_JC:
    jc_tgt = min(1.0, SCALE * combine("JC", JC_i, JC_u))
    # reuse an existing JC exciter if a previous run already added one, so re-running the script does NOT throw a name collision
    jc = None
    for e in list(root.mechModel.getMuscleExciters()):
        if e.getName() == "mex_JawClose":
            jc = e
            break
    if jc is None:
        jc = FrankActivations.makeJawCloseExciter(exciters)
        root.mechModel.addMuscleExciter(jc)
    pj = FrankActivations.createMuscleProbe(
            jc, "JC", time, array([0.0, 0.0, jc_tgt, jc_tgt], 'd'))
    pj.setInterpolationOrder(Order.CubicStep)
    root.addInputProbe(pj)
    added += 1
    print("%-6s i=%.3f u=%.2f -> %.3f  (jaw close, synthesized)" % ("JC", JC_i, JC_u, jc_tgt))

t = 0.0
while t < 1.00:
    root.addWayPoint(t)
    t += 0.01
root.addBreakPoint(1.00)

setname = "LIT" if targets is TARGETS_LIT else "CODE"
print("i+u combined demo loaded: %d probes (%s targets, W=%.2f, SCALE=%.2f). Press Play." % (added, setname, W, SCALE))
