# run_speech_a.py
# Batch (headless) simulation of the /a/ vowel for FrankModel2
#
# Usage (cmd line):
#   artisynth -script run_speech_a.py [ 0.223 0.386 ]
# Note: in ArtiSynth's -script Jython environment, sys.argv is exactly the
# list of strings placed inside [ ] -- it does NOT have the script's own
# path prepended like normal Python does. So we use sys.argv directly.
customTimes = list(sys.argv)
print("Requested custom output times: %s" % customTimes)
loadModel('artisynth.models.frank2.FrankModel2', *customTimes)
rm = root()

from artisynth.models.frank2 import FrankActivations
allExciters = rm.getAllExciters()
FrankActivations.probe_snd_a(rm, rm.mechModel, allExciters)  # 先加母音 probe

rm.createSynthEmmaPoints()  # 這時候呼叫,才抓得到正確的 stop time

stopTime = rm.getInputProbes().get(rm.getInputProbes().size() - 1).getStopTime()

# Play to each requested custom time in turn, exporting geometry the instant
# the sim actually stops there. play(t) reliably lands exactly on t (it's
# the same mechanism the final play(stopTime) below relies on), so there's
# no need to match floating point times inside advance() anymore, and you're
# free to pick any time you want - it does not need to be a multiple of 0.005.
outputTimes = list(rm.getCustomSaveTimes())
for t in outputTimes:
    play(t)
    waitForStop()
    rm.exportGeometry(t)

# run out the rest of the simulation to the final probe stop time
play(stopTime)
waitForStop()
quit()