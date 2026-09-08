# run_speech_a.py
# Batch (headless) simulation of the /a/ vowel for FrankModel2

loadModel('artisynth.models.frank2.FrankModel2')
rm = root()

from artisynth.models.frank2 import FrankActivations
allExciters = rm.getAllExciters()
FrankActivations.probe_snd_u(rm, rm.mechModel, allExciters)  # 先加母音 probe

rm.createSynthEmmaPoints()  # 這時候呼叫,才抓得到正確的 stop time

stopTime = rm.getInputProbes().get(rm.getInputProbes().size() - 1).getStopTime()
play(stopTime)
waitForStop()
quit()