# run_speech_a.py
# Batch (headless) simulation of the /a/ vowel for FrankModel2

loadModel('artisynth.models.frank2.FrankModel2')
rm = root()

from artisynth.models.frank2 import FrankActivations

tongue = rm.tongue
larynx = rm.larynx
externalMuscles = rm.mechModel.findComponent("ExternalMuscles")

FrankActivations.probe_snd_a_wu2014_0715Cyd(
    rm, rm.mechModel,
    tongue.getMuscleBundles(),
    externalMuscles,
    tongue,
    larynx
)  # 加母音 probe

rm.createSynthEmmaPoints()  # 這時候呼叫,才抓得到正確的 stop time

stopTime = rm.getInputProbes().get(rm.getInputProbes().size() - 1).getStopTime()
play(stopTime)
waitForStop()
quit()