from artisynth.models.frank2 import FrankModel2BatchWorker
from jarray import array
import sys


args = array(sys.argv, String)
print(args)
worker = FrankModel2BatchWorker(args)
worker.run()
