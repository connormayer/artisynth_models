'''
@author: Connor Mayer
'''
from artisynth.models.quantalLips import QuantalLipsBatchWorker
from jarray import array
import sys

args = array(sys.argv, String)
worker = QuantalLipsBatchWorker(args)
worker.run()
