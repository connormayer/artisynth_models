'''
# @author: Connor Mayer
# Ported for FrankModel3 batch simulation.
'''
from artisynth.models.frank3 import FrankModel3BatchWorker
from jarray import array
import sys

args = array(sys.argv, String)
worker = FrankModel3BatchWorker(args)
worker.run()
