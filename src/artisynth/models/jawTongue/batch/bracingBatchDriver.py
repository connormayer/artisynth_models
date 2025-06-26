'''
# @author: Connor Mayer
'''
from artisynth.models.jawTongue import BadinJawHyoidTonguePositionBatchWorker
from jarray import array
import sys

print(sys.argv)
args = array(sys.argv, String)
print(args)
worker = BadinJawHyoidTonguePositionBatchWorker(args)
worker.run()
