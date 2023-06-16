'''
@author: Connor Mayer
'''
from artisynth.models.jawTongue import BadinJawHyoidTongueContactBatchWorker
from jarray import array
import sys

print(sys.argv)
args = array(sys.argv, String)
print(args)
worker = BadinJawHyoidTongueContactBatchWorker(args)
worker.run()
