import numpy as np

gridArrayIDs=np.load('gridArrayIDs.npy')
tags2base=np.load('tags2base.npy')
tags2cam=np.load('tags2cam.npy')

marker2base = [-0.215, -0.191, 0.012]
ind = 12
print(gridArrayIDs[ind])
print(tags2base[ind,:])

# print(tags2base)

# print(tags2cam)