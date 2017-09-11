#!/usr/bin/env python
import numpy as np

ARR=np.array([1,2,3,4,5,6,7,8,9])
print(ARR)
ARR = np.reshape(ARR,(3,3))
# print(ARR)
ARR = np.flipud(ARR)
print(ARR)
ARR = ARR.T
print(ARR)
ARR = np.flipud(ARR)
print(ARR)
ARR=np.reshape(ARR,(ARR.size))
print ARR