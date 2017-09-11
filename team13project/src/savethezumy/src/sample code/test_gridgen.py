#!/usr/bin/env python
import ar_tag_grid as gridgen
import numpy as np

cornerNum=10;
ar_tagnums = np.array([6,5,7,8,9,10,11,12,13,14,19,20,21,22,23,25])
grid=gridgen.grid_gen(cornerNum,ar_tagnums)
print(grid)