import numpy as np

# create a (5, 5) array containing unique integers drawn from [0, 100]
uarray = np.random.choice(np.arange(0, 101), replace=False, size=(5, 1, 3))

# check that each item occurs only once
print(uarray)
print((np.bincount(uarray.ravel()) == 1).all())
# True
