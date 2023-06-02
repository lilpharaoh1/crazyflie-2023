arr =[True, True, True]

if arr:
    print("yes")
else:
    print("no")

# print("---------------------------")

import numpy as np 
# arr = np.array()

# print(np.zeros((3, 1)) )

print("---------------------")

waypoints = np.array([0.0,  0.0,  1.0]).reshape(3,1)
pose = np.array([0.0, 0.01, 1.01]).reshape(3,1)
#                       [1.0,  1.0,  1.0],
#                       [1.5,  1.3,  1.0]])

# waypoints = np.delete(waypoints, 0, axis=0)

# print(waypoints)