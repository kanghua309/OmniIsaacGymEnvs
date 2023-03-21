from ahrs.filters import Madgwick
import numpy as np
madgwick = Madgwick()
num_samples = 3
gyro_data = np.array([[1,2,3],[2,2,2],[3,3,3]])
acc_data = np.array([[1,2,3],[2,2,2],[3,3,3]])
Q = np.tile([1., 0., 0., 0.], (num_samples, 1)) # Allocate for quaternions
PQ = [1., 0., 0., 0.]
for t in range(1, num_samples):
    NQ = madgwick.updateIMU(PQ, gyr=gyro_data[t], acc=acc_data[t])
    print(NQ)
    PQ = NQ

# for t in range(1, num_samples):
#     Q[t] = madgwick.updateIMU(Q[t-1], gyr=gyro_data[t], acc=acc_data[t])
#     print(Q[t])
