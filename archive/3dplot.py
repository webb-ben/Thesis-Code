t1 = 1
t2 = 2
t3 = 3
t4 = 4
x = 5
y = 6
z = 7
st1 = 8
st2 = 9
st3 = 10
st4 = 11
lt1 = 12
lt2 = 13
lt3 = 14
lt4 = 15
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
from scipy import stats
from mpl_toolkits.mplot3d import Axes3D

grouped_x = []
grouped_y = []
grouped_alt = []
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(18,24):
    filename = 'DataFolder/Data/Line1/Line%03i.csv' % i
    A = np.genfromtxt(filename, delimiter=",", skip_header=2)

    scat = ax.scatter(A[:,x]**2, A[:,t2], A[:,y],label= 'Y = %.2f' %np.mean(A[:, y]))
#     plt.scatter(A[:,x], A[:,t2], s = 1, label= 'Y = %.2f' %np.mean(A[:, y]))
    grouped_x.extend(A[:,x])
    grouped_y.extend(A[:,t2])
    grouped_alt.extend(A[:,t1])

grouped_x = np.mat(grouped_x).T
grouped_y = np.asarray(grouped_y)
grouped_alt = np.mat(grouped_alt).T

modified_x =  np.cos(np.pi * grouped_x * 0.00661375661)

A2 = np.hstack([grouped_alt, modified_x, np.ones([modified_x.shape[0], 1])])
AAinv = np.linalg.inv( np.dot(A2.T, A2) )
result = scipy.linalg.lstsq(A2, grouped_y)
b = result[0]
N = grouped_y.shape[0]
C = b.shape[0]
df_e = N-C
df_r = C-1
error = grouped_y - np.dot(A2, b)
sse = np.dot(error.T, error) / df_e
stderr = np.sqrt( np.diagonal( sse[0, 0] * AAinv ) )
t = b.T / stderr
p = 2*(1 - stats.t.cdf(abs(t), df_e))
r2 = 1 - error.var() / grouped_y.var()

line_x = np.linspace(grouped_x.min(), grouped_x.max(), 100)
line_alt = np.linspace(grouped_alt.min(), grouped_alt.max(), 100)
line_y = line_alt * b[0] + np.cos(np.pi * line_x * 0.00661375661) * b[1] + b[2]

# plt.plot(line_x, line_y, '-', label='Linear Regression')

plt.legend(markerscale=5)
plt.xlabel("X")
plt.ylabel("θ2")

print ("r2: ",  r2)
# print ("Linear Equation: θ2 = %0.3f * y + %0.3f * cos(0.00661375661π * x) + %0.3f" % (b[0], b[1], b[2]))
print (b)
plt.autoscale()
plt.show()