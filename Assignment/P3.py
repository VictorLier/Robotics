import matplotlib.pyplot as plt
import numpy as np

# Define the circle:
points = np.linspace(0, 2*np.pi, 36)
R = 32 # Radius
p0_c = np.array([150, 0, 120]) # Center of the circle

p0 = p0_c[:, np.newaxis] + R * np.array([np.zeros(len(points)), np.cos(points), np.sin(points)])


# plot the circle
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(p0[0], p0[1], p0[2], c='b')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

