import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

### LTE Psuedocode

## Given: Trajectory X, Constraints C
## Output: Deformed Trajectory X' which meets constraints

# 1. Generate L (Laplacian)
# 2. Transform X to Laplacian Coordinates (D = LX)
# 3. Apply Constraints P = [D; C]
# 4. Update Weights L' = [L; W]
# 5. Transform to Cartesian Coordinates X' = L' \ P

#note: C is broken down into constraints and indices corresponding to those constraints in code

fixed_weight = 1e9

def generate_laplacian(n_pts):
    L = 2.*np.diag(np.ones((n_pts,))) - np.diag(np.ones((n_pts-1,)),1) - np.diag(np.ones((n_pts-1,)),-1)
    L[0,1] = -2.
    L[-1,-2] = -2.
    L = L / 2.
    return L

def laplacian_transform(L, X):
    D = np.matmul(L, X)
    return D
    
def append_constraints(D, C, inds):
    for const in C:
        D = np.vstack((D, const.reshape((1, len(const))) * fixed_weight))
    return D

def append_weights(L, C, inds):
    n_pts = len(L)
    for i in inds:
        to_append = np.ones((1, n_pts))
        to_append[0, i] = fixed_weight
        L = np.vstack((L, to_append))
    return L
    
def cartesian_transform(new_L, P):
    new_traj, _, _, _ = np.linalg.lstsq(new_L, P, rcond=-1)
    return new_traj


def LTE(X, C, inds):
    n_pts, n_dims = np.shape(X)
    L = generate_laplacian(n_pts)
    D = laplacian_transform(L, X)
    P = append_constraints(D, C, inds)
    new_L = append_weights(L, C, inds)
    new_X = cartesian_transform(new_L, P)
    return new_X

def main2D():
    # demonstration
    num_points = 50
    t = np.linspace(0, 10, num_points).reshape((num_points, 1))
    x_demo = np.sin(t) + 0.01 * t**2 - 0.05 * (t-5)**2
    y_demo = np.cos(t) - 0.01 * t - 0.03 * t**2
    
    traj = np.hstack((x_demo, y_demo))
    
    new_traj = LTE(traj, [np.array([x_demo[0], y_demo[0]]), np.array([x_demo[-1], y_demo[-1]])], [0, num_points-1])
    
    new_traj2 = LTE(traj, [np.array([x_demo[0]+0.5, y_demo[0]-0.2]), np.array([x_demo[-1], y_demo[-1]])], [0, num_points-1])
    
    new_traj3 = LTE(traj, [np.array([x_demo[0]+0.5, y_demo[0]-0.2]), np.array([x_demo[20]-0.1, y_demo[20]-0.3]), np.array([x_demo[-1], y_demo[-1]+0.2])], [0, 20, num_points-1])
    
    plt.rcParams['figure.figsize'] = (6.5, 6.5)
    fig, axs = plt.subplots(2, 2)
    axs[0][0].set_title('Demonstration')
    axs[1][0].set_title('Same Constraints')
    axs[0][1].set_title('New Initial Point')
    axs[1][1].set_title('New Initial, Final and Viapoint')
    axs[0][0].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    axs[1][0].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    axs[0][1].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    axs[1][1].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    
    axs[1][0].plot(new_traj[:, 0], new_traj[:, 1], 'm', lw=3)
    axs[1][0].plot(x_demo[0], y_demo[0], 'k.', ms=10)
    axs[1][0].plot(x_demo[-1], y_demo[-1], 'k.', ms=10)
    axs[1][0].plot(x_demo[0], y_demo[0], 'rx', ms=10, mew=2)
    axs[1][0].plot(x_demo[-1], y_demo[-1], 'rx', ms=10, mew=2)
    
    axs[0][1].plot(new_traj2[:, 0], new_traj2[:, 1], 'g', lw=3)
    axs[0][1].plot(x_demo[0], y_demo[0], 'k.', ms=10)
    axs[0][1].plot(x_demo[-1], y_demo[-1], 'k.', ms=10)
    axs[0][1].plot(x_demo[0]+0.5, y_demo[0]-0.2, 'rx', ms=10, mew=2)
    axs[0][1].plot(x_demo[-1], y_demo[-1], 'rx', ms=10, mew=2)
    
    axs[1][1].plot(new_traj3[:, 0], new_traj3[:, 1], 'b', lw=3)
    axs[1][1].plot(x_demo[0], y_demo[0], 'k.', ms=10)
    axs[1][1].plot(x_demo[20], y_demo[20], 'k.', ms=10)
    axs[1][1].plot(x_demo[-1], y_demo[-1], 'k.', ms=10)
    axs[1][1].plot(x_demo[0]+0.5, y_demo[0]-0.2, 'rx', ms=10, mew=2)
    axs[1][1].plot(x_demo[20]-0.1, y_demo[20]-0.3, 'rx', ms=10, mew=2)
    axs[1][1].plot(x_demo[-1], y_demo[-1]+0.2, 'rx', ms=10, mew=2)
    
    plt.show()
    
def main3D():
    # demonstration
    num_points = 50
    t = np.linspace(0, 10, num_points).reshape((num_points, 1))
    x_demo = np.sin(t) + 0.01 * t**2 - 0.05 * (t-5)**2
    y_demo = np.cos(t) - 0.01 * t - 0.03 * t**2
    z_demo = 2 * np.cos(t) * np.sin(t) - 0.01 * t**1.5 + 0.03 * t**2
    
    traj = np.hstack((x_demo, y_demo, z_demo))
    
    new_traj = LTE(traj, [np.array([x_demo[0] + 0.5, y_demo[0] - 0.5, z_demo[0] + 0.5]), np.array([x_demo[-1], y_demo[-1], z_demo[-1]])], [0, num_points-1])
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot(x_demo.flatten(), y_demo.flatten(), z_demo.flatten(), 'k', lw=3)
    ax.plot(new_traj[:, 0].flatten(), new_traj[:, 1].flatten(), new_traj[:, 2].flatten(), 'm', lw=3)
    ax.plot(x_demo[0], y_demo[0], z_demo[0], 'k.', ms=10)
    ax.plot(x_demo[-1], y_demo[-1], z_demo[-1], 'k.', ms=10)
    ax.plot(new_traj[0, 0], new_traj[0, 1], new_traj[0, 2], 'rx', ms=10, mew=2)
    ax.plot(new_traj[-1, 0], new_traj[-1, 1], new_traj[-1, 2], 'rx', ms=10, mew=2)
    
    plt.show()

if __name__ == '__main__':
    main2D()
    main3D()

