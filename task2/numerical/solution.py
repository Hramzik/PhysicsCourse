import numpy as np
from pathlib import Path

m = 1.0
k = 1.0
L = 1.0
mu = np.deg2rad(70.0)

x0, y0 = 1.0, 1.0
vx0, vy0 = 0.0, 1.0

t0, t1, dt = 0.0, 20.0, 0.01
n = int((t1 - t0) / dt) + 1
tt = np.linspace(t0, t1, n)

OUT_DIR = Path(*Path(__file__).resolve().parent.parts[-2:]) / "img"
OUT_DIR.mkdir(parents=True, exist_ok=True)

A = np.array([L, 0.0])
B = np.array([0.0, L])
C = np.array([L + L * np.sin(mu), L + L * np.cos(mu)])
EPS = 1e-12


def spring_force(x, y, anchor):
    dx = x - anchor[0]
    dy = y - anchor[1]
    length = np.hypot(dx, dy)
    length = max(length, EPS)
    coeff = k * (L - length) / length
    return coeff * dx, coeff * dy


def force_total(x, y):
    f1x, f1y = spring_force(x, y, A)
    f2x, f2y = spring_force(x, y, B)
    f3x, f3y = spring_force(x, y, C)
    return f1x + f2x + f3x, f1y + f2y + f3y


def accel(x, y):
    fx, fy = force_total(x, y)
    return fx / m, fy / m


def energy(x, y, vx, vy):
    l1 = max(np.hypot(x - A[0], y - A[1]), EPS)
    l2 = max(np.hypot(x - B[0], y - B[1]), EPS)
    l3 = max(np.hypot(x - C[0], y - C[1]), EPS)
    kinetic = 0.5 * m * (vx * vx + vy * vy)
    potential = 0.5 * k * ((l1 - L) ** 2 + (l2 - L) ** 2 + (l3 - L) ** 2)
    return kinetic + potential


def init_state():
    x = np.empty(n)
    y = np.empty(n)
    vx = np.empty(n)
    vy = np.empty(n)
    x[0], y[0], vx[0], vy[0] = x0, y0, vx0, vy0
    return x, y, vx, vy


def explicit_euler():
    x, y, vx, vy = init_state()
    for i in range(n - 1):
        ax, ay = accel(x[i], y[i])
        x[i + 1] = x[i] + dt * vx[i]
        y[i + 1] = y[i] + dt * vy[i]
        vx[i + 1] = vx[i] + dt * ax
        vy[i + 1] = vy[i] + dt * ay
    return x, y, vx, vy


def implicit_euler(max_iter=40, tol=1e-10):
    x, y, vx, vy = init_state()
    for i in range(n - 1):
        xg = x[i] + dt * vx[i]
        yg = y[i] + dt * vy[i]
        vxg = vx[i]
        vyg = vy[i]

        for _ in range(max_iter):
            ax, ay = accel(xg, yg)
            vxn = vx[i] + dt * ax
            vyn = vy[i] + dt * ay
            xn = x[i] + dt * vxn
            yn = y[i] + dt * vyn

            err = max(abs(xn - xg), abs(yn - yg), abs(vxn - vxg), abs(vyn - vyg))
            xg, yg, vxg, vyg = xn, yn, vxn, vyn
            if err < tol:
                break

        x[i + 1], y[i + 1], vx[i + 1], vy[i + 1] = xg, yg, vxg, vyg
    return x, y, vx, vy


def semi_implicit_euler():
    x, y, vx, vy = init_state()
    for i in range(n - 1):
        ax, ay = accel(x[i], y[i])
        vx[i + 1] = vx[i] + dt * ax
        vy[i + 1] = vy[i] + dt * ay
        x[i + 1] = x[i] + dt * vx[i + 1]
        y[i + 1] = y[i] + dt * vy[i + 1]
    return x, y, vx, vy


def verlet():
    x, y, vx, vy = init_state()
    for i in range(n - 1):
        ax, ay = accel(x[i], y[i])
        x[i + 1] = x[i] + dt * vx[i] + 0.5 * dt * dt * ax
        y[i + 1] = y[i] + dt * vy[i] + 0.5 * dt * dt * ay
        ax_new, ay_new = accel(x[i + 1], y[i + 1])
        vx[i + 1] = vx[i] + 0.5 * dt * (ax + ax_new)
        vy[i + 1] = vy[i] + 0.5 * dt * (ay + ay_new)
    return x, y, vx, vy


def save_plots(name, x, y, vx, vy):
    import matplotlib.pyplot as plt
    e = np.array([energy(xi, yi, vxi, vyi) for xi, yi, vxi, vyi in zip(x, y, vx, vy)])

    plt.figure()
    plt.plot(tt, e)
    plt.xlabel("t")
    plt.ylabel("E")
    plt.grid(True)
    energy_path = OUT_DIR / f"{name}_energy.png"
    plt.savefig(energy_path, dpi=200, bbox_inches="tight")
    plt.close()

    plt.figure()
    plt.plot(x, y)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True)
    plt.axis("equal")
    traj_path = OUT_DIR / f"{name}_trajectory.png"
    plt.savefig(traj_path, dpi=200, bbox_inches="tight")
    plt.close()

    print(f"{name}: {energy_path}")
    print(f"{name}: {traj_path}")


def main():
    methods = {
        "explicit_euler": explicit_euler,
        "implicit_euler": implicit_euler,
        "semi_implicit_euler": semi_implicit_euler,
        "verlet": verlet,
    }

    for name, solver in methods.items():
        x, y, vx, vy = solver()
        save_plots(name, x, y, vx, vy)


if __name__ == "__main__":
    main()
