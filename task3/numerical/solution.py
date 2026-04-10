import numpy as np
from pathlib import Path

m = 1.0
k = 1.0
L = 1.0

x10, y10 = 1.0, 0.0
vx10, vy10 = 0.0, 1.0

x20, y20 = 2.0, 0.0
vx20, vy20 = 0.0, 1.0

t0, t1, dt = 0.0, 20.0, 0.01
n = int((t1 - t0) / dt) + 1
tt = np.linspace(t0, t1, n)

OUT_DIR = Path(*Path(__file__).resolve().parent.parts[-2:]) / "img"
OUT_DIR.mkdir(parents=True, exist_ok=True)

sin70 = float(np.sin(np.deg2rad(70.0)))
cos70 = float(np.cos(np.deg2rad(70.0)))

A = np.array([0.0, 0.0], dtype=float)
B = np.array([2.0 + sin70, cos70], dtype=float)

O1 = np.array([1.0, 0.0], dtype=float)
O2 = np.array([2.0, 0.0], dtype=float)

EPS = 1e-12


def spring_force(p: np.ndarray, anchor: np.ndarray):
    dx = float(p[0] - anchor[0])
    dy = float(p[1] - anchor[1])
    length = float(np.hypot(dx, dy))
    length = max(length, EPS)
    coeff = k * (L - length) / length
    return coeff * dx, coeff * dy


def spring_force_between(p1: np.ndarray, p2: np.ndarray):
    dx = float(p1[0] - p2[0])
    dy = float(p1[1] - p2[1])
    length = float(np.hypot(dx, dy))
    length = max(length, EPS)
    coeff = k * (L - length) / length
    return coeff * dx, coeff * dy


def force_total(x1: float, y1: float, x2: float, y2: float):
    p1 = np.array([x1, y1], dtype=float)
    p2 = np.array([x2, y2], dtype=float)

    # Пружина 1: A -> точка 1
    f1x, f1y = spring_force(p1, A)

    # Пружина 2: точка 1 <-> точка 2
    f2x, f2y = spring_force_between(p1, p2)

    # Пружина 3: точка 2 -> B
    f4x, f4y = spring_force(p2, B)

    # Сумма сил
    fx1 = f1x + f2x
    fy1 = f1y + f2y

    fx2 = (-f2x) + f4x
    fy2 = (-f2y) + f4y

    return fx1, fy1, fx2, fy2


def accel(x1: float, y1: float, x2: float, y2: float):
    fx1, fy1, fx2, fy2 = force_total(x1, y1, x2, y2)
    return fx1 / m, fy1 / m, fx2 / m, fy2 / m


def energy(x1: float, y1: float, vx1: float, vy1: float, x2: float, y2: float, vx2: float, vy2: float):
    l1 = max(float(np.hypot(x1 - A[0], y1 - A[1])), EPS)
    l2 = max(float(np.hypot(x1 - x2, y1 - y2)), EPS)
    l3 = max(float(np.hypot(x2 - B[0], y2 - B[1])), EPS)

    kinetic = 0.5 * m * (vx1 * vx1 + vy1 * vy1 + vx2 * vx2 + vy2 * vy2)
    potential = 0.5 * k * ((l1 - L) ** 2 + (l2 - L) ** 2 + (l3 - L) ** 2)
    return kinetic + potential


def init_state():
    x1 = np.empty(n)
    y1 = np.empty(n)
    vx1 = np.empty(n)
    vy1 = np.empty(n)

    x2 = np.empty(n)
    y2 = np.empty(n)
    vx2 = np.empty(n)
    vy2 = np.empty(n)

    x1[0], y1[0], vx1[0], vy1[0] = x10, y10, vx10, vy10
    x2[0], y2[0], vx2[0], vy2[0] = x20, y20, vx20, vy20

    return x1, y1, vx1, vy1, x2, y2, vx2, vy2


def explicit_euler():
    x1, y1, vx1, vy1, x2, y2, vx2, vy2 = init_state()
    for i in range(n - 1):
        ax1, ay1, ax2, ay2 = accel(x1[i], y1[i], x2[i], y2[i])

        x1[i + 1] = x1[i] + dt * vx1[i]
        y1[i + 1] = y1[i] + dt * vy1[i]
        vx1[i + 1] = vx1[i] + dt * ax1
        vy1[i + 1] = vy1[i] + dt * ay1

        x2[i + 1] = x2[i] + dt * vx2[i]
        y2[i + 1] = y2[i] + dt * vy2[i]
        vx2[i + 1] = vx2[i] + dt * ax2
        vy2[i + 1] = vy2[i] + dt * ay2

    return x1, y1, vx1, vy1, x2, y2, vx2, vy2


def implicit_euler(max_iter: int = 40, tol: float = 1e-10):
    x1, y1, vx1, vy1, x2, y2, vx2, vy2 = init_state()

    for i in range(n - 1):
        x1g = x1[i] + dt * vx1[i]
        y1g = y1[i] + dt * vy1[i]
        vx1g = vx1[i]
        vy1g = vy1[i]

        x2g = x2[i] + dt * vx2[i]
        y2g = y2[i] + dt * vy2[i]
        vx2g = vx2[i]
        vy2g = vy2[i]

        for _ in range(max_iter):
            ax1, ay1, ax2, ay2 = accel(x1g, y1g, x2g, y2g)

            vx1n = vx1[i] + dt * ax1
            vy1n = vy1[i] + dt * ay1
            x1n = x1[i] + dt * vx1n
            y1n = y1[i] + dt * vy1n

            vx2n = vx2[i] + dt * ax2
            vy2n = vy2[i] + dt * ay2
            x2n = x2[i] + dt * vx2n
            y2n = y2[i] + dt * vy2n

            err = max(
                abs(x1n - x1g),
                abs(y1n - y1g),
                abs(vx1n - vx1g),
                abs(vy1n - vy1g),
                abs(x2n - x2g),
                abs(y2n - y2g),
                abs(vx2n - vx2g),
                abs(vy2n - vy2g),
            )

            x1g, y1g, vx1g, vy1g = x1n, y1n, vx1n, vy1n
            x2g, y2g, vx2g, vy2g = x2n, y2n, vx2n, vy2n

            if err < tol:
                break

        x1[i + 1], y1[i + 1], vx1[i + 1], vy1[i + 1] = x1g, y1g, vx1g, vy1g
        x2[i + 1], y2[i + 1], vx2[i + 1], vy2[i + 1] = x2g, y2g, vx2g, vy2g

    return x1, y1, vx1, vy1, x2, y2, vx2, vy2


def semi_implicit_euler():
    x1, y1, vx1, vy1, x2, y2, vx2, vy2 = init_state()
    for i in range(n - 1):
        ax1, ay1, ax2, ay2 = accel(x1[i], y1[i], x2[i], y2[i])

        vx1[i + 1] = vx1[i] + dt * ax1
        vy1[i + 1] = vy1[i] + dt * ay1
        x1[i + 1] = x1[i] + dt * vx1[i + 1]
        y1[i + 1] = y1[i] + dt * vy1[i + 1]

        vx2[i + 1] = vx2[i] + dt * ax2
        vy2[i + 1] = vy2[i] + dt * ay2
        x2[i + 1] = x2[i] + dt * vx2[i + 1]
        y2[i + 1] = y2[i] + dt * vy2[i + 1]

    return x1, y1, vx1, vy1, x2, y2, vx2, vy2


def verlet():
    x1, y1, vx1, vy1, x2, y2, vx2, vy2 = init_state()

    for i in range(n - 1):
        ax1, ay1, ax2, ay2 = accel(x1[i], y1[i], x2[i], y2[i])

        x1[i + 1] = x1[i] + dt * vx1[i] + 0.5 * dt * dt * ax1
        y1[i + 1] = y1[i] + dt * vy1[i] + 0.5 * dt * dt * ay1

        x2[i + 1] = x2[i] + dt * vx2[i] + 0.5 * dt * dt * ax2
        y2[i + 1] = y2[i] + dt * vy2[i] + 0.5 * dt * dt * ay2

        ax1_new, ay1_new, ax2_new, ay2_new = accel(x1[i + 1], y1[i + 1], x2[i + 1], y2[i + 1])

        vx1[i + 1] = vx1[i] + 0.5 * dt * (ax1 + ax1_new)
        vy1[i + 1] = vy1[i] + 0.5 * dt * (ay1 + ay1_new)

        vx2[i + 1] = vx2[i] + 0.5 * dt * (ax2 + ax2_new)
        vy2[i + 1] = vy2[i] + 0.5 * dt * (ay2 + ay2_new)

    return x1, y1, vx1, vy1, x2, y2, vx2, vy2


def save_plots(name: str, x1, y1, vx1, vy1, x2, y2, vx2, vy2):
    import matplotlib.pyplot as plt

    e = np.array(
        [
            energy(a, b, c, d, e2, f2, g2, h2)
            for a, b, c, d, e2, f2, g2, h2 in zip(x1, y1, vx1, vy1, x2, y2, vx2, vy2)
        ],
        dtype=float,
    )

    plt.figure()
    plt.plot(tt, e)
    plt.xlabel("t")
    plt.ylabel("E")
    plt.grid(True)
    energy_path = OUT_DIR / f"{name}_energy.png"
    plt.savefig(energy_path, dpi=200, bbox_inches="tight")
    plt.close()

    plt.figure()
    plt.plot(x1, y1)
    plt.plot(x2, y2)
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
        x1, y1, vx1, vy1, x2, y2, vx2, vy2 = solver()
        save_plots(name, x1, y1, vx1, vy1, x2, y2, vx2, vy2)


if __name__ == "__main__":
    main()
