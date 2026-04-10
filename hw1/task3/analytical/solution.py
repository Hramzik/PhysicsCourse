import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def main():
    t = sp.symbols("t", real=True)

    OUT_DIR = Path(*Path(__file__).resolve().parent.parts[-3:]) / "img"
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    sin70 = float(np.sin(np.deg2rad(70.0)))
    cos70 = float(np.cos(np.deg2rad(70.0)))

    A = sp.Matrix([sp.Float(0.0), sp.Float(0.0)])
    B = sp.Matrix([sp.Float(2.0 + sin70), sp.Float(cos70)])

    O1 = sp.Matrix([sp.Float(1.0), sp.Float(0.0)])
    O2 = sp.Matrix([sp.Float(2.0), sp.Float(0.0)])

    m = sp.Integer(1)
    k = sp.Integer(1)
    L = sp.Integer(1)

    x10, y10 = 1.0, 0.0
    vx10, vy10 = 0.0, 1.0
    x20, y20 = 2.0, 0.0
    vx20, vy20 = 0.0, 1.0

    x1 = sp.Function("x1")
    y1 = sp.Function("y1")
    x2 = sp.Function("x2")
    y2 = sp.Function("y2")

    x1t = x1(t)
    y1t = y1(t)
    x2t = x2(t)
    y2t = y2(t)

    l1 = sp.sqrt(x1t**2 + y1t**2)
    l2 = sp.sqrt((x1t - x2t) ** 2 + (y1t - y2t) ** 2)
    l3 = sp.sqrt((x2t - B[0]) ** 2 + (y2t - B[1]) ** 2)

    F1x = k * (L - l1) * x1t / l1
    F1y = k * (L - l1) * y1t / l1

    F2x = k * (L - l2) * (x1t - x2t) / l2
    F2y = k * (L - l2) * (y1t - y2t) / l2

    F3x = k * (L - l2) * (x2t - x1t) / l2
    F3y = k * (L - l2) * (y2t - y1t) / l2

    F4x = k * (L - l3) * (x2t - B[0]) / l3
    F4y = k * (L - l3) * (y2t - B[1]) / l3

    eq_x1 = sp.Eq(m * sp.diff(x1t, t, t), F1x + F2x)
    eq_y1 = sp.Eq(m * sp.diff(y1t, t, t), F1y + F2y)
    eq_x2 = sp.Eq(m * sp.diff(x2t, t, t), F3x + F4x)
    eq_y2 = sp.Eq(m * sp.diff(y2t, t, t), F3y + F4y)

    try:
        solution = sp.dsolve([eq_x1, eq_y1, eq_x2, eq_y2], [x1(t), y1(t), x2(t), y2(t)])
        print("✓ Аналитическое решение найдено:")
        print(solution)
    except Exception as e:
        print("✗ Аналитическое решение не найдено")
        print(f"  Причина: {type(e).__name__}")
    print()

    # Линеаризация

    dx1, dy1, dx2, dy2 = sp.symbols("dx1 dy1 dx2 dy2", real=True)

    Fx1_eq = (F1x + F2x).subs({x1t: O1[0] + dx1, y1t: O1[1] + dy1, x2t: O2[0] + dx2, y2t: O2[1] + dy2})
    Fy1_eq = (F1y + F2y).subs({x1t: O1[0] + dx1, y1t: O1[1] + dy1, x2t: O2[0] + dx2, y2t: O2[1] + dy2})
    Fx2_eq = (F3x + F4x).subs({x1t: O1[0] + dx1, y1t: O1[1] + dy1, x2t: O2[0] + dx2, y2t: O2[1] + dy2})
    Fy2_eq = (F3y + F4y).subs({x1t: O1[0] + dx1, y1t: O1[1] + dy1, x2t: O2[0] + dx2, y2t: O2[1] + dy2})

    q = sp.Matrix([dx1, dy1, dx2, dy2])
    F_eq = sp.Matrix([Fx1_eq, Fy1_eq, Fx2_eq, Fy2_eq])

    K = F_eq.jacobian(q).subs({dx1: 0, dy1: 0, dx2: 0, dy2: 0})

    print("Линеаризованная система (в окрестности O1, O2):")
    for i in range(4):
        row = [sp.N(K[i, j]) for j in range(4)]
        print(f"  q{i+1}'' = {row}")
    print()

    # z' = A z, z = (q, q')
    Z0 = sp.Matrix(
        [
            sp.Float(x10) - O1[0],
            sp.Float(y10) - O1[1],
            sp.Float(x20) - O2[0],
            sp.Float(y20) - O2[1],
            sp.Float(vx10),
            sp.Float(vy10),
            sp.Float(vx20),
            sp.Float(vy20),
        ]
    )

    O4 = sp.zeros(4, 4)
    I4 = sp.eye(4)

    A_mat = sp.Matrix(
        [
            [O4, I4],
            [K / m, O4],
        ]
    )

    try:
        # z(t) = V * exp(Λ t) * V^{-1} * z(0)
        A_num = np.array(A_mat.evalf().tolist(), dtype=np.complex128)
        z0_num = np.array(Z0.evalf().tolist(), dtype=np.complex128).reshape(8)

        eigvals, V = np.linalg.eig(A_num)
        c = np.linalg.solve(V, z0_num)

        tt = np.linspace(0.0, 20.0, 1000)
        exp_lt = np.exp(np.outer(tt, eigvals))
        Y = exp_lt * c
        Z_all = Y @ V.T
        Z_all = np.real(np.real_if_close(Z_all, tol=10_000))

        x1_arr = float(O1[0]) + Z_all[:, 0]
        y1_arr = float(O1[1]) + Z_all[:, 1]
        x2_arr = float(O2[0]) + Z_all[:, 2]
        y2_arr = float(O2[1]) + Z_all[:, 3]

        vx1_arr = Z_all[:, 4]
        vy1_arr = Z_all[:, 5]
        vx2_arr = Z_all[:, 6]
        vy2_arr = Z_all[:, 7]

        print("✓ Решение линеаризованной системы найдено")

        Bx = float(B[0])
        By = float(B[1])

        l1_arr = np.sqrt(x1_arr**2 + y1_arr**2)
        l2_arr = np.sqrt((x1_arr - x2_arr) ** 2 + (y1_arr - y2_arr) ** 2)
        l3_arr = np.sqrt((x2_arr - Bx) ** 2 + (y2_arr - By) ** 2)

        E_arr = 0.5 * (vx1_arr**2 + vy1_arr**2 + vx2_arr**2 + vy2_arr**2) + 0.5 * (
            (l1_arr - 1.0) ** 2 + (l2_arr - 1.0) ** 2 + (l3_arr - 1.0) ** 2
        )

        plt.figure()
        plt.plot(tt, np.array(E_arr, dtype=float))
        plt.xlabel("t")
        plt.ylabel("E")
        plt.grid(True)
        e_path = OUT_DIR / "energy.png"
        plt.savefig(e_path, dpi=200, bbox_inches="tight")
        plt.close()
        print("График энергии сохранён в " + str(e_path))

        plt.figure()
        plt.plot(np.array(x1_arr, dtype=float), np.array(y1_arr, dtype=float))
        plt.plot(np.array(x2_arr, dtype=float), np.array(y2_arr, dtype=float))
        plt.xlabel("x")
        plt.ylabel("y")
        plt.grid(True)
        plt.axis("equal")
        tr_path = OUT_DIR / "trajectory.png"
        plt.savefig(tr_path, dpi=200, bbox_inches="tight")
        plt.close()
        print("График траекторий сохранён в " + str(tr_path))

    except Exception as e:
        print(f"✗ Решение линеаризованной системы не найдено: {type(e).__name__}")
        print(f"  {str(e)[:300]}")


if __name__ == "__main__":
    main()
