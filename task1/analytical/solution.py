import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def main():
    t = sp.symbols("t", real=True)

    OUT_DIR = Path(*Path(__file__).resolve().parent.parts[-2:]) / "img"
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # Константы, иначе умирает на тригонометрии
    sin35 = float(np.sin(np.deg2rad(35.0)))
    cos35 = float(np.cos(np.deg2rad(35.0)))

    cot35 = cos35 / sin35
    sin70 = 2.0 * sin35 * cos35
    cos70 = cos35 * cos35 - sin35 * sin35
    tan70 = sin70 / cos70

    A = sp.Matrix([sp.Float(cot35), sp.Float(0.0)])
    B = sp.Matrix([sp.Float(cot35 - sin70), sp.Float(1.0 + cos70)])
    O = sp.Matrix([sp.Float(cot35), sp.Float(1.0)])

    m = sp.Integer(1)
    k = sp.Integer(1)
    L = sp.Integer(1)

    x0, y0 = 1.0, 1.0
    vx0, vy0 = 1.0, 0.0

    x = sp.Function("x")
    y = sp.Function("y")
    xt = x(t)
    yt = y(t)

    l1 = sp.sqrt((xt - A[0]) ** 2 + (yt - A[1]) ** 2)
    l2 = sp.sqrt((xt - B[0]) ** 2 + (yt - B[1]) ** 2)

    F1x = k * (L - l1) * (xt - A[0]) / l1
    F1y = k * (L - l1) * (yt - A[1]) / l1

    F2x = k * (L - l2) * (xt - B[0]) / l2
    F2y = k * (L - l2) * (yt - B[1]) / l2

    eq_x = sp.Eq(m * sp.diff(xt, t, t), F1x + F2x)
    eq_y = sp.Eq(m * sp.diff(yt, t, t), F1y + F2y)

    try:
        solution = sp.dsolve([eq_x, eq_y], [x(t), y(t)])
        print("✓ Аналитическое решение найдено:")
        print(solution)
    except Exception as e:
        print("✗ Аналитическое решение не найдено")
        print(f"  Причина: {type(e).__name__}")
    print()

    dx, dy = sp.symbols("dx dy", real=True)

    Fx_eq = (F1x + F2x).subs({xt: O[0] + dx, yt: O[1] + dy})
    Fy_eq = (F1y + F2y).subs({xt: O[0] + dx, yt: O[1] + dy})

    dFx_ddx = sp.diff(Fx_eq, dx).subs({dx: 0, dy: 0})
    dFx_ddy = sp.diff(Fx_eq, dy).subs({dx: 0, dy: 0})
    dFy_ddx = sp.diff(Fy_eq, dx).subs({dx: 0, dy: 0})
    dFy_ddy = sp.diff(Fy_eq, dy).subs({dx: 0, dy: 0})

    dxt = sp.Function("delta_x")
    dyt = sp.Function("delta_y")

    Fx_lin = dFx_ddx * dxt(t) + dFx_ddy * dyt(t)
    Fy_lin = dFy_ddx * dxt(t) + dFy_ddy * dyt(t)

    eq_x_lin = sp.Eq(m * sp.diff(dxt(t), t, t), Fx_lin)
    eq_y_lin = sp.Eq(m * sp.diff(dyt(t), t, t), Fy_lin)

    ics = {
        dxt(0): sp.Float(x0) - O[0],
        sp.diff(dxt(t), t).subs(t, 0): sp.Float(vx0),
        dyt(0): sp.Float(y0) - O[1],
        sp.diff(dyt(t), t).subs(t, 0): sp.Float(vy0),
    }

    print("Линеаризованная система (в окрестности O):")
    print(f"  δx'' = ({sp.N(dFx_ddx)})·δx + ({sp.N(dFx_ddy)})·δy")
    print(f"  δy'' = ({sp.N(dFy_ddx)})·δx + ({sp.N(dFy_ddy)})·δy")
    print()

    try:
        solution_lin = sp.dsolve([eq_x_lin, eq_y_lin], [dxt(t), dyt(t)], ics=ics)
        sol_dx = solution_lin[0].rhs
        sol_dy = solution_lin[1].rhs

        x_lin = sp.simplify(O[0] + sol_dx)
        y_lin = sp.simplify(O[1] + sol_dy)

        print("✓ Решение линеаризованной системы найдено:")
        print(f"  x(t) = {x_lin}")
        print(f"  y(t) = {y_lin}")
        print()

        vx_lin = sp.diff(x_lin, t)
        vy_lin = sp.diff(y_lin, t)

        l1_lin = sp.sqrt((x_lin - A[0]) ** 2 + (y_lin - A[1]) ** 2)
        l2_lin = sp.sqrt((x_lin - B[0]) ** 2 + (y_lin - B[1]) ** 2)

        E_expr = sp.simplify(
            sp.Rational(1, 2) * (vx_lin ** 2 + vy_lin ** 2)
            + sp.Rational(1, 2) * ((l1_lin - 1) ** 2 + (l2_lin - 1) ** 2)
        )

        tt = np.linspace(0.0, 20.0, 1000)

        E_f = sp.lambdify(t, sp.N(E_expr), "numpy")
        x_f = sp.lambdify(t, sp.N(x_lin), "numpy")
        y_f = sp.lambdify(t, sp.N(y_lin), "numpy")

        plt.figure()
        plt.plot(tt, np.array(E_f(tt), dtype=float))
        plt.xlabel("t")
        plt.ylabel("E")
        plt.grid(True)
        e_path = OUT_DIR / "energy.png"
        plt.savefig(e_path, dpi=200, bbox_inches="tight")
        plt.close()
        print("График энергии сохранён в " + str(e_path))

        xx = np.array(x_f(tt), dtype=float)
        yy = np.array(y_f(tt), dtype=float)
        plt.figure()
        plt.plot(xx, yy)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.grid(True)
        plt.axis("equal")
        tr_path = OUT_DIR / "trajectory.png"
        plt.savefig(tr_path, dpi=200, bbox_inches="tight")
        plt.close()
        print("График траектории сохранён в " + str(tr_path))

    except Exception as e:
        print(f"✗ Решение линеаризованной системы не найдено: {type(e).__name__}")
        print(f"  {str(e)[:300]}")


if __name__ == "__main__":
    main()
