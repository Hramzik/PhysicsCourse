import sympy as sp

t = sp.symbols('t', real=True)
m = sp.symbols('m', positive=True, real=True)
mu = sp.symbols('mu', real=True)
k1, k2, k3 = sp.symbols('k1 k2 k3', positive=True, real=True)
L1, L2, L3 = sp.symbols('L1 L2 L3', positive=True, real=True)
x0, y0, vx0, vy0 = sp.symbols('x0 y0 vx0 vy0', real=True)

x = sp.Function('x')
y = sp.Function('y')
xt = x(t)
yt = y(t)

Cx = L2 + L3*sp.sin(mu)
Cy = L1 + L3*sp.cos(mu)

l1 = sp.sqrt((xt - L2)**2 + yt**2)
l2 = sp.sqrt(xt**2 + (yt - L1)**2)
l3 = sp.sqrt((xt - Cx)**2 + (yt - Cy)**2)

F1x = k1 * (L1 - l1) * (xt - L2) / l1
F1y = k1 * (L1 - l1) * yt / l1
F2x = k2 * (L2 - l2) * xt / l2
F2y = k2 * (L2 - l2) * (yt - L1) / l2
F3x = k3 * (L3 - l3) * (xt - Cx) / l3
F3y = k3 * (L3 - l3) * (yt - Cy) / l3

eq_x = sp.Eq(m * sp.diff(xt, t, t), F1x + F2x + F3x)
eq_y = sp.Eq(m * sp.diff(yt, t, t), F1y + F2y + F3y)

try:
    solution = sp.dsolve([eq_x, eq_y], [x(t), y(t)])
    print("✓ Аналитическое решение найдено:")
    print(solution)
except Exception as e:
    print(f"✗ Аналитическое решение не найдено")
    print(f"  Причина: {type(e).__name__}")
print()

# === Линеаризация системы ===

dx = sp.symbols('dx', real=True)
dy = sp.symbols('dy', real=True)

F1x_eq = F1x.subs([(xt, L2 + dx), (yt, L1 + dy)])
F1y_eq = F1y.subs([(xt, L2 + dx), (yt, L1 + dy)])
F2x_eq = F2x.subs([(xt, L2 + dx), (yt, L1 + dy)])
F2y_eq = F2y.subs([(xt, L2 + dx), (yt, L1 + dy)])
F3x_eq = F3x.subs([(xt, L2 + dx), (yt, L1 + dy)])
F3y_eq = F3y.subs([(xt, L2 + dx), (yt, L1 + dy)])

Fx_total_eq = F1x_eq + F2x_eq + F3x_eq
Fy_total_eq = F1y_eq + F2y_eq + F3y_eq

F_series_x = sp.series(Fx_total_eq, dx, 0, 2)
F_series_y = sp.series(Fy_total_eq, dy, 0, 2)

F_linearized_x = F_series_x.removeO()
F_linearized_y = F_series_y.removeO()

dFx_ddx = sp.diff(Fx_total_eq, dx).subs([(dx, 0), (dy, 0)])
dFx_ddy = sp.diff(Fx_total_eq, dy).subs([(dx, 0), (dy, 0)])
dFy_ddx = sp.diff(Fy_total_eq, dx).subs([(dx, 0), (dy, 0)])
dFy_ddy = sp.diff(Fy_total_eq, dy).subs([(dx, 0), (dy, 0)])

# Критически необходимая оптимизация,
# Так как без нее решение ищется дольше 1 катки в Clash Royale
dFx_ddx = sp.trigsimp(dFx_ddx)
dFx_ddy = sp.trigsimp(dFx_ddy)
dFy_ddx = sp.trigsimp(dFy_ddx)
dFy_ddy = sp.trigsimp(dFy_ddy)

dxt = sp.Function('delta_x')
dyt = sp.Function('delta_y')

Fx_lin = dFx_ddx * dxt(t) + dFx_ddy * dyt(t)
Fy_lin = dFy_ddx * dxt(t) + dFy_ddy * dyt(t)

eq_x_lin = sp.Eq(m * sp.diff(dxt(t), t, t), Fx_lin)
eq_y_lin = sp.Eq(m * sp.diff(dyt(t), t, t), Fy_lin)

print("Линеаризованная система:")
print(f"  m·δx'' = ({dFx_ddx})·δx + ({dFx_ddy})·δy")
print(f"  m·δy'' = ({dFy_ddx})·δx + ({dFy_ddy})·δy")
print()

try:
    solution_lin = sp.dsolve([eq_x_lin, eq_y_lin], [dxt(t), dyt(t)])
    print("✓ Решение линеаризованной системы в окрестности равновесия найдено:")

    sol_dx = solution_lin[0].rhs
    sol_dy = solution_lin[1].rhs
    print(f"  x(t) = L2 + ({sol_dx})")
    print(f"  y(t) = L1 + ({sol_dy})")

except Exception as e:
    print(f"✗ Решение линеаризованной системы не найдено: {type(e).__name__}")
    print(f"  {str(e)[:300]}")
