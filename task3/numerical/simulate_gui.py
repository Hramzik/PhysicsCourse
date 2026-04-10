import tkinter as tk
from tkinter import ttk
import numpy as np

import solution as sim


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Simulation (task3)")

        self.speed = tk.DoubleVar(value=1.0)
        self.method = tk.StringVar(value="explicit_euler")

        self.i = 0
        self.x1 = self.y1 = None
        self.x2 = self.y2 = None

        top = ttk.Frame(root)
        top.pack(fill="x", padx=10, pady=10)

        ttk.Button(top, text="Явный Эйлер", command=lambda: self.start("explicit_euler")).pack(side="left")
        ttk.Button(top, text="Неявный Эйлер", command=lambda: self.start("implicit_euler")).pack(side="left", padx=5)
        ttk.Button(top, text="Полуявный Эйлер", command=lambda: self.start("semi_implicit_euler")).pack(side="left")
        ttk.Button(top, text="Верле", command=lambda: self.start("verlet")).pack(side="left", padx=5)

        ttk.Label(top, text="Скорость").pack(side="left", padx=(10, 5))
        ttk.Scale(top, from_=0.2, to=5.0, variable=self.speed, orient="horizontal").pack(
            side="left", fill="x", expand=True
        )

        self.label = ttk.Label(root, text="")
        self.label.pack(fill="x", padx=10)

        self.canvas = tk.Canvas(root, width=700, height=500, bg="white")
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

        pts0 = np.vstack(
            [
                sim.A.reshape(1, 2),
                sim.B.reshape(1, 2),
                sim.O1.reshape(1, 2),
                sim.O2.reshape(1, 2),
            ]
        )
        self.xmin, self.ymin = pts0.min(axis=0) - 0.8
        self.xmax, self.ymax = pts0.max(axis=0) + 0.8

        self.wall_ids = {
            "1": self.canvas.create_line(0, 0, 0, 0, fill="#444", width=3),
            "2": self.canvas.create_line(0, 0, 0, 0, fill="#444", width=3),
        }
        for wid in self.wall_ids.values():
            self.canvas.tag_lower(wid)

        self.anchor_ids = {}
        self.anchor_text_ids = {}
        for name, p in {"A": sim.A, "B": sim.B}.items():
            self.anchor_ids[name] = self.canvas.create_oval(0, 0, 0, 0, fill="black")
            self.anchor_text_ids[name] = self.canvas.create_text(0, 0, text=name, anchor="sw")

        self.spring_ids = {
            "1": self.canvas.create_line(0, 0, 0, 0, fill="#888"),
            "2": self.canvas.create_line(0, 0, 0, 0, fill="#888"),
            "3": self.canvas.create_line(0, 0, 0, 0, fill="#888"),
        }

        self.mass1_id = self.canvas.create_oval(0, 0, 0, 0, fill="red")
        self.mass2_id = self.canvas.create_oval(0, 0, 0, 0, fill="blue")

        self._job = None
        self._ready = False
        self.canvas.bind("<Configure>", self._on_configure)

    def start(self, method: str):
        if self._job is not None:
            try:
                self.root.after_cancel(self._job)
            except Exception:
                pass
            self._job = None

        if not self._ready:
            self.method.set(method)
            return

        self.method.set(method)
        self.i = 0

        solver = getattr(sim, method)
        x1, y1, vx1, vy1, x2, y2, vx2, vy2 = solver()
        self.x1, self.y1 = x1, y1
        self.x2, self.y2 = x2, y2

        pts = np.vstack(
            [
                np.column_stack([x1, y1]),
                np.column_stack([x2, y2]),
                sim.A.reshape(1, 2),
                sim.B.reshape(1, 2),
                sim.O1.reshape(1, 2),
                sim.O2.reshape(1, 2),
            ]
        )
        self.xmin, self.ymin = pts.min(axis=0) - 0.2
        self.xmax, self.ymax = pts.max(axis=0) + 0.2

        self._redraw_static()
        self._tick()

    def _on_configure(self, event=None):
        if self.canvas.winfo_width() <= 50 or self.canvas.winfo_height() <= 50:
            return

        if not self._ready:
            self._ready = True
            self._redraw_static()
            self.start(self.method.get())
            return

        self._redraw_static()

    def _to_canvas(self, x, y):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        sx = (w - 40) / max(self.xmax - self.xmin, 1e-9)
        sy = (h - 40) / max(self.ymax - self.ymin, 1e-9)
        s = min(sx, sy)
        cx = 20 + (x - self.xmin) * s
        cy = h - 20 - (y - self.ymin) * s
        return cx, cy

    def _redraw_static(self):
        self._redraw_walls()

        r = 5
        for name, p in {"A": sim.A, "B": sim.B}.items():
            cx, cy = self._to_canvas(float(p[0]), float(p[1]))
            self.canvas.coords(self.anchor_ids[name], cx - r, cy - r, cx + r, cy + r)
            self.canvas.coords(self.anchor_text_ids[name], cx + r + 2, cy - r - 2)

    def _redraw_walls(self):
        extent = float(max(self.xmax - self.xmin, self.ymax - self.ymin, 1.0))

        # Стена 1: x = 0
        w1_p1 = (0.0, self.ymin - extent)
        w1_p2 = (0.0, self.ymax + extent)

        # Стена 2: проходит через B и перпендикулярна пружине 3 в равновесии
        # Направление пружины 3 в равновесии: O2 -> B
        v = sim.B.astype(float) - sim.O2.astype(float)
        nrm = float(np.hypot(v[0], v[1]))
        if nrm < 1e-12:
            d = np.array([1.0, 0.0])
        else:
            v = v / nrm
            d = np.array([-v[1], v[0]])

        w2_p1 = (float(sim.B[0] - extent * d[0]), float(sim.B[1] - extent * d[1]))
        w2_p2 = (float(sim.B[0] + extent * d[0]), float(sim.B[1] + extent * d[1]))

        x1, y1 = self._to_canvas(*w1_p1)
        x2, y2 = self._to_canvas(*w1_p2)
        self.canvas.coords(self.wall_ids["1"], x1, y1, x2, y2)

        x1, y1 = self._to_canvas(*w2_p1)
        x2, y2 = self._to_canvas(*w2_p2)
        self.canvas.coords(self.wall_ids["2"], x1, y1, x2, y2)

    def _tick(self):
        if self.x1 is None:
            return

        if self.i >= len(self.x1):
            self.i = 0

        p1x, p1y = float(self.x1[self.i]), float(self.y1[self.i])
        p2x, p2y = float(self.x2[self.i]), float(self.y2[self.i])

        c1x, c1y = self._to_canvas(p1x, p1y)
        c2x, c2y = self._to_canvas(p2x, p2y)

        r = 6
        self.canvas.coords(self.mass1_id, c1x - r, c1y - r, c1x + r, c1y + r)
        self.canvas.coords(self.mass2_id, c2x - r, c2y - r, c2x + r, c2y + r)

        ax, ay = self._to_canvas(float(sim.A[0]), float(sim.A[1]))
        bx, by = self._to_canvas(float(sim.B[0]), float(sim.B[1]))

        # Пружины
        self.canvas.coords(self.spring_ids["1"], ax, ay, c1x, c1y)
        self.canvas.coords(self.spring_ids["2"], c1x, c1y, c2x, c2y)
        self.canvas.coords(self.spring_ids["3"], c2x, c2y, bx, by)

        self.label.config(text=f"{self.method.get()}   t={sim.tt[self.i]:.2f}")

        step = max(1, int(self.speed.get()))
        self.i += step
        delay = max(1, int(20 / max(self.speed.get(), 0.2)))
        self._job = self.root.after(delay, self._tick)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
