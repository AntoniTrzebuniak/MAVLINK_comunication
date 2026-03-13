# Symulacja ruchu kuli z oporem powietrza (masa m = 0.16 kg)
# Dane wejściowe i parametry
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# parametry zadania
d = 0.05                 # średnica [m]
r = d/2
A = math.pi * r**2       # pole przekroju czołowego [m^2]
Cd = 0.50                # współczynnik oporu kuli
rho = 1.2              # gęstość powietrza [kg/m^3]
m = 0.16                 # masa [kg] - od użytkownika
g = 9.81                 # przyspieszenie grawitacyjne [m/s^2]

# warunki początkowe
vx0 = 23.0               # prędkość pozioma [m/s]
vy0 = 0.0                # prędkość pionowa [m/s]
x0, y0 = 0.0, 40.0       # początkowe położenie [m]

# funkcja przyspieszenia z oporem (wektorowo)
def accel(vx, vy):
    v = math.hypot(vx, vy)
    if v == 0:
        return 0.0, -g
    Fd = 0.5 * rho * Cd * A * v**2
    ax = - (Fd / m) * (vx / v)
    ay = -g - (Fd / m) * (vy / v)
    return ax, ay

# integrator (RK4) do momentu zetknięcia z ziemią (y<=0)
dt = 0.001  # krok czasowy [s]
t = 0.0
x, y = x0, y0
vx, vy = vx0, vy0

# listy do wykresu
ts = [t]
xs = [x]
ys = [y]
vxs = [vx]
vys = [vy]

while y > 0 and t < 30.0:
    # RK4 dla równań x' = vx, y' = vy, vx' = ax(vx,vy), vy' = ay(vx,vy)
    def deriv(state):
        vx_s, vy_s = state[2], state[3]
        ax_s, ay_s = accel(vx_s, vy_s)
        return np.array([vx_s, vy_s, ax_s, ay_s])
    
    state = np.array([x, y, vx, vy])
    k1 = deriv(state)
    k2 = deriv(state + 0.5*dt*k1)
    k3 = deriv(state + 0.5*dt*k2)
    k4 = deriv(state + dt*k3)
    state_next = state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
    
    x, y, vx, vy = state_next
    t += dt
    
    ts.append(t)
    xs.append(x)
    ys.append(y)
    vxs.append(vx)
    vys.append(vy)

# interpolacja ostatniego kroku żeby znaleźć dokładniejsze x przy y=0
if ys[-1] < 0:
    x1, y1 = xs[-2], ys[-2]
    x2, y2 = xs[-1], ys[-1]
    # liniowa interpolacja po y
    alpha = (0 - y1) / (y2 - y1)
    x_ground = x1 + alpha * (x2 - x1)
    t_ground = ts[-2] + alpha * (ts[-1] - ts[-2])
else:
    x_ground = xs[-1]
    t_ground = ts[-1]

# porównanie z przypadkiem bez oporu (prosty rachunek)
t_freefall = math.sqrt(2 * y0 / g)
x_nodrag = vx0 * t_freefall

# Wyniki w tabeli
summary = pd.DataFrame({
    "opis": ["masa [kg]", "średnica [m]", "pole A [m^2]", "Cd", "gęstość powietrza [kg/m^3]",
             "czas spadku z oporem [s]", "zasięg z oporem [m]", "czas spadku bez oporu [s]", "zasięg bez oporu [m]"],
    "wartość": [m, d, A, Cd, rho, round(t_ground, 4), round(x_ground, 3), round(t_freefall, 4), round(x_nodrag, 3)]
})

# Rysunki: trajektoria i prędkości
plt.figure(figsize=(8,5))
plt.plot(xs, ys)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajektoria kuli (z oporem powietrza)")
plt.grid(True)
plt.show()

plt.figure(figsize=(8,5))
plt.plot(ts, vxs, label="vx")
plt.plot(ts, vys, label="vy")
plt.xlabel("t [s]")
plt.ylabel("prędkość [m/s]")
plt.title("Składowe prędkości w czasie")
plt.legend()
plt.grid(True)
plt.show()
# Wyświetl tabelę i wydrukuj wynik końcowy

print(f"Zasięg z oporem: {x_ground:.3f} m")
print(f"Czas spadku (z oporem): {t_ground:.4f} s")
print(f"Zasięg bez oporu (porównanie): {x_nodrag:.3f} m (czas bez oporu {t_freefall:.4f} s)")

