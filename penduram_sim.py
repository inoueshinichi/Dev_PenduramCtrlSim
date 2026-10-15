import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy import signal # 伝達関数の導出に使用

# ===============================
# 1. 物理パラメータ
# ===============================
M = 0.5  # 車体質量 [kg]
m = 1.0  # 車輪質量 [kg]
J = 0.0005  # 車輪慣性モーメント [kgm^2]
I = 0.02  # 車体慣性モーメント [kgm^2]
r = 0.05  # 車輪半径 [m]
l = 0.10  # 車体長さ [m]
g = 9.81  # 重力加速度 [m/s^2]
# ===============================

A = M + m + J/r**2
B = I + m*l**2
D = A*B - (m * l)**2

if D <= 0:
    raise ValueError("行列式がゼロ未満です。パラメータを確認してください。 D <= 0: D = {:.2f}".format(D))

# ===============================
# 伝達関数の導出
# ===============================
# G_theta(s) = Theta(s) / U(s)
G_theta = signal.TransferFunction([-(A + m*l/r)], [D, 0, -A*m*g*l]) # 分母=2次多項式

# G_x(s) = X(s) / U(s)
G_x = signal.TransferFunction([B/r + m*l, 0, -m*g*l/r], [D, 0, -A*m*g*l, 0, 0]) # 分母=4次多項式

print("G_theta(s) = Theta(s) / U(s)")
print(G_theta.num, G_theta.den)
print("G_x(s) = X(s) / U(s)")
print(G_x.num, G_x.den)

# 状態変数 z = [x, theta, x_dot, theta_dot]
def dynamics_nonlinear(t, z, u):
    x, theta, x_dot, theta_dot = z

    """ 運動方程式の連立方程式
    M(theta) * [x_ddot, theta_ddot]^T = rhs
    """
    # M(theta) = [[A, m*l*cos(theta)], [m*l*cos(theta), B]]
    M_theta = np.array([[A, m*l*np.cos(theta)], [m*l*np.cos(theta), B]])
    
    rhs = np.array([u/r + m*l * np.sin(theta) * theta_dot**2,
                    -u + m * g * l * np.sin(theta)])

    x_ddot, theta_ddot = np.linalg.solve(M_theta, rhs)

    return np.array([x_dot, theta_dot, x_ddot, theta_ddot])


# ===============================
# 2. 非線形モデルを平衡点 theta=0 において線形化
# ===============================
A_s = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, -(m**2*g*l**2)/D, 0, 0],
    [0, (A*m*g*l)/D, 0, 0],
], dtype=np.float64)

print("A_s Eigenvalues:", np.linalg.eigvals(A_s))

B_s = np.array([
    [0],
    [0],
    [(B/r + m*l)/D],
    [-(A + m*l/r)/D],
], dtype=np.float64)


def dynamics_linear(t, z, u):
    return A_s @ z + B_s[:,0] * u


# ===============================
# 3. シミュレーション条件
# ===============================
t_start = 0.0
t_end = 0.6
dt = 0.001
t_eval = np.arange(t_start, t_end + dt / 2, dt)

# 初期傾斜角 0.02 rad(約1.15約)
# z0 = np.array([
#     0.0,  # x[m]
#     0.02,  # theta[rad]
#     0.0,  # x_dot[m/s]
#     0.0,  # theta_dot[rad/s]
# ])

z0 = np.array([
    0.0, 
    0.3, # 0.3rad (約17.19度)
    0.0, 
    0.0, 
])

# まずは入力トルクをゼロにして自由応答を観察
def input_torque(t, z):
    return 0.0

def simulate(rhs):
    def wrapped(t, z):
        u = input_torque(t, z)
        return rhs(t, z, u)

    return solve_ivp(
        wrapped,
        (t_start, t_end),
        z0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
    )

sol_nonlinear = simulate(dynamics_nonlinear)
sol_linear = simulate(dynamics_linear)

if not sol_nonlinear.success or not sol_linear.success:
    raise RuntimeError("シミュレーションが失敗しました。")


# ===============================
# 4. グラフ描画 (線形モデルと非線形モデルの比較)
# ===============================
fig, axes = plt.subplots(2, 1, figsize=(9,7), sharex=True)
axes[0].plot(sol_nonlinear.t, sol_nonlinear.y[0], label='Nonlinear')
axes[0].plot(sol_linear.t, sol_linear.y[0], label='Linear')
axes[0].set_ylabel('Position [m]')
axes[0].legend()
axes[1].plot(sol_nonlinear.t, sol_nonlinear.y[1], label='Nonlinear')
axes[1].plot(sol_linear.t, sol_linear.y[1], label='Linear')
axes[1].set_ylabel('Angle [rad]')
axes[1].legend()

plt.tight_layout()
plt.show()

