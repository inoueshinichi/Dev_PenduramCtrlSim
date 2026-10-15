# Dev_PenduramCtrlSim

車輪型倒立振子の制御シミュレーション

# 物理モデル

```text
      車体の鉛直方向
            ↑
            |     ■ 車体の頂点
            |<θ> /
            |  ○ 車体の重心
            | /
--[x=Φr]--> ◎ 車輪
0 ---------------> x
```

## 物理パラメータ


| 場所  | 記号  | 単位 | 意味 |
| ---   | ---  | ---  | --- |
| 車体  | `m` | [kg] | 車体の重量 |
|      | `J` | [kgm^2] | 車体の慣性モーメント |
|      | `l` | [m] | 車輪から車体の重心までの距離 (車体の長さは`2l`) |
| 車輪  | `M` | [kg] | 車輪の重量 |
|      | `I` | [kgm^2] | 車輪の回転慣性 |
|      | `r` | [m] | 車輪の半径 |
| 共通  | `g` | [m/s^2] | 重力加速度 |


## 制御パラメータ


| 記号  | 単位 | 意味 |
| --- | ---   | --- |
| `x` | [kg]  | 車輪の水平位置 |
| `θ` | [rad] | 車体の鉛直方向からの傾き(x軸正方向を`+`) |




## 制御入力

- `u` モータの駆動トルク [Nm]

## 前提条件
- 【車輪と地面】との摩擦力は無視
- 【車軸】の粘性摩擦力は無視
- 【車輪】のモータは左右1個ずつ合計2個使うが、特性は揃っているとする

- 【拘束条件】 車輪と床は滑らない
```math
x = \phi r
```



# 運動方程式の導出


| 方法          | 特徴                                  | 可否  |
| ----------- | ----------------------------------- | --- |
| ニュートン・オイラー法 | 接触力(作用/反作用)を直接記述できる. 一般的な運動方程式の作り方. | ×   |
| ラグランジュ法     | 系のエネルギーから運動方程式を求める. 接触力を陰にできる.      | 〇   |


## ラグランジュ法による運動方程式の導出

### ラグランジュ方程式
```math
\frac{d}{dt}(\frac{\delta L}{\delta \dot{q}_i}) - \frac{\delta L}{\delta q_i} = Q_i
```

今回選択する一般化座標

```math
q = [x, θ]
```
```math
Q_x = \frac{\tau}{r}
```
```math
Q_\theta = -\tau
```


ラグラジアン $L = T - V$
- T: 系全体の運動エネルギー
- V: 系全体の位置エネルギー 
- $q_i$: 一般化座標
- $Q_i$: その座標に対する一般化力

#### 運動エネルギー T
- 並進運動エネルギー $T_{trans} = \frac{1}{2}m\nu^2$
- 回転運動エネルギー $T_{rot} = \frac{1}{2}I\omega^2$

車体の重心座標 $(x_G, y_G)$
- $x_G = x + lsin\theta$
- $y_G = lcos\theta$

重心速度 $(\dot{x_G}, \dot{y_G})$
- $\dot{x_G} = \dot{x} + lcos\theta \dot{\theta}$
- $\dot{y_G} = -lsin\theta \dot{\theta}$

車体の並進運動エネルギー $T_{body,trans}$
```math
T_{body,trans} = \frac{1}{2}m(\dot{x}^2 + 2lcos\theta \dot{x} \dot{\theta} + l^2 + \dot{\theta}^2)
```

車体の回転運動エネルギー $T_{body,rot}$
```math
T_{body,rot} = \frac{1}{2} I \dot{\theta}^2
```

車輪の並進運動エネルギー $T_{wheel,trans}$
```math
T_{wheel,trans} = \frac{1}{2}M\dot{x}^2
```

車輪の回転運動エネルギー $T_{wheel,rot}$
```math
T_{wheel,rot} = \frac{1}{2}J(\frac{\dot{x}}{r})^2
```

##### 運動エネルギー T
```math 
\begin{aligned}
T &= T_{body,trans} + T_{body,rot} + T_{wheel,trans} + T_{wheel,rot} \\
 &= \frac{1}{2}(M + m + \frac{J}{r^2})\dot{x}^2 + mlcos\theta \dot{x} \dot{\theta} + \frac{1}{2}(I + ml^2) \dot{\theta}^2
\end{aligned}
```

#### 位置エネルギー V
- 重力による位置エネルギーは車軸の位置を基準にすることで
```math
V = mgy_G
```

#### 一般化力 $Q_x, Q_\theta$ の定義
- 一般化力は, 仮想仕事を一般化座標の微小変位で表したときの係数.
- 車軸の回転角度 $\phi$
- モータの駆動トルク $\tau$
- 車輪が前進方向に回転するようにトルクを与えると、モータは車軸だけでなく、反作用として車体にもトルクを与える.


##### 車体の回転方向に関する仮想仕事 
```math
\delta W = \tau(\delta \phi - \delta \theta)
```

- 転がり条件 $\delta \phi = \frac{\delta x}{r}$ を使うと

##### 車軸の並進方向に関する仮想仕事
```math
\delta W = \frac{\tau}{r} \delta x - \tau \delta \theta
```

- 車軸の位置 $x$ 方向に関する一般化力: $Q_x = \frac{\tau}{r}$
- 車体の回転 $\theta$ 方向に関する一般化力: $Q_\theta = -\tau$


#### 一般化座標`x`に対してのラグランジュ方程式
```math
\frac{d}{dt} \frac{\delta L}{\delta \dot{x}} - \frac{\delta L}{\delta x} = Q_x
```

```math
\frac{\delta L}{\delta x} = 0
```

```math
\frac{\delta L}{\delta \dot{x}} = (M + m + \frac{J}{r^2})\dot{x} + mlcos\theta \dot{\theta}
```

```math
\frac{d}{dt} \frac{\delta L}{\delta \dot{x}} = (M + m + \frac{J}{r^2})\ddot{x} + mlcos\theta \ddot{\theta} - mlsin\theta \dot{\theta}^2
```

並進方向の運動方程式
```math
(M + m + \frac{J}{r^2})\ddot{x} + mlcos\theta \ddot{\theta} - mlsin\theta \ddot{\theta} = \frac{\tau}{r}
```

#### 一般化座標`θ`にたいしてのラグランジュ方程式
```math
\frac{d}{dt}\frac{\delta L}{\delta \dot{\theta}} - \frac{\delta L}{\delta \theta} = Q_\theta
```

```math
\frac{\delta L}{\delta \dot{\theta}} = mlcos\theta \dot{x} + (I + ml^2) \dot{\theta}
```

```math
\frac{d}{dt}\frac{\delta L}{\delta \dot{\theta}} = mlcos\theta \ddot{x} - mlsin\theta \dot{x} \dot{\theta} + (I + ml^2) \ddot{\theta}
```

```math
\frac{\delta L}{\delta \theta} = -mlsin\theta \dot{x} \dot{\theta} + mglsin\theta
```

回転方向の運動方程式
```math
mlcos\theta\ddot{x} + (I + ml^2)\ddot{\theta} - mglsin\theta = -\tau
```


# 運動方程式
略記
```math
A = M + m + \frac{J}{r^2}
```
```math
B = I + ml^2
```

車輪の位置に関する運動方程式
```math
A\ddot{x} + mlcos\theta\ddot{\theta} - mlsin\theta\ddot{\theta}^2 = \frac{u}{r}
```

車体の重心周りの回転に関する運動方程式
```math
mlcos\theta\ddot{x} + B\ddot{\theta} - mglsin\theta = -u
```

行列表現
```math
\begin{bmatrix}
A & mlcos\theta \\
mlcos\theta & B
\end{bmatrix}

\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
= 

\begin{bmatrix}
\frac{u}{r} + mlsin\theta\dot{\theta}^2 \\
-u + mglsin\theta
\end{bmatrix}
```

## 非線形モデル(詳細モデル)
```math
D(\theta) = AB -m^2l^2cos^2\theta
```

```math
\ddot{x} = \frac{B(\frac{u}{r} + mlsin\theta\dot{\theta}^2) - mlcos\theta(-u + mglsin\theta)}{D(\theta)}
```

```math
\ddot{\theta} = \frac{A(-u + mglsin(\theta)) - mlcos\theta(\frac{u}{r} + mlsin\theta\dot{\theta}^2)}{D(\theta)}
```

## 線形化モデル(公称モデル)
- 平衡点 $\theta=0$ で線形近似
- 線形近似はテイラー展開の1次の項のみを使う

```math
\begin{aligned}
sin\theta &= \theta \\
cos\theta &= 1
\end{aligned}
```

運動方程式
```math
\begin{aligned}
A\ddot{x} + ml\ddot{\theta} &= \frac{u}{r} \\
ml\ddot{x} + B\ddot{\theta} - mgl\theta &= -u
\end{aligned}
```

微分方程式
```math
\begin{aligned}
\ddot{x} &= -\frac{m^2gl^2}{D(\theta)}\theta + \frac{\frac{B}{r} + ml}{D(\theta)}u \\
\ddot{\theta} &= \frac{Amgl}{D(\theta)}\theta - \frac{A + \frac{ml}{r}}{D(\theta)} 
\end{aligned}u
```

# 制御モデル

### 制御入力
- $u$ モーターの駆動トルク 

### 制御出力
- $\ddot{x}$ 車軸の水平加速度
- $\ddot{\theta}$ 車体の傾斜角の角加速度

### 1入力2出力モデル
- 状態変数 $x = [x, \theta, \dot{x}, \dot{\theta}]^T$

### 状態空間モデル
```math
A_s =
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & -\frac{m^2gl^2}{D(\theta)} & 0 & 0 \\
0 & \frac{Amgl}{D(\theta)} & 0 & 0 
\end{bmatrix}
```

```math
B_s =
\begin{bmatrix}
0 \\
0 \\
\frac{\frac{B}{r} + ml}{D(\theta)} \\
-\frac{A + \frac{ml}{r}}{D(\theta)}
\end{bmatrix}
```

```math
\dot{\bold{x}} = A_s\bold{x} + B_su
```

## 伝達関数の導出
- 2つの運動方程式を初期値ゼロでラプラス変換する

運動方程式 (再掲)
```math
\begin{aligned}
A\ddot{x} + ml\ddot{\theta} &= \frac{u}{r} \\
ml\ddot{x} + B\ddot{\theta} - mgl\theta &= -u
\end{aligned}
```

ラプラス変換
```math
As^2X(s) + mls^2\Theta(s) = \frac{U(s)}{r}
```
```math
mls^2X(s) + (Bs^2 - mgl)\Theta(s) = - U(s)
```

連立方程式を解くと...

### 伝達関数(1)
- 入力: 駆動トルク $u=\tau$
- 出力: 傾斜角 $\theta$
- 分母多項式の極の1つが正になるので不安定

```math
G_\theta(s) = \frac{\Theta(s)}{U(s)}
```

```math
G_\theta(s) = - \frac{A + \frac{ml}{r}}{Ds^2 - Amgl}
```

```math
G_\theta(s) = - \frac{(A+\frac{ml}{r})/D}{s^2 - Amgl/D}
```

### 伝達関数(2)
- 入力: 駆動トルク $u=\tau$
- 出力: 車軸の位置 $x$

```math
G_x(s) = \frac{X(s)}{U(s)}
```

```math
G_x(s) = \frac{(\frac{B}{r} + ml)s^2 - \frac{mgl}{r}}{s^2(Ds^2 - Amgl)}
```

