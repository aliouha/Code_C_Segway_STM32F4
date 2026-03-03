# Segway STM32 — Code embarqué LQG

## Fichiers

| Fichier | Rôle |
|---|---|
| `segway_control.h` | Constantes, paramètres, structures |
| `segway_control.c` | Implémentation complète LQG + drivers |

---

## Architecture logicielle

```
TIM6 IRQ (500 Hz — toutes les 2ms)
    │
    ├── 1. MPU6050_ReadAngles()     → θ, θ̇  (I2C1)
    ├── 2. Encoders_ReadPosition()  → x, ẋ  (TIM2+TIM3 QEI)
    ├── 3. Sécurité θ > 30° ?       → coupure moteurs
    ├── 4. Kalman_Predict(F)        → x̂⁻, P⁻
    ├── 5. Kalman_Update(x, θ)      → x̂, P
    ├── 6. LQR_Compute(x̂)          → F = -K*x̂
    └── 7. VESC_SendForce(F)        → UART2 → moteurs
```

---

## Matériel requis

| Composant | Modèle | Interface |
|---|---|---|
| Microcontrôleur | STM32F4 Discovery | — |
| IMU | MPU6050 | I2C1 (PB8/PB9) |
| Encodeur gauche | ABZ 4096 CPR | TIM2 (PA0/PA1) |
| Encodeur droit | ABZ 4096 CPR | TIM3 (PA6/PA7) |
| Driver moteur | VESC 6 ou ODrive | UART2 (PA2/PA3) |
| Debug | USB-UART | UART1 (PA9/PA10) |

---

## Configuration STM32CubeMX

```
System Core → TIM6 : période = 168MHz / (168*2) = 500Hz, NVIC enable
Connectivity → I2C1 : Fast Mode 400kHz
Connectivity → USART1 : 115200, debug
Connectivity → USART2 : 115200, VESC
Timers → TIM2 : Encoder Mode, period=0xFFFF
Timers → TIM3 : Encoder Mode, period=0xFFFF
```

---

## Gains LQR (calculés offline par Python)

```
Q = diag(1, 1, 100, 10)    R = 0.1

K = [-3.162,  -11.821,  -125.141,  -29.044]
     K_x       K_ẋ        K_θ        K_θ̇
```

Recalculer avec `scipy.linalg.solve_continuous_are(A, B, Q, R)` si les paramètres physiques changent.

---

## Brancher le MPU6050

```
MPU6050       STM32F4 Discovery
VCC    →      3.3V
GND    →      GND
SDA    →      PB9  (I2C1_SDA)
SCL    →      PB8  (I2C1_SCL)
AD0    →      GND  (adresse 0x68)
INT    →      (optionnel — mode interrupt)
```

---

## Fréquences recommandées

```
Boucle contrôle LQG  : 500 Hz   (TIM6 IRQ)
Lecture IMU          : 500 Hz   (même boucle)
Lecture encodeurs    : 500 Hz   (même boucle)
Debug UART           :  10 Hz   (tous les 50 cycles)
Commande VESC        : 500 Hz   (même boucle)
```