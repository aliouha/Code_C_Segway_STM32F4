/* ============================================================
 *  segway_control.h
 *  Segway Pendule Inversé — Contrôle LQG (LQR + Kalman)
 *  Microcontrôleur : STM32F4xx  |  Fréquence boucle : 500 Hz
 *
 *  Auteur  : Aliou Harber
 *  Matériel:
 *    - IMU    : MPU6050 (I2C1)  → mesure θ + θ̇
 *    - Encodeurs : TIM2 + TIM3  → mesure x + ẋ
 *    - Moteurs : VESC via UART2 → commande couple
 *    - LED debug : PC13
 *  ============================================================ */

#ifndef SEGWAY_CONTROL_H
#define SEGWAY_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ── Paramètres physiques ── */
#define M_CHASSIS       2.0f      /* kg  — masse châssis              */
#define M_PILOT         3.0f      /* kg  — masse pilote               */
#define L_PENDULE       0.4f      /* m   — hauteur centre de gravité  */
#define G_GRAVITY       9.81f     /* m/s²                             */
#define B_FRICTION      0.3f      /* N.m.s — frottement               */
#define WHEEL_RADIUS    0.125f    /* m   — rayon roue                 */
#define ENCODER_CPR     4096      /* impulsions/tour encodeur         */

/* ── Boucle de contrôle ── */
#define CONTROL_HZ      500       /* Hz — fréquence boucle            */
#define DT              (1.0f / CONTROL_HZ)   /* s — pas de temps     */

/* ── Sécurité ── */
#define THETA_MAX_DEG   30.0f     /* ° — angle max avant coupure      */
#define F_MAX_N         80.0f     /* N — force moteur max             */
#define TORQUE_MAX_NM   15.0f     /* N.m — couple max moteur          */

/* ── Gains LQR (calculés par Python/scipy — Riccati) ── */
/* Q = diag(1, 1, 100, 10)   R = 0.1                    */
#define K_X         3.162f    /* gain position x             */
#define K_XD        11.821f   /* gain vitesse ẋ              */
#define K_THETA     125.141f  /* gain angle θ (dominant)     */
#define K_THETAD    29.044f   /* gain vitesse angulaire θ̇    */

/* ── Kalman — bruits capteurs ── */
#define Q_NOISE_X       0.001f   /* covariance bruit processus x   */
#define Q_NOISE_XD      0.01f    /* covariance bruit processus ẋ   */
#define Q_NOISE_TH      0.001f   /* covariance bruit processus θ   */
#define Q_NOISE_THD     0.01f    /* covariance bruit processus θ̇   */
#define R_NOISE_X       0.05f    /* bruit encodeur                 */
#define R_NOISE_THETA   0.005f   /* bruit IMU (rad²)               */

/* ── Structures ── */
typedef struct {
    float x;        /* position chariot (m)        */
    float xd;       /* vitesse linéaire (m/s)      */
    float theta;    /* angle inclinaison (rad)      */
    float thetad;   /* vitesse angulaire (rad/s)   */
} State_t;

typedef struct {
    float P[4][4];  /* matrice covariance estimée  */
    State_t x_hat;  /* état estimé                 */
} Kalman_t;

typedef struct {
    bool  active;        /* système actif               */
    bool  fallen;        /* chute détectée              */
    float force_cmd;     /* commande force (N)          */
    float torque_cmd;    /* commande couple (N.m)       */
    State_t state_est;   /* état estimé par Kalman      */
    uint32_t loop_count; /* compteur boucles            */
} Controller_t;

#endif /* SEGWAY_CONTROL_H */