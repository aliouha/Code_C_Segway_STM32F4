/* ============================================================
 *  segway_control.c
 *  Segway Pendule Inversé — Implémentation LQG complète
 *
 *  BOUCLE PRINCIPALE (500 Hz — toutes les 2ms) :
 *    1. Lire IMU MPU6050         → θ, θ̇
 *    2. Lire encodeurs           → x, ẋ
 *    3. Kalman PREDICT           → propager état estimé
 *    4. Kalman UPDATE            → corriger avec mesures
 *    5. LQR compute              → F = -K * x_hat
 *    6. Envoyer commande VESC    → couple moteur
 *    7. Sécurité                 → coupure si θ > 30°
 *  ============================================================ */

#include "segway_control.h"

/* ════════════════════════════════════════════════════════════
 *  VARIABLES GLOBALES
 * ════════════════════════════════════════════════════════════ */

static Kalman_t     kf;           /* filtre de Kalman        */
static Controller_t ctrl;         /* état contrôleur         */

/* Matrices système linéarisées (calculées une seule fois)
   d = M + m = 5.0 kg  */
static float Ad[4][4];   /* A discrétisé = I + A*dt          */
static float Bd[4];      /* B discrétisé = B*dt              */
static float C[2][4] = { /* matrice observation : mesure x, θ */
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f}
};

/* Bruits processus et mesure */
static float Q_proc[4] = {Q_NOISE_X, Q_NOISE_XD, Q_NOISE_TH, Q_NOISE_THD};
static float R_meas[2] = {R_NOISE_X, R_NOISE_THETA};

/* Variables encodeurs */
static int32_t  enc_prev_left  = 0;
static int32_t  enc_prev_right = 0;
static float    x_position     = 0.0f;
static float    x_velocity     = 0.0f;

/* ════════════════════════════════════════════════════════════
 *  INIT — MATRICES SYSTÈME
 * ════════════════════════════════════════════════════════════ */

void Segway_InitMatrices(void)
{
    float d  = M_CHASSIS + M_PILOT;      /* = 5.0  */
    float Ld = d * L_PENDULE;             /* = 2.0  */
    float b  = B_FRICTION;
    float m  = M_PILOT;
    float g  = G_GRAVITY;
    float L  = L_PENDULE;

    /*
     *  Matrice A continue (linéarisée θ=0) :
     *  A = [ 0      1          0              0      ]
     *      [ 0    -b/d       -mg/d            0      ]
     *      [ 0      0          0              1      ]
     *      [ 0    b/Ld    (M+m)g/Ld           0      ]
     *
     *  Discrétisation Euler : Ad = I + A*dt
     */

    /* Ligne 0 : dx/dt = ẋ */
    Ad[0][0] = 1.0f;
    Ad[0][1] = DT;
    Ad[0][2] = 0.0f;
    Ad[0][3] = 0.0f;

    /* Ligne 1 : dẋ/dt = -b/d*ẋ - mg/d*θ + F/d */
    Ad[1][0] = 0.0f;
    Ad[1][1] = 1.0f - (b / d) * DT;
    Ad[1][2] = -(m * g / d) * DT;
    Ad[1][3] = 0.0f;

    /* Ligne 2 : dθ/dt = θ̇ */
    Ad[2][0] = 0.0f;
    Ad[2][1] = 0.0f;
    Ad[2][2] = 1.0f;
    Ad[2][3] = DT;

    /* Ligne 3 : dθ̇/dt = b/Ld*ẋ + (M+m)g/Ld*θ - F/Ld */
    Ad[3][0] = 0.0f;
    Ad[3][1] = (b / Ld) * DT;
    Ad[3][2] = (d * g / Ld) * DT;
    Ad[3][3] = 1.0f;

    /*
     *  Vecteur B discret :
     *  B = [ 0,  1/d,  0,  -1/Ld ]^T  * dt
     */
    Bd[0] =  0.0f;
    Bd[1] =  (1.0f / d)  * DT;
    Bd[2] =  0.0f;
    Bd[3] = -(1.0f / Ld) * DT;
}

/* ════════════════════════════════════════════════════════════
 *  INIT — KALMAN
 * ════════════════════════════════════════════════════════════ */

void Kalman_Init(void)
{
    /* État initial : Segway supposé vertical, à l'arrêt */
    kf.x_hat.x      = 0.0f;
    kf.x_hat.xd     = 0.0f;
    kf.x_hat.theta  = 0.0f;
    kf.x_hat.thetad = 0.0f;

    /* Covariance initiale P = 0.1 * I */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            kf.P[i][j] = (i == j) ? 0.1f : 0.0f;
}

/* ════════════════════════════════════════════════════════════
 *  KALMAN — ÉTAPE PREDICT
 *
 *  x̂⁻ = Ad * x̂ + Bd * u
 *  P⁻  = Ad * P * Ad' + Q
 * ════════════════════════════════════════════════════════════ */

void Kalman_Predict(float u)
{
    float *x = (float*)&kf.x_hat;
    float x_pred[4];

    /* x̂⁻ = Ad * x̂ + Bd * u */
    for (int i = 0; i < 4; i++) {
        x_pred[i] = Bd[i] * u;
        for (int j = 0; j < 4; j++)
            x_pred[i] += Ad[i][j] * x[j];
    }
    kf.x_hat.x      = x_pred[0];
    kf.x_hat.xd     = x_pred[1];
    kf.x_hat.theta  = x_pred[2];
    kf.x_hat.thetad = x_pred[3];

    /* P⁻ = Ad * P * Ad' + Q_proc */
    /* Étape intermédiaire : tmp = Ad * P */
    float tmp[4][4], P_pred[4][4];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                tmp[i][j] += Ad[i][k] * kf.P[k][j];
        }

    /* P_pred = tmp * Ad' + Q */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            P_pred[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                P_pred[i][j] += tmp[i][k] * Ad[j][k]; /* Ad'[k][j] = Ad[j][k] */
            if (i == j)
                P_pred[i][j] += Q_proc[i];
        }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            kf.P[i][j] = P_pred[i][j];
}

/* ════════════════════════════════════════════════════════════
 *  KALMAN — ÉTAPE UPDATE
 *
 *  y    = mesures [ x_enc, theta_imu ]
 *  S    = C * P⁻ * C' + R          (2x2)
 *  K    = P⁻ * C' * S⁻¹            (4x2)
 *  x̂    = x̂⁻ + K * (y - C * x̂⁻)
 *  P    = (I - K*C) * P⁻
 * ════════════════════════════════════════════════════════════ */

void Kalman_Update(float y_x, float y_theta)
{
    float y[2] = {y_x, y_theta};
    float *x   = (float*)&kf.x_hat;

    /* Innovation : innovation = y - C * x̂⁻  (taille 2) */
    float innov[2];
    for (int i = 0; i < 2; i++) {
        innov[i] = y[i];
        for (int j = 0; j < 4; j++)
            innov[i] -= C[i][j] * x[j];
    }

    /* S = C * P * C' + R   (2x2) */
    float CP[2][4], S[2][2];
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++) {
            CP[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                CP[i][j] += C[i][k] * kf.P[k][j];
        }

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            S[i][j] = (i == j) ? R_meas[i] : 0.0f;
            for (int k = 0; k < 4; k++)
                S[i][j] += CP[i][k] * C[j][k]; /* C'[k][j] = C[j][k] */
        }

    /* S⁻¹  (inverse 2x2) :
       | a  b |⁻¹ = 1/(ad-bc) * | d -b |
       | c  d |                  |-c  a |   */
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabsf(det) < 1e-10f) return;   /* sécurité division par zéro */

    float S_inv[2][2];
    S_inv[0][0] =  S[1][1] / det;
    S_inv[0][1] = -S[0][1] / det;
    S_inv[1][0] = -S[1][0] / det;
    S_inv[1][1] =  S[0][0] / det;

    /* K = P * C' * S⁻¹  (4x2) */
    float PCT[4][2], K[4][2];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) {
            PCT[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                PCT[i][j] += kf.P[i][k] * C[j][k];
        }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 2; k++)
                K[i][j] += PCT[i][k] * S_inv[k][j];
        }

    /* x̂ = x̂⁻ + K * innov */
    float x_upd[4];
    for (int i = 0; i < 4; i++) {
        x_upd[i] = x[i];
        for (int j = 0; j < 2; j++)
            x_upd[i] += K[i][j] * innov[j];
    }
    kf.x_hat.x      = x_upd[0];
    kf.x_hat.xd     = x_upd[1];
    kf.x_hat.theta  = x_upd[2];
    kf.x_hat.thetad = x_upd[3];

    /* P = (I - K*C) * P */
    float KC[4][4], P_new[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            KC[i][j] = 0.0f;
            for (int k = 0; k < 2; k++)
                KC[i][j] += K[i][k] * C[k][j];
        }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float IKC = (i == j) ? 1.0f - KC[i][j] : -KC[i][j];
            P_new[i][j] = 0.0f;
            for (int k = 0; k < 4; k++)
                P_new[i][j] += IKC * kf.P[k][j];
            /* Note: simplification — pour STM32 temps réel :
               P = (I-KC)*P est suffisant (form. Joseph si besoin) */
        }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            kf.P[i][j] = P_new[i][j];
}

/* ════════════════════════════════════════════════════════════
 *  LQR — CALCUL COMMANDE
 *
 *  F = -K * x̂    avec K calculé offline par Python (Riccati)
 *  K = [-3.162, -11.821, -125.141, -29.044]
 *
 *  Saturation : |F| ≤ F_MAX_N
 * ════════════════════════════════════════════════════════════ */

float LQR_Compute(const State_t *st)
{
    float F = -(K_X      * st->x
              + K_XD     * st->xd
              + K_THETA  * st->theta
              + K_THETAD * st->thetad);

    /* Saturation force */
    if (F >  F_MAX_N) F =  F_MAX_N;
    if (F < -F_MAX_N) F = -F_MAX_N;

    return F;
}

/* ════════════════════════════════════════════════════════════
 *  LECTURE IMU MPU6050 (I2C)
 *
 *  Le MPU6050 donne :
 *    - accéléromètre 3 axes  (int16_t)
 *    - gyroscope 3 axes      (int16_t)
 *
 *  On fusionne via filtre complémentaire :
 *    θ = α*(θ + θ̇*dt) + (1-α)*atan2(ax, az)
 *    α = 0.98  (favorise gyro)
 * ════════════════════════════════════════════════════════════ */

/* Adresses MPU6050 */
#define MPU6050_ADDR    0x68
#define MPU_ACCEL_REG   0x3B
#define MPU_GYRO_REG    0x43
#define MPU_PWR_REG     0x6B

/* Sensibilités par défaut */
#define ACCEL_SCALE     16384.0f   /* LSB/g  — ±2g range  */
#define GYRO_SCALE      131.0f     /* LSB/(°/s) — ±250°/s */

static float theta_fused  = 0.0f;
static float thetad_fused = 0.0f;
#define COMP_ALPHA 0.98f

/* NOTE : MPU6050_ReadRaw() est une fonction HAL I2C (STM32 HAL)
   Signature réelle :
   HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR<<1, reg, 1, buf, len, timeout) */
void MPU6050_ReadAngles(float *theta_out, float *thetad_out)
{
    uint8_t buf[6];
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    /* Lire accéléromètre (6 octets depuis 0x3B) */
    /* HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR<<1, MPU_ACCEL_REG,
                        I2C_MEMADD_SIZE_8BIT, buf, 6, 10);          */
    ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    /* Lire gyroscope (6 octets depuis 0x43) */
    /* HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR<<1, MPU_GYRO_REG,
                        I2C_MEMADD_SIZE_8BIT, buf, 6, 10);          */
    gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
    gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
    gz_raw = (int16_t)((buf[4] << 8) | buf[5]);

    /* Conversion en unités physiques */
    float ax = (float)ax_raw / ACCEL_SCALE;  /* en g */
    float az = (float)az_raw / ACCEL_SCALE;
    float gy = (float)gy_raw / GYRO_SCALE;   /* en °/s */

    /* Angle depuis accéléromètre */
    float theta_accel = atan2f(ax, az);      /* rad */

    /* Vitesse angulaire depuis gyroscope */
    float thetad_gyro = gy * (float)M_PI / 180.0f;  /* rad/s */

    /* Filtre complémentaire */
    theta_fused  = COMP_ALPHA * (theta_fused + thetad_gyro * DT)
                 + (1.0f - COMP_ALPHA) * theta_accel;
    thetad_fused = thetad_gyro;

    *theta_out  = theta_fused;
    *thetad_out = thetad_fused;

    (void)ay_raw; (void)gx_raw; (void)gz_raw; /* évite warnings */
}

/* ════════════════════════════════════════════════════════════
 *  LECTURE ENCODEURS (Timers en mode QEI)
 *
 *  TIM2 : encodeur roue gauche
 *  TIM3 : encodeur roue droite
 *
 *  x    = (enc_L + enc_R) / 2 * (2π*R / CPR)
 *  ẋ    = Δx / dt
 * ════════════════════════════════════════════════════════════ */

void Encoders_ReadPosition(float *x_out, float *xd_out)
{
    /* Lire compteurs timers (registre CNT) */
    /* int32_t enc_L = (int32_t)TIM2->CNT;
       int32_t enc_R = (int32_t)TIM3->CNT;         */
    int32_t enc_L = 0;  /* remplacer par TIM2->CNT  */
    int32_t enc_R = 0;  /* remplacer par TIM3->CNT  */

    /* Delta impulsions depuis dernière lecture */
    int32_t delta_L = enc_L - enc_prev_left;
    int32_t delta_R = enc_R - enc_prev_right;
    enc_prev_left   = enc_L;
    enc_prev_right  = enc_R;

    /* Conversion impulsions → mètres */
    float dist_per_pulse = (2.0f * (float)M_PI * WHEEL_RADIUS) / (float)ENCODER_CPR;
    float dx = ((float)(delta_L + delta_R) / 2.0f) * dist_per_pulse;

    x_position += dx;
    x_velocity  = dx / DT;

    *x_out  = x_position;
    *xd_out = x_velocity;
}

/* ════════════════════════════════════════════════════════════
 *  ENVOI COMMANDE VESC (UART)
 *
 *  VESC reçoit une commande de courant (A) ou de couple (N.m)
 *  via protocole VESC UART (paquet COMM_SET_CURRENT)
 *
 *  Force → Couple : τ = F * R_roue
 *  Couple → Courant : I = τ / (Kt * n_moteurs)
 * ════════════════════════════════════════════════════════════ */

#define VESC_START_BYTE     0x02
#define VESC_CMD_CURRENT    0x06    /* COMM_SET_CURRENT */
#define KT_MOTOR            0.08f   /* N.m/A — constante couple moteur */
#define N_MOTORS            2

/* Protocole VESC : paquet = [0x02][len][cmd][data 4 bytes][CRC][0x03] */
static uint8_t vesc_packet[10];

uint8_t VESC_CRC(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return (uint8_t)(crc & 0xFF);
}

void VESC_SendForce(float F_newton)
{
    /* Conversion force → couple → courant */
    float torque  = F_newton * WHEEL_RADIUS;

    /* Saturation couple */
    if (torque >  TORQUE_MAX_NM) torque =  TORQUE_MAX_NM;
    if (torque < -TORQUE_MAX_NM) torque = -TORQUE_MAX_NM;

    float current = torque / (KT_MOTOR * N_MOTORS);

    /* Encodage courant en int32 (VESC : mA * 1000) */
    int32_t current_raw = (int32_t)(current * 1000.0f);

    /* Construction paquet VESC */
    vesc_packet[0] = VESC_START_BYTE;
    vesc_packet[1] = 5;                        /* longueur payload  */
    vesc_packet[2] = VESC_CMD_CURRENT;
    vesc_packet[3] = (current_raw >> 24) & 0xFF;
    vesc_packet[4] = (current_raw >> 16) & 0xFF;
    vesc_packet[5] = (current_raw >> 8)  & 0xFF;
    vesc_packet[6] = (current_raw)       & 0xFF;
    vesc_packet[7] = VESC_CRC(&vesc_packet[2], 5);
    vesc_packet[8] = 0x03;                     /* stop byte         */

    /* Envoyer via UART2 :
       HAL_UART_Transmit(&huart2, vesc_packet, 9, 5);               */
}

/* ════════════════════════════════════════════════════════════
 *  INIT SYSTÈME
 * ════════════════════════════════════════════════════════════ */

void Segway_Init(void)
{
    /* Init matrices Ad, Bd */
    Segway_InitMatrices();

    /* Init Kalman */
    Kalman_Init();

    /* Reset contrôleur */
    ctrl.active      = false;
    ctrl.fallen      = false;
    ctrl.force_cmd   = 0.0f;
    ctrl.loop_count  = 0;

    /* Init MPU6050 :
       uint8_t wake = 0x00;
       HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR<<1, MPU_PWR_REG,
                          I2C_MEMADD_SIZE_8BIT, &wake, 1, 10);
       HAL_Delay(100);                                               */

    /* Init timers encodeurs en mode QEI :
       HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
       HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);              */

    /* Attendre stabilisation IMU (500ms) */
    /* HAL_Delay(500); */

    /* Activer le contrôle */
    ctrl.active = true;
}

/* ════════════════════════════════════════════════════════════
 *  BOUCLE PRINCIPALE — appelée par Timer Interrupt (500 Hz)
 *
 *  Cette fonction est appelée depuis :
 *  void TIM6_DAC_IRQHandler(void) {
 *      HAL_TIM_IRQHandler(&htim6);
 *  }
 *  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *      if (htim->Instance == TIM6) Segway_ControlLoop();
 *  }
 * ════════════════════════════════════════════════════════════ */

void Segway_ControlLoop(void)
{
    if (!ctrl.active) return;

    ctrl.loop_count++;

    /* ── ÉTAPE 1 : Lecture capteurs ─────────────────────── */
    float theta_meas, thetad_meas;
    float x_meas, xd_meas;

    MPU6050_ReadAngles(&theta_meas, &thetad_meas);
    Encoders_ReadPosition(&x_meas, &xd_meas);

    /* ── ÉTAPE 2 : Sécurité — détection chute ───────────── */
    float theta_deg = theta_meas * 180.0f / (float)M_PI;
    if (fabsf(theta_deg) > THETA_MAX_DEG) {
        ctrl.fallen    = true;
        ctrl.active    = false;
        ctrl.force_cmd = 0.0f;
        VESC_SendForce(0.0f);
        /* HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); LED ON */
        return;
    }

    /* ── ÉTAPE 3 : Kalman PREDICT ───────────────────────── */
    Kalman_Predict(ctrl.force_cmd);

    /* ── ÉTAPE 4 : Kalman UPDATE ────────────────────────── */
    Kalman_Update(x_meas, theta_meas);

    /* ── ÉTAPE 5 : LQR — calcul commande ───────────────── */
    ctrl.state_est = kf.x_hat;
    ctrl.force_cmd = LQR_Compute(&ctrl.state_est);

    /* ── ÉTAPE 6 : Envoi commande moteurs ──────────────── */
    VESC_SendForce(ctrl.force_cmd);

    /* ── ÉTAPE 7 : Debug UART (optionnel, 10 Hz) ────────── */
    if (ctrl.loop_count % 50 == 0) {
        /* Envoyer télémétrie debug via UART1 à 10 Hz :
           char dbg[80];
           snprintf(dbg, sizeof(dbg),
                    "t=%lu x=%.3f xd=%.3f th=%.4f thd=%.4f F=%.2f\r\n",
                    ctrl.loop_count,
                    ctrl.state_est.x,
                    ctrl.state_est.xd,
                    ctrl.state_est.theta,
                    ctrl.state_est.thetad,
                    ctrl.force_cmd);
           HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 5); */
    }
}

/* ════════════════════════════════════════════════════════════
 *  MAIN — Point d'entrée STM32
 * ════════════════════════════════════════════════════════════ */

int main(void)
{
    /* Init HAL (généré par CubeMX) */
    /* HAL_Init();
       SystemClock_Config();       -- 168 MHz
       MX_GPIO_Init();
       MX_I2C1_Init();            -- MPU6050
       MX_UART1_Init();           -- Debug 115200
       MX_UART2_Init();           -- VESC 115200
       MX_TIM2_Init();            -- Encodeur gauche QEI
       MX_TIM3_Init();            -- Encodeur droit  QEI
       MX_TIM6_Init();            -- Timer contrôle 500 Hz    */

    /* Init Segway */
    Segway_Init();

    /* Boucle infinie — le contrôle tourne en interruption TIM6 */
    while (1) {
        /* Tâches basse priorité : communication PC, monitoring... */
        /* HAL_Delay(1); */
    }
}