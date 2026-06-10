#include "motor/svpwm.h"
#include "bsp/bsp_timer.h"
#include "motor/motor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.141592653589793f
#define SQRT3 1.732050807568877f
#define INV_SQRT3 0.577350269f

#define ONE_THIRD  (1.0f / 3.0f)
#define SQRT3_OVER_3 (SQRT3 / 3.0f)
#define TWO_SQRT3_OVER_3 (2.0f * SQRT3 / 3.0f)
#define SVPWM_MAX_MODULATION 0.92f  // 最大调制指数，考虑死区和最小采样时间的限制
/* 引入预生成的正弦表 (const float sin_table[360]) */
#include "svpwm_table.h"

// 定义插值模式：1为线性插值(Linear)，2为二次插值(Quadratic)

/**
 * @brief  电压矢量归一化与过调制保护
 * @note   1. 将输入的 d-q 轴电压指令归一化到 SVPWM 的调制范围 (0~1)，考虑母线电压和最大调制指数。
 *         2. 如果归一化后的电压矢量模长超过 SVPWM 的六边形内切圆 (mod_Ud^2 + mod_Uq^2 > SVPWM_MAX_MODULATION^2)，则等比例缩小到边界上，保持矢量方向不变。
 *
 * @param Vbus       母线电压 (V)
 * @param Ud         d 轴电压给定 (V)
 * @param Uq         q 轴电压给定 (V)
 * @param mod_Ud     归一化后的 d 轴调制电压
 * @param mod_Uq     归一化后的 q 轴调制电压
 */
void Vector_Normalize_Limit(float Vbus, float Ud, float Uq, float *mod_Ud, float *mod_Uq)
{
	// 第1步：电压 → 调制比（归一化到 SVM 调制范围）
    float V_to_mod = 1.0f / ((2.0f / 3.0f) * Vbus);
    *mod_Ud = V_to_mod * Ud;
    *mod_Uq = V_to_mod * Uq ;

	// 第2步：过调制保护 — 向量超出 SVM 六边形内切圆时等比缩小
    float mod_scalefactor = SVPWM_MAX_MODULATION * SQRT3 * 0.5f / sqrtf(*mod_Ud * *mod_Ud + *mod_Uq * *mod_Uq);
    if (mod_scalefactor < 1.0f) {
        *mod_Ud *= mod_scalefactor;
        *mod_Uq *= mod_scalefactor;
    }
}


/**
 * @brief 快速查表求正弦值 (带线性插值)
 * @note 1. 输入角度 angle 单位为度，范围不限，函数内部会进行模360处理。
 * 
 * @param angle      输入角度 (度)
 * @param sin_out    输出正弦值
 * @param cos_out    输出余弦值
 */
void fast_sin_cos(float angle,float *sin_out,float *cos_out)
 {

    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;          // → [0, 360)

    float cos_angle = angle + 90.0f;
    if (cos_angle >= 360.0f) cos_angle -= 360.0f;  // 简单比较，不用 fmodf

    int angle_int = (int)angle;
    int cos_angle_int = (int)cos_angle;
    float angle_frac = angle - (float)angle_int;
    float cos_angle_frac = cos_angle - (float)cos_angle_int;
    

    angle_int = angle_int % 360;
    cos_angle_int = cos_angle_int % 360;


    angle = (float)angle_int + angle_frac;
    cos_angle = (float)cos_angle_int + cos_angle_frac;

    int idx = (int)angle;           
    int cos_idx = (int)cos_angle; 

    float frac = angle - (float)idx;
    float cos_frac = cos_angle - (float)cos_idx; 


    float y_minus_1 = sin_table[(idx - 1 + 360) % 360];
    float y0 = sin_table[idx];
    float y1 = sin_table[(idx + 1) % 360];

    float cos_y_minus_1 = sin_table[(cos_idx - 1 + 360) % 360];
    float cos_y0 = sin_table[cos_idx];
    float cos_y1 = sin_table[(cos_idx + 1) % 360];

    *sin_out = y0 + 0.5f * frac * (y1 - y_minus_1 + frac * (y1 - 2.0f * y0 + y_minus_1));
    *cos_out = cos_y0 + 0.5f * cos_frac * (cos_y1 - cos_y_minus_1 + cos_frac * (cos_y1 - 2.0f * cos_y0 + cos_y_minus_1));

}


/**
 * @brief Clarke 变换 (恒幅值变换)
 * 依据博客推导的最终矩阵公式：
 * [ Ialpha ] = [ 1        0         ] [ ia ]
 * [ Ibeta  ]   [ sqrt(3)/3  2*sqrt(3)/3 ] [ ib ]
 * 因为 Ia + Ib + Ic = 0, 所以可以只通过两相电流求得 alpha-beta 分量
 */
void Clarke_Transform(float Ia, float Ib, float *Ialpha, float *Ibeta)
{
    // Ialpha = 1 * Ia + 0 * Ib
    *Ialpha = Ia;
    
    // Ibeta = (sqrt(3)/3) * Ia + (2*sqrt(3)/3) * Ib
    *Ibeta = SQRT3_OVER_3 * Ia + TWO_SQRT3_OVER_3 * Ib;
}

/**
 * @brief 逆 Park 变换 (InvPark Transform)
 * @note 将 d-q 轴的电压指令转换回 alpha-beta 轴，用于 SVPWM_Calc 的输入
 * 
 * @param Vd         d 轴电压给定 (V)
 * @param Vq         q 轴电压给定 (V)
 * @param angle       电角度 (度)     
 * @param Valpha      输出 alpha 轴电压分量 (V)
 * @param Vbeta       输出 beta 轴电压分量 (V)
 */
void InvPark_Transform(float Vd, float Vq, float angle, float *Valpha, float *Vbeta)
{
    float sin_val, cos_val;
    fast_sin_cos(angle, &sin_val, &cos_val);
    *Valpha = Vd * cos_val - Vq * sin_val;
    *Vbeta  = Vd * sin_val + Vq * cos_val;
}

/**
 * @brief Park 变换
 * 依据博客推导的最终公式：
 * Id = Ialpha * cos(theta) + Ibeta * sin(theta)
 * Iq = -Ialpha * sin(theta) + Ibeta * cos(theta)
 */
void Park_Transform(float Ialpha, float Ibeta, float angle, float *Id, float *Iq)
{
    float sin_val, cos_val;
    fast_sin_cos(angle, &sin_val, &cos_val);

    *Id = Ialpha * cos_val + Ibeta * sin_val;
    *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}





















// 移除原来的static声明
// static float fast_sin(float angle);
// static float fast_cos(float angle);

/**
 * @brief 初始化SVPWM模块
 * @note由于使用了静态const查表，此处无需运行时计算
 */
void SVPWM_Init(void)
{
    // 保留接口，目前无需执行内容
}
/**
 * @brief 电压矢量模长限幅 (适配直角坐标系/旋转坐标系)
 * @param Ualpha/Ud: alpha/d轴电压分量指针
 * @param Ubeta/Uq:  beta/q轴电压分量指针
 * @param U_max:     最大允许电压矢量模长 (例如 0.92f，考虑死区和最小采样时间)
 * @note  保持电压矢量方向不变，等比例缩小分量
 */
void Us_Limit(float *U_d_alpha, float *U_q_beta, float U_max)
{
    // 计算当前电压矢量模长的平方
    float U_square = (*U_d_alpha) * (*U_d_alpha) + (*U_q_beta) * (*U_q_beta);
    float U_max_square = U_max * U_max;

    // 如果当前模长超过最大值，进行等比例缩放
    if (U_square > U_max_square)
    {
        // 计算缩放比例: scale = U_max / sqrt(U_square)
        // 也可以用快速开方求反倒数算法 (Fast Inverse Square Root) 进一步优化
        float scale = U_max / sqrtf(U_square);
        
        *U_d_alpha *= scale;
        *U_q_beta  *= scale;
    }
}

/**
 * @brief SVPWM核心计算
 * @param Ualpha alpha轴电压分量 (归一化到 -1.0 ~ 1.0)
 * @param Ubeta  beta轴电压分量  (归一化到 -1.0 ~ 1.0)
 * @param mod_U_CCR U相比较寄存器归一化值，U相时间
 * @param mod_V_CCR V相比较寄存器归一化值，V相时间
 * @param mod_W_CCR W相比较寄存器归一化值，W相时间
 */
void SVPWM_Calc_Cartesian(float Ualpha, float Ubeta, float *mod_U_CCR, float *mod_V_CCR, float *mod_W_CCR)
{
    // 2. 扇区判断 (去掉 / 2.0f 不影响符号判断，且减少运算)
    float Uref1 = Ubeta;
    float Uref2 = SQRT3 * Ualpha - Ubeta;
    float Uref3 = -SQRT3 * Ualpha - Ubeta;

    uint8_t A = (Uref1 > 0.0f) ? 1 : 0;
    uint8_t B = (Uref2 > 0.0f) ? 1 : 0;
    uint8_t C = (Uref3 > 0.0f) ? 1 : 0;

    uint8_t sector = A + (B << 1) + (C << 2);
    
    // 更新全局电机状态结构体中的扇区信息（映射为1~6），供ADC注入组动态配置使用  、、wxy0505验证
    if (g_pstMotorData != NULL) {
        switch (sector)
        {
            case 3: g_pstMotorData->sector = 1; break;
            case 1: g_pstMotorData->sector = 2; break;
            case 5: g_pstMotorData->sector = 3; break;
            case 4: g_pstMotorData->sector = 4; break;
            case 6: g_pstMotorData->sector = 5; break;
            case 2: g_pstMotorData->sector = 6; break;
            default: g_pstMotorData->sector = 1; break;
        }
    }

    float Ubeta_div_sqrt3 = Ubeta * INV_SQRT3;
    float X = 2.0f * Ubeta_div_sqrt3;
    float Y = Ualpha + Ubeta_div_sqrt3;
    float Z = -Ualpha + Ubeta_div_sqrt3;

    // 4. 求解矢量作用时间 ta, tb
    float ta = 0.0f, tb = 0.0f;

    switch (sector)
    {
    case 3: // 扇区 一
        ta = X;
        tb = -Z;
        break;
    case 1: // 扇区 二
        ta = Y;
        tb = Z;
        break;
    case 5: // 扇区 三
        ta = -Y;
        tb = X;
        break;
    case 4: // 扇区 四
        ta = Z;
        tb = -X;
        break;
    case 6: // 扇区 五
        ta = -Z;
        tb = -Y;
        break;
    case 2: // 扇区 六
        ta = -X;
        tb = Y;
        break;
    default:
        ta = 0.0f;
        tb = 0.0f;
        break;

    }

    // 过调制处理：如果 ta + tb 大于单位时间 T，则按比例缩小
    if ((ta + tb) > 1.0f)
    {
        float sum = ta + tb;
        ta = ta * 1.0f / sum;
        tb = tb * 1.0f / sum;
    }

    // 5. 求解 tmin, tmid, tmax (7段式中心对齐计算)
    // tmin = 零矢量时间(下管全通), tmid = 第一矢量中间相, tmax = 第二矢量最大值
    float tmin = (1.0f - ta - tb) / 2.0f;
    float tmid = tmin + ta;
    float tmax = tmid + tb;


    switch (sector)
    {

    case 3: // 扇区 一
        *mod_U_CCR = tmax;
        *mod_V_CCR = tmid;
        *mod_W_CCR = tmin;
        break;
    case 1: // 扇区 二
        *mod_U_CCR = tmid;
        *mod_V_CCR = tmax;
        *mod_W_CCR = tmin;
        break;
    case 5: // 扇区 三
        *mod_U_CCR = tmin;
        *mod_V_CCR = tmax;
        *mod_W_CCR = tmid;
        break;
    case 4: // 扇区 四
        *mod_U_CCR = tmin;
        *mod_V_CCR = tmid;
        *mod_W_CCR = tmax;
        break;
    case 6: // 扇区 五
        *mod_U_CCR = tmid;
        *mod_V_CCR = tmin;
        *mod_W_CCR = tmax;
        break;
    case 2: // 扇区 六
        *mod_U_CCR = tmax;
        *mod_V_CCR = tmin;
        *mod_W_CCR = tmid;
        break;
    }

}

/**
 * @brief  死区补偿实现
 * @note   基于三相电流极性，在 α-β 轴上叠加补偿电压。
 *         补偿量 Vcomp = Tdead / Tpwm ≈ 200ns / 50μs = 0.004（归一化幅值），
 *         实际效果与母线电压、死区时间、负载电流相关，建议用串口观察电流波形微调。
 */
void Dead_Time_Compensate(float Iu, float Iv, float Iw, float *Ualpha, float *Ubeta)
{
    if (!Ualpha || !Ubeta) return;

    /* 死区补偿幅值（归一化），Tdead=200ns, Tpwm=50us @ 20kHz */
    /* 调试建议：以 0.001 为步长增减，观察电流过零是否平滑 */
    const float Vcomp = 0.004f;

    /* 电流极性的死区阈值（ADC 码值），防止过零附近极性频繁翻转 */
    const float dead_zone = 0.02f;   /* 过零死区（安培），对应约 10 ADC 码值 */

    float sign_u = 0.0f, sign_v = 0.0f, sign_w = 0.0f;
    if      (Iu >  dead_zone) sign_u =  1.0f;
    else if (Iu < -dead_zone) sign_u = -1.0f;
    if      (Iv >  dead_zone) sign_v =  1.0f;
    else if (Iv < -dead_zone) sign_v = -1.0f;
    if      (Iw >  dead_zone) sign_w =  1.0f;
    else if (Iw < -dead_zone) sign_w = -1.0f;

    /* 三相符号转 α-β 补偿电压 */
    float comp_a = Vcomp * (2.0f * sign_u - sign_v - sign_w) / 3.0f;
    float comp_b = Vcomp * (sign_v - sign_w) / 1.7320508f;

    *Ualpha += comp_a;
    *Ubeta  += comp_b;
}


