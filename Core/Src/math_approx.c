#include "math_approx.h"

// 近似的asinf函数，输入范围为[-1, 1]，输出范围为[-pi/2, pi/2]
float asinf(float x)
{
    // 使用泰勒级数展开的前几项来近似计算asinf
    // asinf(x) ≈ x + (1/6)*x^3 + (3/40)*x^5 + (5/112)*x^7
    float x2 = x * x;
    return x + (1.0f / 6.0f) * x * x2 + (3.0f / 40.0f) * x * x2 * x2 + (5.0f / 112.0f) * x * x2 * x2 * x2;
}