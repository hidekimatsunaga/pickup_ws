#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> narrow_sequence = {
    {599.15f, 532.62f, 875.03f, 160.04f, 53.88f, 144.23f, 242.67f, 574.21f, 437.05f, -7804.00f},
    {570.32f, 542.46f, 753.13f, 279.84f, 181.49f, 231.50f, 342.25f, 659.09f, 712.53f, -11772.00f},
    {282.04f, 700.49f, 1102.41f, 422.93f, 363.43f, 215.33f, 568.83f, 656.54f, 623.85f, -13900.00f},
    {570.32f, 542.46f, 753.13f, 279.84f, 181.49f, 231.50f, 342.25f, 659.09f, 712.53f, -11772.00f}
};

} //namespace motor_sequences