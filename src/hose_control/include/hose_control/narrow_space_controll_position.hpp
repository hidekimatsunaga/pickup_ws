#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> narrow_sequence = {
    {497.72f, 523.83f, 495.88f, 234.58f, 250.05f, 283.62f, 573.49f, 511.35f, 485.33f, -14000.00f},
    {545.27f, 552.92f, 801.30f, 276.24f, 183.60f, 215.68f, 428.29f, 747.07f, 710.24f, -11772.00f},
    {256.99f, 710.95f, 1150.58f, 419.33f, 365.54f, 199.51f, 654.87f, 744.52f, 621.56f, -13900.00f},
    {545.27f, 552.92f, 801.30f, 276.24f, 183.60f, 215.68f, 428.29f, 747.07f, 710.24f, -11772.00f}
};

} //namespace motor_sequences