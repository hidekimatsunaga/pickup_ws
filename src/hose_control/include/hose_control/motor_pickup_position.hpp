#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> pickup_sequence = {
    {414.93f, 460.63f, 361.23f, 228.08f, 153.29f, 170.69f, 404.56f, 491.31f, 481.03f, 196.00f},
    {419.23f, 476.27f, 744.17f, 232.30f, 160.49f, 669.20f, 487.44f, 576.56f, 300.67f, 196.00f},
    {535.07f, 734.50f, 1072.89f, 371.52f, 575.16f, 973.39f, 553.71f, 664.01f, 373.27f, 196.00f}
};

} //namespace motor_sequences