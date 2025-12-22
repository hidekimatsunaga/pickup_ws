#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> pickup_sequence = {
    {403.95f, 436.29f, 357.27f, 221.75f, 151.53f, 175.26f, 427.50f, 488.32f, 484.90f, 196.00f},
    {408.25f, 451.93f, 740.21f, 225.97f, 158.73f, 673.77f, 510.38f, 573.57f, 304.54f, 196.00f},
    {524.09f, 710.16f, 1068.93f, 365.19f, 573.40f, 977.96f, 576.65f, 661.02f, 377.14f, 196.00f}
};

} //namespace motor_sequences