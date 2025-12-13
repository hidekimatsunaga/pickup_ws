#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> pickup_sequence = {
    {414.14f, 450.88f, 380.56f, 225.80f, 154.52f, 183.96f, 540.88f, 499.66f, 488.24f, 300.00f},
    {418.44f, 466.52f, 763.50f, 230.02f, 161.72f, 682.47f, 623.76f, 584.91f, 307.88f, 300.00f},
    {534.28f, 724.75f, 1092.22f, 369.24f, 576.39f, 986.66f, 690.03f, 672.36f, 380.48f, 300.00f}
};

} //namespace motor_sequences