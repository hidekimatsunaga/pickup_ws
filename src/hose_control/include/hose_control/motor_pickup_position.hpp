#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> pickup_sequence = {
    {408.08f, 442.97f, 374.59f, 229.84f, 159.61f, 193.45f, 545.45f, 492.98f, 488.06f, 300.00f},
    {412.38f, 458.61f, 757.53f, 234.06f, 166.81f, 691.96f, 628.33f, 578.23f, 307.70f, 300.00f},
    {528.22f, 716.84f, 1086.25f, 373.28f, 581.48f, 996.15f, 694.60f, 665.68f, 380.30f, 300.00f}
};

} //namespace motor_sequences