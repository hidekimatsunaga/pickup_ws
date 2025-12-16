#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> narrow_sequence = {
    {603.28, 539.3, 892.35, 168.13, 61.96, 162.42, 360.62, 578.87, 440.21, -7700},
    {574.45f, 549.14f, 770.45f, 287.93f, 189.57f, 249.69f, 460.20f, 663.75f, 715.69f, -11772.00f},
    {286.17f, 707.17f, 1119.73f, 431.02f, 371.51f, 233.52f, 686.78f, 661.20f, 627.01f, -13900.00f},
    {574.45f, 549.14f, 770.45f, 287.93f, 189.57f, 249.69f, 460.20f, 663.75f, 715.69f, -11772.00f}
};

} //namespace motor_sequences