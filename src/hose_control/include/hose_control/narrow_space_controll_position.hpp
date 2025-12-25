#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> narrow_sequence = {
    {463.97, 508.18, 522.25, 320.98, 199.78, 221.57, 565.4, 775.81, 542.72, -14000},
    {516.18, 570.76, 770.01, 261.39, 207.16, 224.91, 491.31, 786.80, 548.19, -14000},
    {486.75, 508.10, 522.77, 252.69, 195.47, 212.26, 491.84, 788.47, 543.16, -14000} //以前のn
};

} //namespace motor_sequences