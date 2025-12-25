#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> narrow_sequence = {
    {465.64, 540.01, 522.16, 318.43, 241.79, 226.14, 564.70, 775.11, 647.14, -14000},
    {516.18, 570.76, 770.01, 261.39, 207.16, 224.91, 491.31, 786.80, 548.19, -14000},
    {486.75, 508.10, 522.77, 252.69, 195.47, 212.26, 491.84, 788.47, 543.16, -14000} //以前のn
};

} //namespace motor_sequences