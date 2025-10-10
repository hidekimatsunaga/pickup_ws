#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> pickup_sequence = {
    {497.72, 523.83, 495.88, 234.58, 250.05, 283.62, 573.49, 511.35, 485.33},  
    {587.37, 1178.44, 863.79, 381.97, 450.62, 462.66, 772.65, 722.29, 779.24},  
};

} //namespace motor_sequences