#pragma once

#include <vector>

namespace motor_sequences{

// 8x9の行列としてデータを保存
inline const std::vector<std::vector<float>> pickup_sequence = {
    {168.0f, 237.0f, 178.0f, 10.0f, 16.0f, -85.0f, 42.0f, 471.0f, 527.0f},  // b
    {168.0f, 237.0f, 178.0f, 10.0f, 16.0f, -85.0f, 668.0f, 471.0f, 527.0f},  // c
    {168.0f, 366.0f, 852.0f, 1.0f, 185.0f, 1062.0f, 659.0f, 846.0f, 207.0f},  // h
    {342.0f, 743.0f, 1012.0f, 327.0f, 224.0f, 1213.0f, 659.0f, 846.0f, 207.0f}, // f
    {334.0f, 749.0f, 1142.0f, 335.0f, 678.0f, 1167.0f, 668.0f, 327.0f, 201.0f} // p
};

} //namespace motor_sequences