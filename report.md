#  MuJoCo MPC 汽⻋仪表盘项⽬

## 项目信息
**学号**: 232011064
**姓名**: 杨德三
**班级**: 计科2302班
**完成日期** 2025年12月23日

## 项目概述
本作业基于 MuJoCo 与 mjpc（Model Predictive Control）框架，实现一个简化汽车模型的仪表盘系统，并实时输出位置，速度，油量的信息，用于直观展示车辆运行状态。
通过本实验，学生可以加深对 MuJoCo 数据结构、物理仿真流程以及实时渲染机制的理解，并掌握仿真数据到可视化结果的完整实现流程。
---

## 环境要求
- 操作系统：Ubuntu 22.04.5
- 编译器：clang12
- cmake：3.22.1

## 编译和运行

### 编译步骤
\`\`\`bash
cd mujoco_mp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE:STRING=Release -G Ninja -DCMAKE_C_COMPILER:STRING=clang-12 -DCMAKE_CXX_COMPILER:STRING=clang++-12 -DMJPC_BUILD_GRPC_SERVICE:BOOL=ON

cmake --build . --config=Release
\`\`\

### 运行
./mjpc --task=SimpleCar

## 功能说明
- [x] 速度表
- [x] 转速表
- [x] 加速度
- [x] 数字显示（油量）


