# 在CBS系列算法中使用RL算法

技术路线：
1. 对单个节点，通过选择冲突加速CBS算法，以RL技术达到ICBS效果
2. 对每个节点进行打分，在HL选择节点扩展时进行排序，达到CBSH、CBSH2效果
3. 在HL扩展时进行剪枝，不在所有叶节点中选择，根据RL学习结果选择一个子节点，将算法时间复杂度降低到多项式级别

通过RL算法，无法保证解的最优性，尽量保持解的质量。

## 技术路线1

$state(k+1, m, n)$，其中$k$为agent数量，地图尺寸为$m \times n$

$action(a1, a2, t)$， 其中$a$为agent的id，$t$为时间

## 技术路线2

$state(k+1, m, n)$，其中$k$为agent数量，地图尺寸为$m \times n$

$action(a1, a2, t, score1, score2)$， 其中$a$为agent的id，$t$为时间

## 技术路线3

$state(k+1, m, n)$，其中$k$为agent数量，地图尺寸为$m \times n$

$action(a, t)$， 其中$a$为agent的id，$t$为时间