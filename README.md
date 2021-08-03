# 在CBS系列算法中使用RL算法

技术路线：
1. 对单个节点，通过选择冲突加速CBS算法，以RL技术达到ICBS效果
2. 对每个节点进行打分，在HL选择节点扩展时进行排序，达到CBSH、CBSH2效果
3. 在HL扩展时进行剪枝，不在所有叶节点中选择，根据RL学习结果选择一个子节点，将算法时间复杂度降低到多项式级别

通过RL算法，无法保证解的最优性，尽量保持解的质量。

## 技术路线1

$state(k+1, m)$，其中$k$为agent数量，地图尺寸为$m = width \times height$

$action(a1, a2, t)$， 其中$a$为agent的id，$t$为时间

## 技术路线2

$state(k+1, m)$，其中$k$为agent数量，地图尺寸为$m = width \times height$

$action(a1, a2, t, score1, score2)$， 其中$a$为agent的id，$t$为时间

## 技术路线3

$state(k+1, m)$，其中$k$为agent数量，地图尺寸为$m = width \times height$

$action(a, t)$， 其中$a$为agent的id，$t$为时间，$t \not ={0}$

action处理方式：
1. reward加入冲突数评价，避免探索混乱，冲突越来越多。进行归一化，基数为初始时g cost。
2. 得到解的时候用g cost做为reward，为主要优化目标。进行归一化，基数为初始时g cost。
3. 如果action不是冲突，通过reward惩罚，保留探索可能，但是不推荐探索，因为这种方式有可能找到最优解，但是会加大训练难度

第3项不好设置，先不设置reward

station问题：
1. 对某个agent，在地图上对应位置标注时间点，来表示路径。如在同一位置长时间停留，只标志最后停留时刻。（该方法旨在降低信息维度，是否可行待验证）