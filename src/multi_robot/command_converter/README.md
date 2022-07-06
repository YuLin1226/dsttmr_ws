# Brief

This package is for converting velocity command from `dual steering wheel kinematics` to `differential drive kinematics`.


## Assumption

- Two diff-drive robots.
- Formation control as a dual-steering-wheel robot.
- Command:
    - Input: $V_x$, $V_y$, $\omega$
    - Output: $v_{x,i}$, $\omega_{i}$, $i=1,2$




## Methods


- 大車速度命令（$V_x$, $V_y$, $W$）轉成小車速度命令（$v$, $\omega$）方法：
    1. 給定大車速度命令（$V_x$, $V_y$, $W$），計算瞬心位置（垂直合速度向量、半徑為合速度、角速度比值）。
    2. 利用雙舵輪運動學模型，計算前後小車速度命令（$v_x$, $v_y$）。
    3. 利用瞬心位置與小車位置可得小車迴轉半徑，搭配第2點的速度命令，可轉換出小車速度命令（$v$, $\omega$）。