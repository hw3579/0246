import numpy as np

def solve_quintic_trajectory(t_s, t_f, q_s, v_s, a_s, q_f, v_f, a_f):
    # 构造系数矩阵
    A = np.array([
        [1, t_s, t_s**2, t_s**3, t_s**4, t_s**5],
        [0, 1, 2*t_s, 3*t_s**2, 4*t_s**3, 5*t_s**4],
        [0, 0, 2, 6*t_s, 12*t_s**2, 20*t_s**3],
        [1, t_f, t_f**2, t_f**3, t_f**4, t_f**5],
        [0, 1, 2*t_f, 3*t_f**2, 4*t_f**3, 5*t_f**4],
        [0, 0, 2, 6*t_f, 12*t_f**2, 20*t_f**3]
    ])
    
    # 构造右侧边界条件向量
    b = np.array([q_s, v_s, a_s, q_f, v_f, a_f])
    
    # 解方程组
    coefficients = np.linalg.solve(A, b)
    return coefficients

# 示例参数
t_s, t_f = 0, 1  # 起始和终止时间
q_s, v_s, a_s = 0, 0, 0  # 初始位置、速度和加速度
q_f, v_f, a_f = 1, 0, 0  # 目标位置、速度和加速度

coefficients = solve_quintic_trajectory(t_s, t_f, q_s, v_s, a_s, q_f, v_f, a_f)
print("五阶多项式系数：", coefficients)
