import numpy as np


class ForwardKin:

    def __init__(self, d1, d2, d3, d4):
        self.d02 = d1
        self.d24 = d2
        self.d46 = d3
        self.d67 = d4
        self.Ts = []
        self.fk = np.eye(4)

    def dh_table(self, q1, q2, q3, q4, q5, q6, q7):
        return np.array([
            [self.d02, q1, 0, -np.pi / 2],
            [0, q2, 0, np.pi / 2],
            [self.d24, q3, 0, np.pi / 2],
            [0, q4, 0, -np.pi / 2],
            [self.d46, q5, 0, -np.pi / 2],
            [0, q6, 0, np.pi / 2],
            [self.d67, q7, 0, 0],
        ])

    def dh(self, d, theta, a, alpha):
        return np.matrix(
            [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
             [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
             [0, np.sin(alpha), np.cos(alpha), d],
             [0, 0, 0, 1]])

    def forward_kin(self, q1, q2, q3, q4, q5, q6, q7):
        dh_tab = self.dh_table(q1, q2, q3, q4, q5, q6, q7)
        for idx in range(7):
            row = dh_tab[idx]
            T = self.dh(row[0], row[1], row[2], row[3])
            self.Ts.append(T)
            self.fk = self.fk * T
        return self.fk


if __name__ == '__main__':
    fk = ForwardKin(0.103, 0.403, 0.404, 0.257)
    print(fk.forward_kin(0, 0, 0, 0, 0, 0, 0))
    print(fk.Ts)
