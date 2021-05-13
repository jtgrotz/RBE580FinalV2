import numpy as np


class GravityComp:
    # Gravity value (m/s^2)
    G = 9.81

    def __init__(self, fk, m1, m2, m3, m4, m5, m6, m7):
        self.fk = fk
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.m4 = m4
        self.m5 = m5
        self.m6 = m6
        self.m7 = m7

    def gravity_compensation(self, q1, q2, q3, q4, q5, q6, q7):
        # From file GravityCompensation, open and run as live script in matlab
        sigma1 = self.G * self.fk.d67 * np.sin(q2) * np.sin(q4) * np.sin(q6)
        sigma2 = self.G * self.fk.d67 * np.sin(q2) * np.cos(q4) * np.cos(q6)
        sigma3 = self.G * self.fk.d67 * np.cos(q2) * np.sin(q4) * np.cos(q6)
        sigma4 = self.G * self.fk.d67 * np.cos(q2) * np.cos(q4) * np.sin(q6)
        sigma5 = self.G * self.fk.d46 * np.sin(q2) * np.cos(q4)
        sigma6 = self.G * self.fk.d46 * np.cos(q2) * np.sin(q4)
        sigma7 = self.G * self.fk.d24 * np.sin(q2)
        return np.matrix(
            [[0],
             [-sigma7 * self.m3 - sigma7 * self.m4 + (sigma6 - sigma7 - sigma5) * self.m5 + (
                         sigma6 - sigma7 - sigma5) * self.m6 +
              (sigma6 - sigma7 - sigma5 - sigma4 + sigma3 - sigma2 - sigma1) * self.m7],
             [0],
             [(sigma5 - sigma6) * self.m5 + (sigma5 - sigma6) * self.m6 + (
                         sigma5 - sigma6 + sigma4 - sigma3 + sigma2 + sigma1) * self.m7],
             [0],
             [(sigma3 - sigma4 - sigma2 - sigma1) * self.m7],
             [0]])


if __name__ == '__main__':
    from forward_kin import ForwardKin

    fk = ForwardKin(0.103, 0.403, 0.404, 0.257)
    gc = GravityComp(fk, .5, .5, .5, .5, .5, .5, .5)
    gc_vec = gc.gravity_compensation(0, np.pi / 2, 0, 0, 0, 0, 0)
    print(gc_vec)
    gc_vec = gc.gravity_compensation(0, 0, 0, 0, 0, 0, 0)
    print(gc_vec)
