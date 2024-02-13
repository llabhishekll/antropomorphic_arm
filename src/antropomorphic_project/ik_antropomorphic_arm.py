#!/usr/bin/env python3
import sys
import math

# constant value
PI = math.pi


class AntropomorphicInverseKinematic(object):
    def __init__(self, r1, r2, r3):
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3

    def inverse_kinematic(self, px, py, pz, sign1, sign2):
        # dh parameters
        r1 = self.r1
        r2 = self.r2
        r3 = self.r3

        # theta1 (top view)
        theta1_sol1 = math.atan2(py, px)
        theta1_sol2 = math.atan2(math.sin(theta1_sol1 + PI), math.cos(theta1_sol1 + PI))
        theta1 = theta1_sol1 if sign1 else theta1_sol2

        # frame change
        x, y = px / math.cos(theta1), pz

        # theta3 (side view)
        theta3_sol1 = math.acos((x * x + y * y - (r2 * r2 + r3 * r3)) / (2 * r2 * r3))
        theta3_sol2 = math.atan2(math.sin(-theta3_sol1), math.cos(theta3_sol1))
        theta3 = theta3_sol1 if sign2 else theta3_sol2

        # theta2 (side view)
        gamma = math.atan2(y, x)
        beta = math.atan2(r3 * math.sin(theta3), r2 + (r3 * math.cos(theta3)))
        theta2 = math.atan2(math.sin(gamma - beta), math.cos(gamma - beta))

        # return calculate angles
        return theta1, theta2, theta3

    def calculate_solution(self, px, py, pz, sign1, sign2):
        # check solvability
        if (self.r2 <= 0) or (self.r3 <= 0):
            raise ValueError("Arm length cannot be zero or less!")
        else:
            is_possible = True

        # calculate theta1, theta2 and theta3
        theta1, theta2, theta3 = self.inverse_kinematic(px, py, pz, sign1, sign2)

        # check reachability
        if not (-PI / 4 <= theta2 <= 3 * PI / 4):     # joint2 limit
            is_possible = False
        if not (-3 * PI / 4 <= theta3 <= 3 * PI / 4): # joint3 limit
            is_possible = False

        # log info
        print(f" {is_possible} : [{theta1:.5f} {theta2:.5f} {theta3:.5f}]")

        # return final solution
        return is_possible, theta1, theta2, theta3


def main(args=None):
    # initialize
    print("inverse_kinematic_solution")

    # check if arguments
    if len(sys.argv) < 4:
        px = 0.5
        py = 0.6
        pz = 0.7
    else:
        px = float(sys.argv[1])
        py = float(sys.argv[2])
        pz = float(sys.argv[3])

    # class object
    model = AntropomorphicInverseKinematic(0.0, 1.0, 1.0)

    # find solutions
    model.calculate_solution(px, py, pz, True, True)
    model.calculate_solution(px, py, pz, True, False)
    model.calculate_solution(px, py, pz, False, True)
    model.calculate_solution(px, py, pz, False, False)

    # shutdown
    print("All calculation generated!")


if __name__ == "__main__":
    main()
