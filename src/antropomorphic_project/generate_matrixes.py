#!/usr/bin/env python3
import pathlib
import math
import copy
import sympy as sm
from sympy.interactive import printing

# make end matrix display prety
printing.init_printing(use_latex=True)


class AntropomorphicKinematic(object):
    def __init__(self, path):
        self.path = pathlib.Path(path) / "output"
        self.path.mkdir(parents=True, exist_ok=True)

        # dh parameters (placeholder)
        self.ti = sm.Symbol("theta_i")
        self.ai = sm.Symbol("alpha_i")
        self.ri = sm.Symbol("r_i")
        self.di = sm.Symbol("d_i")

        # define generic matrix
        self.matrix = self.generic_matrix()

    def generic_matrix(self):
        # row 1
        m11 = sm.cos(self.ti)
        m12 = -sm.sin(self.ti) * sm.cos(self.ai)
        m13 = sm.sin(self.ti) * sm.sin(self.ai)
        m14 = self.ri * sm.cos(self.ti)
        # row 2
        m21 = sm.sin(self.ti)
        m22 = sm.cos(self.ti) * sm.cos(self.ai)
        m23 = -sm.cos(self.ti) * sm.sin(self.ai)
        m24 = self.ri * sm.sin(self.ti)
        # row 3
        m31 = 0.0
        m32 = sm.sin(self.ai)
        m33 = sm.cos(self.ai)
        m34 = self.di
        # row 4
        m41 = 0.0
        m42 = 0.0
        m43 = 0.0
        m44 = 1.0

        # define matrix using elements
        matrix = sm.Matrix(
            [
                [m11, m12, m13, m14],
                [m21, m22, m23, m24],
                [m31, m32, m33, m34],
                [m41, m42, m43, m44],
            ]
        )
        self.preview(matrix, name="generic")

        # return complete matrix
        return matrix

    def joint_matrix(self, theta, alpha, r, d):
        # create copy of generic matrix
        matrix = copy.deepcopy(self.matrix)

        matrix = matrix.subs(self.ti, theta)
        matrix = matrix.subs(self.ai, alpha)
        matrix = matrix.subs(self.ri, r)
        matrix = matrix.subs(self.di, d)

        # return new matrix
        return matrix

    def chain_substitution(self, matrix, mapper):
        # create copy of matrix
        matrix = copy.deepcopy(matrix)

        # do chain substitution
        for key, value in mapper:
            matrix = matrix.subs(key, value)

        # return new matrix
        return matrix

    def simplify_values(self, matrix):
        # return managed floating point
        condition = lambda v: v.is_Float
        update = lambda v: round(v, 5)
        return sm.nsimplify(matrix.replace(condition, update))

    def simplify_trigonometry(self, matrix):
        # return trigonometry simplified matrix
        return sm.trigsimp(matrix)

    def preview(self, matrix, name):
        # file name and configuration
        filename = self.path / f"{name}.png"
        div = ["-D", "300"]

        # print filepath
        print(f" file : {filename.as_posix()}")

        # save matrix file to disk as file
        sm.preview(matrix, filename=filename, dvioptions=div, viewer="file")


def main(args=None):
    # initialize
    print("antropomorphic_kinematic_matrices")

    # class object and path
    path_root = "src/antropomorphic_project"
    arm_matrix = AntropomorphicKinematic(path_root)

    # dh parameters joint1
    t1 = sm.Symbol("theta_1")
    a1 = math.pi / 2
    r1 = 0.0
    d1 = 0.0

    # A01 : joint1 matrix
    A01 = arm_matrix.joint_matrix(t1, a1, r1, d1)
    A01 = arm_matrix.simplify_values(A01)
    arm_matrix.preview(A01, name="A01")

    # dh parameters joint2
    t2 = sm.Symbol("theta_2")
    a2 = 0.0
    r2 = sm.Symbol("r_2")
    d2 = 0.0

    # A12 : joint2 matrix
    A12 = arm_matrix.joint_matrix(t2, a2, r2, d2)
    A12 = arm_matrix.simplify_values(A12)
    arm_matrix.preview(A12, name="A12")

    # dh parameters joint3
    t3 = sm.Symbol("theta_3")
    a3 = 0.0
    r3 = sm.Symbol("r_3")
    d3 = 0.0

    # A23 : joint3 matrix
    A23 = arm_matrix.joint_matrix(t3, a3, r3, d3)
    A23 = arm_matrix.simplify_values(A23)
    arm_matrix.preview(A23, name="A23")

    # define arm (base to tool) matrix
    A03 = A01 * A12 * A23
    A03 = arm_matrix.simplify_values(A03)
    arm_matrix.preview(A03, name="A03")

    # define arm optimize matrix
    AO3_simplify = arm_matrix.simplify_trigonometry(A03)
    AO3_simplify = arm_matrix.simplify_values(AO3_simplify)
    arm_matrix.preview(AO3_simplify, name="A03_simplify")

    # shutdown
    print("All files generated!")


if __name__ == "__main__":
    main()
