#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
双足步态引擎
========================================

基于三次样条轨迹生成 + 6DOF 解析腿部关节求解的开环步态引擎。
适用于 Sigmaban 尺寸的双足机器人模型。

模块结构：
  - Polynom          多项式类，样条的底层基元
  - Spline           通用样条基类
  - CubicSpline      三次样条，支持位置和速度边界条件
    - 腿部几何求解模块 6 自由度腿部解析求解（12 步算法）
    - HumanoidModel    机器人模型封装，提供左右腿角度求解接口
    - 参数结构体       步态参数定义（步频、步幅、抬脚高度等）
  - walk()           步态计算主函数
    - 引擎封装类       面向对象的状态管理与计算入口
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# Polynom — 多项式类
# ---------------------------------------------------------------------------

class Polynom:
    """
    Simple one-dimensional polynomial class for spline generation.

    Stores coefficients indexed from constant to higher degree.

    Coefficients are in a list: [a0, a1, a2, a3, ...]
    representing: a0 + a1*x + a2*x^2 + a3*x^3 + ...
    """

    def __init__(self, degree=None):
        """
        Default constructor creates an empty polynomial.
        If degree is given, creates a polynomial of that degree
        with all coefficients initialized to 0.0.
        """
        if degree is not None:
            self._coefs = [0.0] * (degree + 1)
        else:
            self._coefs = []

    @property
    def coefs(self):
        """Access to coefficient list."""
        return self._coefs

    def degree(self):
        """Return polynomial degree. -1 means empty polynomial."""
        return len(self._coefs) - 1

    def pos(self, x):
        """
        Evaluate the polynomial at x.
        P(x) = sum(coefs[i] * x^i)
        """
        xx = 1.0
        val = 0.0
        for i in range(len(self._coefs)):
            val += xx * self._coefs[i]
            xx *= x
        return val

    def vel(self, x):
        """
        Evaluate the first derivative of the polynomial at x.
        P'(x) = sum(i * coefs[i] * x^(i-1)) for i >= 1
        """
        xx = 1.0
        val = 0.0
        for i in range(1, len(self._coefs)):
            val += i * xx * self._coefs[i]
            xx *= x
        return val

    def acc(self, x):
        """
        Evaluate the second derivative of the polynomial at x.
        P''(x) = sum((i-1)*i * coefs[i] * x^(i-2)) for i >= 2
        """
        xx = 1.0
        val = 0.0
        for i in range(2, len(self._coefs)):
            val += (i - 1) * i * xx * self._coefs[i]
            xx *= x
        return val

    def __imul__(self, coef):
        """Multiply all coefficients by a scalar."""
        for i in range(len(self._coefs)):
            self._coefs[i] *= coef
        return self

    def __iadd__(self, other):
        """Add another polynomial to this one."""
        while len(other._coefs) > len(self._coefs):
            self._coefs.append(0.0)
        for i in range(len(other._coefs)):
            self._coefs[i] += other._coefs[i]
        return self

    def __repr__(self):
        return "Polynom(degree={} coefs={})".format(self.degree(), self._coefs)


# ---------------------------------------------------------------------------
# Spline — 通用样条基类
# ---------------------------------------------------------------------------

class Spline:
    """
    Generic one-dimensional polynomial spline generator.

    Stores a list of SplinePart objects, each containing a Polynom
    valid on an interval [min, max].
    """

    class SplinePart:
        """Internal spline part: a polynomial valid on [min, max]."""

        def __init__(self, polynom=None, min_val=0.0, max_val=0.0):
            self.polynom = polynom if polynom is not None else Polynom()
            self.min = min_val
            self.max = max_val

    def __init__(self):
        self._splines = []

    def pos(self, t):
        """Return spline interpolation value at t."""
        return self._interpolation(t, 'pos')

    def vel(self, t):
        """Return spline first derivative at t."""
        return self._interpolation(t, 'vel')

    def acc(self, t):
        """Return spline second derivative at t."""
        return self._interpolation(t, 'acc')

    def posMod(self, t):
        """Return spline value with t bound between 0 and 1."""
        return self._interpolation_mod(t, 'pos')

    def velMod(self, t):
        """Return spline first derivative with t bound between 0 and 1."""
        return self._interpolation_mod(t, 'vel')

    def accMod(self, t):
        """Return spline second derivative with t bound between 0 and 1."""
        return self._interpolation_mod(t, 'acc')

    def min(self):
        """Return minimum abscissa value for which spline is defined."""
        if len(self._splines) == 0:
            return 0.0
        else:
            return self._splines[0].min

    def max(self):
        """Return maximum abscissa value for which spline is defined."""
        if len(self._splines) == 0:
            return 0.0
        else:
            return self._splines[-1].max

    def _interpolation(self, x, func_name):
        """
        Return spline interpolation of given value using given
        polynom evaluation function name ('pos', 'vel', 'acc').

        Uses bijection (binary) search to find the correct spline segment.
        """
        # Bound asked abscissa into spline range
        if x <= self._splines[0].min:
            x = self._splines[0].min
        if x >= self._splines[-1].max:
            x = self._splines[-1].max

        # 二分查找定位样条区间
        index_low = 0
        index_up = len(self._splines) - 1
        while index_low != index_up:
            index = (index_up + index_low) // 2
            if x < self._splines[index].min:
                index_up = index - 1
            elif x > self._splines[index].max:
                index_low = index + 1
            else:
                index_up = index
                index_low = index

        # Compute and return spline value
        func = getattr(self._splines[index_up].polynom, func_name)
        return func(x - self._splines[index_up].min)

    def _interpolation_mod(self, x, func_name):
        """
        Return interpolation with x bound between 0 and 1.
        """
        if x < 0.0:
            x = 1.0 + (x - (int(x) / 1))
        elif x > 1.0:
            x = (x - (int(x) / 1))
        return self._interpolation(x, func_name)


# ---------------------------------------------------------------------------
# CubicSpline — 三次样条
# ---------------------------------------------------------------------------

class CubicSpline(Spline):
    """
    Implementation of 3rd order polynomial splines.
    """

    class Point:
        """Simple point structure with time, position, and velocity."""

        def __init__(self, time, position, velocity):
            self.time = time
            self.position = position
            self.velocity = velocity

    def __init__(self):
        super().__init__()
        self._points = []

    def addPoint(self, time, position, velocity=0.0):
        """
        Add a new point with its time, position value, and velocity.
        After adding, recomputes all spline segments.
        """
        self._points.append(CubicSpline.Point(time, position, velocity))
        self._computeSplines()

    def _polynomFit(self, t, pos1, vel1, pos2, vel2):
        """
        Fit a cubic polynomial between 0 and t with given
        initial and final position and velocity conditions.
        """
        if t <= 0.00001:
            raise ValueError("CubicSpline invalid spline interval")
        t2 = t * t
        t3 = t2 * t
        p = Polynom()
        p.coefs.clear()
        p.coefs.extend([0.0, 0.0, 0.0, 0.0])
        p.coefs[0] = pos1
        p.coefs[1] = vel1
        p.coefs[3] = (vel2 - vel1 - 2.0 * (pos2 - pos1 - vel1 * t) / t) / t2
        p.coefs[2] = (pos2 - pos1 - vel1 * t - p.coefs[3] * t3) / t2
        return p

    def _computeSplines(self):
        """
        Recompute splines interpolation model.
        """
        self._splines.clear()
        if len(self._points) < 2:
            return

        # Sort points by time
        self._points.sort(key=lambda p: p.time)

        for i in range(1, len(self._points)):
            time = self._points[i].time - self._points[i - 1].time
            if time > 0.00001:
                poly = self._polynomFit(
                    time,
                    self._points[i - 1].position,
                    self._points[i - 1].velocity,
                    self._points[i].position,
                    self._points[i].velocity,
                )
                self._splines.append(
                    Spline.SplinePart(
                        poly,
                        self._points[i - 1].time,
                        self._points[i].time,
                    )
                )


# ---------------------------------------------------------------------------
# 6 自由度腿部解析求解
# ---------------------------------------------------------------------------

class LegIK_Vector3D:
    """
    Simple 3D vector with arithmetic operators.
    """

    def __init__(self, x1=0.0, x2=0.0, x3=0.0):
        self.data = [x1, x2, x3]

    def __getitem__(self, index):
        return self.data[index]

    def __setitem__(self, index, value):
        self.data[index] = value

    def length(self):
        """Return the Euclidean length of the vector."""
        return math.sqrt(
            self.data[0] * self.data[0]
            + self.data[1] * self.data[1]
            + self.data[2] * self.data[2]
        )

    def normalize(self):
        """Normalize this vector in place. Does nothing if length is zero."""
        l = self.length()
        if _ik_is_zero(l):
            return
        inv_l = 1.0 / l
        self.data[0] *= inv_l
        self.data[1] *= inv_l
        self.data[2] *= inv_l

    def __add__(self, other):
        return LegIK_Vector3D(
            self.data[0] + other.data[0],
            self.data[1] + other.data[1],
            self.data[2] + other.data[2],
        )

    def __sub__(self, other):
        return LegIK_Vector3D(
            self.data[0] - other.data[0],
            self.data[1] - other.data[1],
            self.data[2] - other.data[2],
        )

    def __neg__(self):
        return LegIK_Vector3D(-self.data[0], -self.data[1], -self.data[2])

    def __repr__(self):
        return "({:.6f}, {:.6f}, {:.6f})".format(
            self.data[0], self.data[1], self.data[2]
        )


def _ik_scalar_mul(x, v):
    """Scalar * Vector3D multiplication."""
    return LegIK_Vector3D(x * v[0], x * v[1], x * v[2])


def _ik_scalar_prod(v1, v2):
    """Dot product of two Vector3D."""
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def _ik_vect_prod(v1, v2):
    """Cross product of two Vector3D."""
    return LegIK_Vector3D(
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    )


# 腿部求解器的浮点容差常量
_IK_GLOBAL_EPSILON = 0.0000001


def _ik_val_abs(x):
    return x if x >= 0 else -x


def _ik_is_zero(x):
    return _ik_val_abs(x) < _IK_GLOBAL_EPSILON


def _ik_bound(min_val, max_val, x):
    """Bound x between min_val and max_val (modifies x in-place conceptually, returns bounded value)."""
    if x < min_val:
        x = min_val
    if x > max_val:
        x = max_val
    return x


def _ik_sign(x):
    return 1 if x >= 0 else -1


class LegIK_Frame3D:
    """
    3x3 rotation matrix stored as 3 rows of Vector3D.
    """

    def __init__(self):
        """Initialize as canonical (identity) frame."""
        self.rows = [
            LegIK_Vector3D(1, 0, 0),
            LegIK_Vector3D(0, 1, 0),
            LegIK_Vector3D(0, 0, 1),
        ]

    def __getitem__(self, index):
        return self.rows[index]

    def __setitem__(self, index, value):
        self.rows[index] = value

    @staticmethod
    def from_euler(psi, theta, phi):
        """
        Build rotation matrix from Euler angles (psi, theta, phi).

        R[0] = [cos(phi)*cos(psi) - sin(phi)*cos(theta)*sin(psi),
                cos(phi)*sin(psi) + sin(phi)*cos(theta)*cos(psi),
                sin(phi)*sin(theta)]

        R[1] = [-sin(phi)*cos(psi) - cos(phi)*cos(theta)*sin(psi),
                -sin(phi)*sin(psi) + cos(phi)*cos(theta)*cos(psi),
                cos(phi)*sin(theta)]

        R[2] = [sin(theta)*sin(psi),
                -sin(theta)*cos(psi),
                cos(theta)]
        """
        res = LegIK_Frame3D()
        res[0] = LegIK_Vector3D(
            math.cos(phi) * math.cos(psi) - math.sin(phi) * math.cos(theta) * math.sin(psi),
            math.cos(phi) * math.sin(psi) + math.sin(phi) * math.cos(theta) * math.cos(psi),
            math.sin(phi) * math.sin(theta),
        )
        res[1] = LegIK_Vector3D(
            -math.sin(phi) * math.cos(psi) - math.cos(phi) * math.cos(theta) * math.sin(psi),
            -math.sin(phi) * math.sin(psi) + math.cos(phi) * math.cos(theta) * math.cos(psi),
            math.cos(phi) * math.sin(theta),
        )
        res[2] = LegIK_Vector3D(
            math.sin(theta) * math.sin(psi),
            -math.sin(theta) * math.cos(psi),
            math.cos(theta),
        )
        return res


class LegIK_Position:
    """
    Stores 6 joint angles theta[0..5].
    """

    def __init__(self, t0=0.0, t1=0.0, t2=0.0, t3=0.0, t4=0.0, t5=0.0):
        self.theta = [t0, t1, t2, t3, t4, t5]


class LegIK_IK:
    """
    Analytical solver for a 6-DOF leg.

    Constructor takes three segment lengths:
      L0 = distHipToKnee
      L1 = distKneeToAnkle
      L2 = distAnkleToGround
    """

    def __init__(self, L0, L1, L2):
        self.L = [L0, L1, L2]

    def compute(self, C, orientation, result):
        """
        Compute leg joint solution.

        Parameters:
                    C: foot tip position (3D vector)
                    orientation: foot orientation (3x3 rotation matrix)
                    result: output pose object to be filled with 6 joint angles

        Returns True on success, False if position is unreachable.

        This is the full 12-step analytical algorithm for a 6-DOF leg.
        """
        if _ik_is_zero(self.L[0]) or _ik_is_zero(self.L[1]):
            return False

        e1 = LegIK_Vector3D(1, 0, 0)
        e2 = LegIK_Vector3D(0, 1, 0)
        e3 = LegIK_Vector3D(0, 0, 1)

        # Step 1: Compute ankle position B
        B = C + _ik_scalar_mul(self.L[2], orientation[2])
        B_len = B.length()
        if B[2] >= 0 or _ik_is_zero(B_len):
            return False

        # Step 2: Compute phi (hip yaw plane direction)
        phi = LegIK_Vector3D()
        if not _ik_is_zero(orientation[0][2]):
            a_pp = 1.0
            b_pp = -B[2] / orientation[0][2]
            phi = _ik_scalar_mul(a_pp, B) + _ik_scalar_mul(b_pp, orientation[0])
            phi.normalize()
        else:
            phi = orientation[0]
            phi.normalize()

        # Phi is oriented forward or to the left
        phi_e1 = _ik_scalar_prod(phi, e1)
        if not (phi_e1 > 0 or (_ik_is_zero(phi_e1) and _ik_scalar_prod(phi, e2) >= 0)):
            phi = _ik_scalar_mul(-1.0, phi)

        # Step 3: theta[0] = hip_yaw
        result.theta[0] = math.atan2(phi[1], phi[0])

        # Step 4: Compute G (projection of B onto phi)
        G = _ik_scalar_mul(_ik_scalar_prod(B, phi), phi)

        # Step 5: theta[1] = hip_roll
        zeta = _ik_scalar_mul(-1.0, _ik_vect_prod(phi, e3))
        result.theta[1] = math.atan2(_ik_scalar_prod(B - G, zeta), -B[2])

        # Step 6: theta[3] = knee (cosine rule)
        q = (self.L[0] * self.L[0] + self.L[1] * self.L[1] - B_len * B_len) / (
            2 * self.L[0] * self.L[1]
        )
        if q < (-1.0 - _IK_GLOBAL_EPSILON) or q > (1.0 + _IK_GLOBAL_EPSILON):
            return False
        q = _ik_bound(-1.0, 1.0, q)
        result.theta[3] = math.pi - math.acos(q)

        # Step 7: Compute omega (upper leg direction vector)
        omega = LegIK_Vector3D(
            -math.sin(result.theta[0]) * math.sin(result.theta[1]),
            math.cos(result.theta[0]) * math.sin(result.theta[1]),
            -math.cos(result.theta[1]),
        )

        # Step 8: Compute alpha
        q = _ik_scalar_prod(B, omega) / B_len
        q = _ik_bound(-1.0, 1.0, q)
        alpha = _ik_sign(_ik_scalar_prod(_ik_vect_prod(B, omega), zeta)) * math.acos(q)

        # Step 9: Compute A_omega_B (cosine rule)
        q = (self.L[0] * self.L[0] + B_len * B_len - self.L[1] * self.L[1]) / (
            2 * self.L[0] * B_len
        )
        if q < (-1.0 - _IK_GLOBAL_EPSILON) or q > (1.0 + _IK_GLOBAL_EPSILON):
            return False
        q = _ik_bound(-1.0, 1.0, q)
        A_omega_B = math.acos(q)

        # Step 10: theta[2] = hip_pitch
        result.theta[2] = alpha + A_omega_B

        # Step 11: theta[4] = ankle_pitch
        q = _ik_scalar_prod(phi, orientation[0])
        q = _ik_bound(-1.0, 1.0, q)
        beta = -_ik_sign(
            _ik_scalar_prod(_ik_vect_prod(phi, orientation[0]), zeta)
        ) * math.acos(q)
        result.theta[4] = beta + result.theta[3] - result.theta[2]

        # Step 12: theta[5] = ankle_roll
        tau = _ik_vect_prod(phi, omega)
        q = _ik_scalar_prod(tau, orientation[1])
        q = _ik_bound(-1.0, 1.0, q)
        result.theta[5] = _ik_sign(
            _ik_scalar_prod(_ik_vect_prod(tau, orientation[1]), orientation[0])
        ) * math.acos(q)

        return True


# ---------------------------------------------------------------------------
# EulerType — 欧拉角类型枚举
# ---------------------------------------------------------------------------

class EulerType:
    """
    All combinations of Euler angles types in same order as rotation application.
    """
    EulerYawPitchRoll = 0
    EulerYawRollPitch = 1
    EulerRollPitchYaw = 2
    EulerRollYawPitch = 3
    EulerPitchRollYaw = 4
    EulerPitchYawRoll = 5


# ---------------------------------------------------------------------------
# 步态输出（12 个关节角）
# ---------------------------------------------------------------------------

class IKWalkOutputs:
    """
    Leg motor result positions (in radians).
    """

    def __init__(self):
        self.left_hip_yaw = 0.0
        self.left_hip_roll = 0.0
        self.left_hip_pitch = 0.0
        self.left_knee = 0.0
        self.left_ankle_pitch = 0.0
        self.left_ankle_roll = 0.0
        self.right_hip_yaw = 0.0
        self.right_hip_roll = 0.0
        self.right_hip_pitch = 0.0
        self.right_knee = 0.0
        self.right_ankle_pitch = 0.0
        self.right_ankle_roll = 0.0


# ---------------------------------------------------------------------------
# HumanoidModel — 机器人模型与腿部求解接口
# ---------------------------------------------------------------------------

class HumanoidModel:
    """
    Humanoid model with analytical leg-joint solving.
    Uses numpy for matrix operations.
    """

    def __init__(self, distHipToKnee, distKneeToAnkle, distAnkleToGround, distFeetLateral):
        self._legHipToKnee = distHipToKnee
        self._legKneeToAnkle = distKneeToAnkle
        self._legAnkleToGround = distAnkleToGround
        self._legLateral = distFeetLateral

    def legsLength(self):
        """Return the initial vertical distance from trunk frame to foot tip frame (Z)."""
        return self._legHipToKnee + self._legKneeToAnkle + self._legAnkleToGround

    def feetDistance(self):
        """Return the initial lateral distance between each feet."""
        return self._legLateral

    def legIkLeft(self, footPos, angles, eulerType, outputs):
        """
        Run analytical leg solving on left leg and assign outputs.

        Parameters:
          footPos: numpy array [x, y, z] - target foot position
          angles: numpy array [pitch, roll, yaw] - target foot orientation angles
          eulerType: EulerType enum value
          outputs: output container to be updated

        Returns True on success.
        """
        ik = LegIK_IK(self._legHipToKnee, self._legKneeToAnkle, self._legAnkleToGround)
        legIKTarget = self._buildTargetPos(footPos)
        legIKMatrix = self._buildTargetOrientation(angles, eulerType)

        result = LegIK_Position()
        isSucess = ik.compute(legIKTarget, legIKMatrix, result)

        if isSucess:
            self._checkNaN(result, legIKTarget, legIKMatrix)
            self._setIKResult(result, True, outputs)

        return isSucess

    def legIkRight(self, footPos, angles, eulerType, outputs):
        """
        Run analytical leg solving on right leg and assign outputs.

        Parameters are same as legIkLeft.
        """
        ik = LegIK_IK(self._legHipToKnee, self._legKneeToAnkle, self._legAnkleToGround)
        legIKTarget = self._buildTargetPos(footPos)
        legIKMatrix = self._buildTargetOrientation(angles, eulerType)

        result = LegIK_Position()
        isSucess = ik.compute(legIKTarget, legIKMatrix, result)

        if isSucess:
            self._checkNaN(result, legIKTarget, legIKMatrix)
            self._setIKResult(result, False, outputs)

        return isSucess

    def _eulersToMatrix(self, angles, eulerType):
        """
        Convert given euler angle of given convention type to rotation matrix.
        """
        if eulerType == EulerType.EulerYawPitchRoll:
            yawRot = _angle_axis_matrix(angles[0], 2)   # Z axis
            pitchRot = _angle_axis_matrix(angles[1], 1)  # Y axis
            rollRot = _angle_axis_matrix(angles[2], 0)   # X axis
            rotMatrix = rollRot @ pitchRot @ yawRot
        elif eulerType == EulerType.EulerYawRollPitch:
            yawRot = _angle_axis_matrix(angles[0], 2)   # Z axis
            pitchRot = _angle_axis_matrix(angles[2], 1)  # Y axis
            rollRot = _angle_axis_matrix(angles[1], 0)   # X axis
            rotMatrix = pitchRot @ rollRot @ yawRot
        elif eulerType == EulerType.EulerRollPitchYaw:
            yawRot = _angle_axis_matrix(angles[2], 2)   # Z axis
            pitchRot = _angle_axis_matrix(angles[1], 1)  # Y axis
            rollRot = _angle_axis_matrix(angles[0], 0)   # X axis
            rotMatrix = yawRot @ pitchRot @ rollRot
        elif eulerType == EulerType.EulerRollYawPitch:
            yawRot = _angle_axis_matrix(angles[1], 2)   # Z axis
            pitchRot = _angle_axis_matrix(angles[2], 1)  # Y axis
            rollRot = _angle_axis_matrix(angles[0], 0)   # X axis
            rotMatrix = pitchRot @ yawRot @ rollRot
        elif eulerType == EulerType.EulerPitchRollYaw:
            yawRot = _angle_axis_matrix(angles[2], 2)   # Z axis
            pitchRot = _angle_axis_matrix(angles[0], 1)  # Y axis
            rollRot = _angle_axis_matrix(angles[1], 0)   # X axis
            rotMatrix = yawRot @ rollRot @ pitchRot
        elif eulerType == EulerType.EulerPitchYawRoll:
            yawRot = _angle_axis_matrix(angles[1], 2)   # Z axis
            pitchRot = _angle_axis_matrix(angles[0], 1)  # Y axis
            rollRot = _angle_axis_matrix(angles[2], 0)   # X axis
            rotMatrix = rollRot @ yawRot @ pitchRot
        else:
            rotMatrix = np.eye(3)
        return rotMatrix

    def _buildTargetPos(self, footPos):
        """
        Compute and return the position reference vector in internal structure.
        """
        target = footPos.copy()
        target[2] -= self.legsLength()

        legIKTarget = LegIK_Vector3D(target[0], target[1], target[2])
        return legIKTarget

    def _buildTargetOrientation(self, angles, eulerType):
        """
        Compute and return the orientation reference matrix in internal structure.
        """
        rotMatrixFrame = self._eulersToMatrix(angles, eulerType)
        rotMatrixTarget = rotMatrixFrame

        legIKMatrix = LegIK_Frame3D()
        legIKMatrix[0][0] = rotMatrixTarget[0, 0]
        legIKMatrix[0][1] = rotMatrixTarget[0, 1]
        legIKMatrix[0][2] = rotMatrixTarget[0, 2]
        legIKMatrix[1][0] = rotMatrixTarget[1, 0]
        legIKMatrix[1][1] = rotMatrixTarget[1, 1]
        legIKMatrix[1][2] = rotMatrixTarget[1, 2]
        legIKMatrix[2][0] = rotMatrixTarget[2, 0]
        legIKMatrix[2][1] = rotMatrixTarget[2, 1]
        legIKMatrix[2][2] = rotMatrixTarget[2, 2]
        return legIKMatrix

    def _setIKResult(self, result, isLeftLeg, outputs):
        """
        Assign model leg DOF to given solving results.

        Note the sign convention:
          hip_pitch and ankle_pitch are NEGATED.
        """
        if isLeftLeg:
            outputs.left_hip_yaw = result.theta[0]
            outputs.left_hip_roll = result.theta[1]
            outputs.left_hip_pitch = -result.theta[2]
            outputs.left_knee = result.theta[3]
            outputs.left_ankle_pitch = -result.theta[4]
            outputs.left_ankle_roll = result.theta[5]
        else:
            outputs.right_hip_yaw = result.theta[0]
            outputs.right_hip_roll = result.theta[1]
            outputs.right_hip_pitch = -result.theta[2]
            outputs.right_knee = result.theta[3]
            outputs.right_ankle_pitch = -result.theta[4]
            outputs.right_ankle_roll = result.theta[5]

    def _checkNaN(self, result, pos, orientation):
        """
        Check computed joint values and throw an error in case of NaN.
        """
        if (
            math.isnan(result.theta[0])
            or math.isnan(result.theta[1])
            or math.isnan(result.theta[2])
            or math.isnan(result.theta[3])
            or math.isnan(result.theta[4])
            or math.isnan(result.theta[5])
        ):
            raise ValueError(
                "LegIK NaN invalid result. "
                "theta0={} theta1={} theta2={} theta3={} theta4={} theta5={} "
                "pos={} orientation=({},{},{})".format(
                    result.theta[0],
                    result.theta[1],
                    result.theta[2],
                    result.theta[3],
                    result.theta[4],
                    result.theta[5],
                    pos,
                    orientation[0],
                    orientation[1],
                    orientation[2],
                )
            )


def _angle_axis_matrix(angle, axis):
    """
    Build a 3x3 rotation matrix from an angle and axis index.

    axis: 0 = X, 1 = Y, 2 = Z
    """
    c = math.cos(angle)
    s = math.sin(angle)
    if axis == 0:  # X axis (roll)
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ])
    elif axis == 1:  # Y axis (pitch)
        return np.array([
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ])
    else:  # Z axis (yaw)
        return np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])


# ---------------------------------------------------------------------------
# 步态参数定义
# ---------------------------------------------------------------------------

class IKWalkParameters:
    """
    Walk parameters structure.
    Default values are tuned for the Sigmaban robot model.
    """

    def __init__(self):
        # Model leg typical length between each rotation axis
        self.distHipToKnee = 0.093
        self.distKneeToAnkle = 0.105
        self.distAnkleToGround = 0.032

        # Distance between the two feet in lateral axis while in zero position
        self.distFeetLateral = 0.092

        # Complete (two legs) walk cycle frequency in Hertz
        self.freq = 1.7

        # Global gain multiplying all time dependant movement between 0 and 1.
        # Control walk enabled/disabled smoothing.
        # 0 is walk disabled. 1 is walk fully enabled.
        self.enabledGain = 1.0

        # Length of double support phase in phase time (between 0 and 1).
        # 0 is null double support and full single support.
        # 1 is full double support and null single support.
        self.supportPhaseRatio = 0.0

        # Lateral offset on default foot position in meters (foot lateral distance).
        # 0 is default. > 0 is both feet external offset.
        self.footYOffset = 0.025

        # Forward length of each foot step in meters.
        # >0 goes forward, <0 goes backward. (dynamic parameter)
        self.stepGain = 0.0

        # Vertical rise height of each foot in meters (positive)
        self.riseGain = 0.035

        # Angular yaw rotation of each foot for each step in radian.
        # 0 does not turn. >0 turns left. <0 turns right. (dynamic parameter)
        self.turnGain = 0.0

        # Lateral length of each foot step in meters.
        # >0 goes left. <0 goes right. (dynamic parameter)
        self.lateralGain = 0.0

        # Vertical foot offset from trunk in meters (positive).
        # 0 is in init position. > 0 set the robot lower to the ground.
        self.trunkZOffset = 0.02

        # Lateral trunk oscillation amplitude in meters (positive)
        self.swingGain = 0.02

        # Lateral angular oscillation amplitude of swing trunkRoll in radian
        self.swingRollGain = 0.0

        # Phase shift of lateral trunk oscillation between 0 and 1
        self.swingPhase = 0.25

        # Foot X-Z spline velocities at ground take off and ground landing.
        # Typical values ranges within 0 and 5.
        self.stepUpVel = 4.0
        self.stepDownVel = 4.0
        self.riseUpVel = 4.0
        self.riseDownVel = 4.0

        # Time length in phase time where swing lateral oscillation
        # remains on the same side. Between 0 and 0.5.
        self.swingPause = 0.0

        # Swing lateral spline velocity (positive).
        # Control the "smoothness" of swing trajectory.
        # Typical values are between 0 and 5.
        self.swingVel = 4.0

        # Forward trunk-foot offset with respect to foot in meters.
        # >0 moves the trunk forward. <0 moves the trunk backward.
        # 仿真中减小前倾偏移，避免 CoG 过度前移导致前倒。
        self.trunkXOffset = 0.0

        # Lateral trunk-foot offset with respect to foot in meters.
        # >0 moves the trunk on the left. <0 moves the trunk on the right.
        self.trunkYOffset = 0.0

        # Trunk angular rotation around Y in radian.
        # >0 bends the trunk forward. <0 bends the trunk backward.
        # 真实 Sigmaban 用 0.15（配合实际质量分布），仿真中大幅减小避免前倾摔倒。
        self.trunkPitch = 0.05

        # Trunk angular rotation around X in radian.
        # >0 bends the trunk on the right. <0 bends the trunk on the left.
        self.trunkRoll = 0.0

        # Add extra offset on X, Y and Z direction on left and right feet in meters
        self.extraLeftX = 0.0
        self.extraLeftY = 0.0
        self.extraLeftZ = 0.0
        self.extraRightX = 0.0
        self.extraRightY = 0.0
        self.extraRightZ = 0.0

        # Add extra angular offset on Yaw, Pitch and Roll rotation
        # of left and right foot in radians
        self.extraLeftYaw = 0.0
        self.extraLeftPitch = 0.0
        self.extraLeftRoll = 0.0
        self.extraRightYaw = 0.0
        self.extraRightPitch = 0.0
        self.extraRightRoll = 0.0


# ---------------------------------------------------------------------------
# boundPhase — 相位归一化
# ---------------------------------------------------------------------------

def _boundPhase(phase):
    """
    Cycle given phase between 0 and 1.
    Modifies phase in-place.
    """
    while phase >= 1.0:
        phase -= 1.0
        # Bound to zero in case of floating point error
        if phase < 0.0:
            phase = 0.0
    return phase


# ---------------------------------------------------------------------------
# walk() — 步态计算主函数
# ---------------------------------------------------------------------------

def walk(params, dt, phase, outputs):
    """
    Compute and return target motor reference positions using given walk
    parameters at given phase (between 0 and 1).

    Phase is updated according to frequency parameter and given time step dt.
    If leg solving fails, False is returned and neither phase nor
    output is updated.

    Parameters:
    params: walk parameters instance
      dt: time step in seconds
      phase: current walk phase (float, will be updated in-place)
    outputs: output container (will be updated with joint angles)

    Returns:
    True if solving succeeded and outputs/phase were updated.
    False if solving failed (outputs/phase NOT updated).
    """
    # Init Humanoid Model
    model = HumanoidModel(
        params.distHipToKnee,
        params.distKneeToAnkle,
        params.distAnkleToGround,
        params.distFeetLateral,
    )

    # Compute phase for left and right leg
    phaseLeft = phase
    phaseRight = phase + 0.5
    phaseLeft = _boundPhase(phaseLeft)
    phaseRight = _boundPhase(phaseRight)

    # Compute the length of a step (from ground touch to take off) in phase time
    stepLength = 0.5 * params.supportPhaseRatio + 0.5

    # Build X foot step spline.
    # The foot goes backward between t=0 and t=stepLength and then goes forward.
    # Custom velocity (tangents) are applied at foot take off and landing.
    # During foot backward movement, constant velocity is applied because both
    # foot must have the same velocity during double support phase.
    stepSpline = CubicSpline()
    stepSpline.addPoint(0.0, 0.5, -1.0 / stepLength)
    stepSpline.addPoint(stepLength, -0.5, -1.0 / stepLength)
    stepSpline.addPoint(stepLength, -0.5, params.stepUpVel)
    stepSpline.addPoint(1.0, 0.5, -params.stepDownVel)

    # Build Y trunk swing spline.
    # The trunk lateral oscillation goes from right to left,
    # waits a bit (swingPause) on left side then goes to the right and pauses as well.
    # Trajectory "smoothness" can be tuned with swingVel updating splines tangents.
    swingSpline = CubicSpline()
    swingSpline.addPoint(0.0, -1.0)
    swingSpline.addPoint(params.swingPause / 2.0, -1.0)
    swingSpline.addPoint(params.swingPause / 2.0, -1.0, params.swingVel)
    swingSpline.addPoint(0.5 - params.swingPause / 2.0, 1.0, params.swingVel)
    swingSpline.addPoint(0.5 - params.swingPause / 2.0, 1.0)
    swingSpline.addPoint(0.5 + params.swingPause / 2.0, 1.0)
    swingSpline.addPoint(0.5 + params.swingPause / 2.0, 1.0, -params.swingVel)
    swingSpline.addPoint(1.0 - params.swingPause / 2.0, -1.0, -params.swingVel)
    swingSpline.addPoint(1.0 - params.swingPause / 2.0, -1.0)
    swingSpline.addPoint(1.0, -1.0, 0.0)

    # Build Z foot rise spline.
    # The foot stays on the ground during backward step and then moves up and down.
    # Custom velocities (tangents) can be tuned to achieve specific trajectory
    # at foot take off and landing.
    riseSpline = CubicSpline()
    riseSpline.addPoint(0.0, 0.0)
    riseSpline.addPoint(stepLength, 0.0)
    riseSpline.addPoint(stepLength, 0.0, params.riseUpVel)
    riseSpline.addPoint((1.0 + stepLength) / 2.0, 1.0)
    riseSpline.addPoint(1.0, 0.0, -params.riseDownVel)

    # Build Yaw foot turn spline.
    # This is the same as stepSpline but movement occurs only during single
    # support phase as robot degrees of freedom could not achieve rotation
    # during double support phase.
    turnSpline = CubicSpline()
    turnSpline.addPoint(0.0, 0.0)
    turnSpline.addPoint(stepLength - 0.5, 0.0)
    turnSpline.addPoint(0.5, 1.0)
    turnSpline.addPoint(stepLength, 1.0)
    turnSpline.addPoint(1.0, 0.0)

    # Compute swing value
    swingVal = (
        params.enabledGain
        * params.swingGain
        * swingSpline.posMod(0.5 + phaseLeft + params.swingPhase)
    )

    # Compute feet forward (step) oscillation
    leftX = params.enabledGain * params.stepGain * stepSpline.pos(phaseLeft)
    rightX = params.enabledGain * params.stepGain * stepSpline.pos(phaseRight)

    # Compute feet swing oscillation
    leftY = swingVal
    rightY = swingVal

    # Compute feet lateral movement oscillation
    leftY += params.enabledGain * params.lateralGain * (
        stepSpline.pos(phaseLeft) + 0.5 * (1.0 if params.lateralGain >= 0.0 else -1.0)
    )
    rightY += params.enabledGain * params.lateralGain * (
        stepSpline.pos(phaseRight) + 0.5 * (-1.0 if params.lateralGain >= 0.0 else 1.0)
    )

    # Set feet lateral offset (feet distance from trunk center)
    leftY += params.footYOffset
    rightY += -params.footYOffset

    # Compute feet vertical (rise) oscillation and offset
    leftZ = params.enabledGain * params.riseGain * riseSpline.pos(phaseLeft)
    rightZ = params.enabledGain * params.riseGain * riseSpline.pos(phaseRight)

    # Set trunk to foot distance height offset
    leftZ += params.trunkZOffset
    rightZ += params.trunkZOffset

    # Compute feet rotation (turn) oscillation
    leftYaw = params.enabledGain * params.turnGain * turnSpline.pos(phaseLeft)
    rightYaw = params.enabledGain * params.turnGain * turnSpline.pos(phaseRight)

    # Compute trunk roll angle
    rollVal = (
        params.enabledGain
        * -params.swingRollGain
        * swingSpline.posMod(0.5 + phaseLeft + params.swingPhase)
    )

    # Set trunk roll offset
    rollVal += params.trunkRoll

    # Set feet orientation
    leftPitch = params.trunkPitch
    leftRoll = rollVal
    rightPitch = params.trunkPitch
    rightRoll = rollVal

    # Add custom extra foot offset on both feet
    leftX += params.extraLeftX
    leftY += params.extraLeftY
    leftZ += params.extraLeftZ
    leftYaw += params.extraLeftYaw
    leftPitch += params.extraLeftPitch
    leftRoll += params.extraLeftRoll
    rightX += params.extraRightX
    rightY += params.extraRightY
    rightZ += params.extraRightZ
    rightYaw += params.extraRightYaw
    rightPitch += params.extraRightPitch
    rightRoll += params.extraRightRoll

    # Build rotation matrix for trunk pitch and roll orientation.
    pitchRot = _angle_axis_matrix(-params.trunkPitch, 1)  # Y axis
    rollRot = _angle_axis_matrix(-rollVal, 0)             # X axis
    rotation = pitchRot @ rollRot

    # Build target vector.
    # Used Euler angles orders is Pitch Roll Yaw because
    # Yaw has to be applied last, after the foot get the good
    # ground orientation. Roll has to be applied after Pitch.
    posLeft = np.array([leftX, leftY, leftZ])
    angleLeft = np.array([leftPitch, leftRoll, leftYaw])
    posRight = np.array([rightX, rightY, rightZ])
    angleRight = np.array([rightPitch, rightRoll, rightYaw])

    # Rotate built feet trajectory to meet asked trunk Pitch and Roll
    # new ground orientation
    posLeft = rotation @ posLeft
    posRight = rotation @ posRight

    # Apply trunk X-Y offset
    posLeft[0] -= params.trunkXOffset
    posRight[0] -= params.trunkXOffset
    posLeft[1] -= params.trunkYOffset
    posRight[1] -= params.trunkYOffset

    # In case of trunk Roll rotation, an height (Z) positive offset have to be
    # applied on external foot to set both feet on same level
    deltaLen = model.feetDistance() * math.tan(rollVal)
    if rollVal > 0.0:
        posRight[2] += deltaLen
    elif rollVal < 0.0:
        posLeft[2] -= deltaLen

    # Trunk X and Y offset is applied to compensate Pitch and Roll rotation.
    # It is better for tuning if trunk pitch or roll rotation do not apply
    # offset on trunk position.
    posLeft[0] += model.legsLength() * math.tan(params.trunkPitch)
    posRight[0] += model.legsLength() * math.tan(params.trunkPitch)
    posLeft[1] -= model.legsLength() * math.tan(rollVal)
    posRight[1] -= model.legsLength() * math.tan(rollVal)

    # Run leg solving on both legs using Pitch-Roll-Yaw convention
    successLeft = model.legIkLeft(
        posLeft, angleLeft, EulerType.EulerPitchRollYaw, outputs
    )
    successRight = model.legIkRight(
        posRight, angleRight, EulerType.EulerPitchRollYaw, outputs
    )

    # Check solving success
    if not successLeft or not successRight:
        return False

    # Increment given phase
    phase += dt * params.freq

    # Cycling between 0 and 1
    phase = _boundPhase(phase)

    return phase


# ---------------------------------------------------------------------------
# 引擎封装类（convenience wrapper class）
# ---------------------------------------------------------------------------

class IKWalkEngine:
    """
    Convenience wrapper class that encapsulates the walk() function
    with internal phase state management.
    """

    def __init__(self):
        self.phase = 0.0

    def compute(self, dt, params):
        """
        Compute one time step of the walk engine.

        Parameters:
          dt: time step in seconds
          params: walk parameters instance

        Returns:
                    output container on success, or None if solving failed.
          On failure, phase is NOT updated.
        """
        outputs = IKWalkOutputs()
        new_phase = walk(params, dt, self.phase, outputs)
        if new_phase is False:
            return None
        self.phase = new_phase
        return outputs

    def get_zero_pose(self):
        """Return zero pose (standing). All angles are 0.0."""
        outputs = IKWalkOutputs()
        return outputs


# ---------------------------------------------------------------------------
# angles_to_ros_command  (utility function)
# ---------------------------------------------------------------------------

def angles_to_ros_command(outputs, arm_angles=None):
    """
    Map output joint angles to the 16-joint Float64MultiArray command.

    Command array order (matching controllers_walk.yaml):
      [0]  left_shoulder_pitch
      [1]  right_shoulder_pitch
      [2]  left_elbow
      [3]  right_elbow
      [4]  left_hip_yaw
      [5]  right_hip_yaw
      [6]  left_hip_roll
      [7]  right_hip_roll
      [8]  left_hip_pitch
      [9]  right_hip_pitch
      [10] left_knee
      [11] right_knee
      [12] left_ankle_pitch
      [13] right_ankle_pitch
      [14] left_ankle_roll
      [15] right_ankle_roll

    Parameters:
            outputs: output container instance
      arm_angles: [left_shoulder, right_shoulder, left_elbow, right_elbow]
                  defaults to [0, 0, 0, 0]
    """
    if arm_angles is None:
        arm_angles = [0.0, 0.0, 0.0, 0.0]

    cmd = [
        arm_angles[0],                # [0]  left_shoulder_pitch
        arm_angles[1],                # [1]  right_shoulder_pitch
        arm_angles[2],                # [2]  left_elbow
        arm_angles[3],                # [3]  right_elbow
        outputs.left_hip_yaw,         # [4]  left_hip_yaw
        outputs.right_hip_yaw,        # [5]  right_hip_yaw
        outputs.left_hip_roll,        # [6]  left_hip_roll
        outputs.right_hip_roll,       # [7]  right_hip_roll
        outputs.left_hip_pitch,       # [8]  left_hip_pitch
        outputs.right_hip_pitch,      # [9]  right_hip_pitch
        outputs.left_knee,            # [10] left_knee
        outputs.right_knee,           # [11] right_knee
        outputs.left_ankle_pitch,     # [12] left_ankle_pitch
        outputs.right_ankle_pitch,    # [13] right_ankle_pitch
        outputs.left_ankle_roll,      # [14] left_ankle_roll
        outputs.right_ankle_roll,     # [15] right_ankle_roll
    ]
    return cmd


# ---------------------------------------------------------------------------
# Main / test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=== IKWalk Engine - Faithful Python Port ===")
    print("Testing walk engine with Sigmaban default parameters...\n")

    params = IKWalkParameters()

    # Walk on place (no step, no turn, no lateral)
    params.enabledGain = 1.0
    params.stepGain = 0.0
    params.lateralGain = 0.0
    params.turnGain = 0.0

    engine = IKWalkEngine()
    engine_frequency = 50.0
    dt = 1.0 / engine_frequency

    print("Running 2 seconds of walking on place...")
    for i in range(int(2.0 * engine_frequency)):
        result = engine.compute(dt, params)
        if result is not None:
            t = (i + 1) * dt
            print(
                "t={:.3f} phase={:.4f} | L: yaw={:.4f} pitch={:.4f} roll={:.4f} "
                "knee={:.4f} a_pitch={:.4f} a_roll={:.4f} | R: yaw={:.4f} "
                "pitch={:.4f} roll={:.4f} knee={:.4f} a_pitch={:.4f} a_roll={:.4f}".format(
                    t,
                    engine.phase,
                    result.left_hip_yaw,
                    result.left_hip_pitch,
                    result.left_hip_roll,
                    result.left_knee,
                    result.left_ankle_pitch,
                    result.left_ankle_roll,
                    result.right_hip_yaw,
                    result.right_hip_pitch,
                    result.right_hip_roll,
                    result.right_knee,
                    result.right_ankle_pitch,
                    result.right_ankle_roll,
                )
            )
        else:
            print("  IK failed at step {}".format(i))

    print("\n=== Test complete ===")
