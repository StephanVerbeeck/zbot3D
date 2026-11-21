package main

import (
	"math"

	"github.com/g3n/engine/math32"
)

// Quat represents a unit quaternion (W + Xi + Yj + Zk) used for 3D rotations.
type Quat struct {
	W float32
	X float32
	Y float32
	Z float32
}

// IdentityQuat returns the identity rotation quaternion.
func IdentityQuat() Quat {
	return Quat{W: 1, X: 0, Y: 0, Z: 0}
}

// NewQuatFromAxisAngle creates a normalized quaternion representing a rotation
// of `angle` radians around `axis`.
func NewQuatFromAxisAngle(axis Point3D, angle float32) Quat {
	normAxis := axis.Normalize()
	if normAxis.LengthSq() < 1e-12 {
		return IdentityQuat()
	}
	half := angle * 0.5
	s := math32.Sin(half)
	c := math32.Cos(half)
	return Quat{
		W: c,
		X: normAxis.X * s,
		Y: normAxis.Y * s,
		Z: normAxis.Z * s,
	}.Normalize()
}

// NewQuatFromEuler builds a quaternion from Euler angles (in radians) applied in X, Y, Z order.
func NewQuatFromEuler(euler Point3D) Quat {
	qx := NewQuatFromAxisAngle(Point3D{X: 1, Y: 0, Z: 0}, euler.X)
	qy := NewQuatFromAxisAngle(Point3D{X: 0, Y: 1, Z: 0}, euler.Y)
	qz := NewQuatFromAxisAngle(Point3D{X: 0, Y: 0, Z: 1}, euler.Z)
	return qx.Mul(qy).Mul(qz).Normalize()
}

// Normalize returns a unit-length copy of the quaternion.
func (q Quat) Normalize() Quat {
	lenSq := q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z
	if lenSq < 1e-12 {
		return IdentityQuat()
	}
	invLen := 1.0 / math32.Sqrt(lenSq)
	return Quat{
		W: q.W * invLen,
		X: q.X * invLen,
		Y: q.Y * invLen,
		Z: q.Z * invLen,
	}
}

// Conjugate returns the conjugate (inverse for unit quaternions) of q.
func (q Quat) Conjugate() Quat {
	return Quat{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}
}

// Mul returns the Hamilton product q * other (applying `other` first, then `q`).
func (q Quat) Mul(other Quat) Quat {
	return Quat{
		W: q.W*other.W - q.X*other.X - q.Y*other.Y - q.Z*other.Z,
		X: q.W*other.X + q.X*other.W + q.Y*other.Z - q.Z*other.Y,
		Y: q.W*other.Y - q.X*other.Z + q.Y*other.W + q.Z*other.X,
		Z: q.W*other.Z + q.X*other.Y - q.Y*other.X + q.Z*other.W,
	}
}

// Rotate rotates a Point3D by the unit quaternion q.
func (q Quat) Rotate(p Point3D) Point3D {
	// v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
	qv := Point3D{X: q.X, Y: q.Y, Z: q.Z}
	t := qv.Cross(p).Scale(2.0)
	return p.Add(t.Scale(q.W)).Add(qv.Cross(t))
}

// RotateAround rotates point `p` around `pivot` using quaternion `q`.
func RotateAround(p, pivot Point3D, q Quat) Point3D {
	rel := p.Sub(pivot)
	return pivot.Add(q.Rotate(rel))
}

// ToEuler converts the quaternion into XYZ Euler angles (radians) in a Point3D.
func (q Quat) ToEuler() Point3D {
	qn := q.Normalize()

	// X (roll)
	sinrCosp := 2.0 * (qn.W*qn.X + qn.Y*qn.Z)
	cosrCosp := 1.0 - 2.0*(qn.X*qn.X+qn.Y*qn.Y)
	rx := math32.Atan2(sinrCosp, cosrCosp)

	// Y (pitch)
	sinp := 2.0 * (qn.W*qn.Y - qn.Z*qn.X)
	var ry float32
	if math32.Abs(sinp) >= 1.0 {
		if sinp >= 0 {
			ry = math.Pi / 2
		} else {
			ry = -math.Pi / 2
		}
	} else {
		ry = math32.Asin(sinp)
	}

	// Z (yaw)
	sinyCosp := 2.0 * (qn.W*qn.Z + qn.X*qn.Y)
	cosyCosp := 1.0 - 2.0*(qn.Y*qn.Y+qn.Z*qn.Z)
	rz := math32.Atan2(sinyCosp, cosyCosp)

	return Point3D{X: rx, Y: ry, Z: rz}
}

// SlerpQuat performs spherical linear interpolation between quaternions `a` and `b`.
func SlerpQuat(a, b Quat, t float32) Quat {
	an := a.Normalize()
	bn := b.Normalize()

	dot := an.W*bn.W + an.X*bn.X + an.Y*bn.Y + an.Z*bn.Z
	if dot < 0 {
		bn = Quat{W: -bn.W, X: -bn.X, Y: -bn.Y, Z: -bn.Z}
		dot = -dot
	}

	if dot > 0.9995 {
		return Quat{
			W: an.W + t*(bn.W-an.W),
			X: an.X + t*(bn.X-an.X),
			Y: an.Y + t*(bn.Y-an.Y),
			Z: an.Z + t*(bn.Z-an.Z),
		}.Normalize()
	}

	theta0 := math32.Acos(dot)
	theta := theta0 * t
	sinTheta := math32.Sin(theta)
	sinTheta0 := math32.Sin(theta0)

	s0 := math32.Cos(theta) - dot*sinTheta/sinTheta0
	s1 := sinTheta / sinTheta0

	return Quat{
		W: s0*an.W + s1*bn.W,
		X: s0*an.X + s1*bn.X,
		Y: s0*an.Y + s1*bn.Y,
		Z: s0*an.Z + s1*bn.Z,
	}.Normalize()
}

// SetNodeRotation applies a quaternion rotation to any G3N node supporting SetRotation.
func SetNodeRotation(node interface {
	SetRotation(float32, float32, float32)
}, q Quat) {
	euler := q.ToEuler()
	node.SetRotation(euler.X, euler.Y, euler.Z)
}
