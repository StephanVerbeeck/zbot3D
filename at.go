package main

import (
	"math"

	"github.com/g3n/engine/core"
	"github.com/g3n/engine/math32"
)

// At represents a 3D pose (translation + orientation) combining a Move vector
// and a Turn quaternion.
type At struct {
	Move Point3D
	Turn Quat
}

// IdentityAt returns the identity transform at the origin with no rotation.
func IdentityAt() At {
	return At{
		Move: Point3D{},
		Turn: IdentityQuat(),
	}
}

// NewAt constructs an At transform from a Move vector and a normalized Turn quaternion.
func NewAt(move Point3D, turn Quat) At {
	return At{
		Move: move,
		Turn: turn.Normalize(),
	}
}

// NewAtXYZ constructs an At transform from (x, y, z) coordinates and a Turn quaternion.
func NewAtXYZ(x, y, z float32, turn Quat) At {
	return At{
		Move: Point3D{X: x, Y: y, Z: z},
		Turn: turn.Normalize(),
	}
}

// AtMove constructs a pure translation At transform with identity rotation.
func AtMove(move Point3D) At {
	return At{
		Move: move,
		Turn: IdentityQuat(),
	}
}

// AtXYZ constructs a pure translation At transform from (x, y, z) coordinates.
func AtXYZ(x, y, z float32) At {
	return At{
		Move: Point3D{X: x, Y: y, Z: z},
		Turn: IdentityQuat(),
	}
}

// AtTurn constructs a pure rotational At transform at the origin.
func AtTurn(turn Quat) At {
	return At{
		Move: Point3D{},
		Turn: turn.Normalize(),
	}
}

// NewAtAxisAngle constructs an At transform from a translation and an axis-angle rotation (in radians).
func NewAtAxisAngle(move Point3D, axis Point3D, angle float32) At {
	return At{
		Move: move,
		Turn: NewQuatFromAxisAngle(axis, angle),
	}
}

// NewAtAxisTurns constructs an At transform from a translation and an axis rotation in full turns (1.0 = 360°).
func NewAtAxisTurns(move Point3D, axis Point3D, turns float32) At {
	return At{
		Move: move,
		Turn: NewQuatFromAxisAngle(axis, turns*2.0*math.Pi),
	}
}

// LookAt constructs an At positioned at `eye` with its local +Z axis directed toward `target`
// and local +Y aligned with `up`.
func LookAt(eye, target, up Point3D) At {
	forward := target.Sub(eye).Normalize()
	if forward.LengthSq() < 1e-12 {
		return AtMove(eye)
	}

	upNorm := up.Normalize()
	if upNorm.LengthSq() < 1e-12 {
		upNorm = Point3D{Y: 1}
	}
	if math32.Abs(forward.Dot(upNorm)) > 0.9999 {
		if math32.Abs(forward.Z) < 0.9999 {
			upNorm = Point3D{Z: 1}
		} else {
			upNorm = Point3D{X: 1}
		}
	}

	right := upNorm.Cross(forward).Normalize()
	trueUp := forward.Cross(right).Normalize()

	var q Quat
	trace := right.X + trueUp.Y + forward.Z
	if trace > 0 {
		s := math32.Sqrt(trace+1.0) * 2.0
		q = Quat{
			W: 0.25 * s,
			X: (trueUp.Z - forward.Y) / s,
			Y: (forward.X - right.Z) / s,
			Z: (right.Y - trueUp.X) / s,
		}
	} else if right.X > trueUp.Y && right.X > forward.Z {
		s := math32.Sqrt(1.0+right.X-trueUp.Y-forward.Z) * 2.0
		q = Quat{
			W: (trueUp.Z - forward.Y) / s,
			X: 0.25 * s,
			Y: (trueUp.X + right.Y) / s,
			Z: (forward.X + right.Z) / s,
		}
	} else if trueUp.Y > forward.Z {
		s := math32.Sqrt(1.0+trueUp.Y-right.X-forward.Z) * 2.0
		q = Quat{
			W: (forward.X - right.Z) / s,
			X: (trueUp.X + right.Y) / s,
			Y: 0.25 * s,
			Z: (forward.Y + trueUp.Z) / s,
		}
	} else {
		s := math32.Sqrt(1.0+forward.Z-right.X-trueUp.Y) * 2.0
		q = Quat{
			W: (right.Y - trueUp.X) / s,
			X: (forward.X + right.Z) / s,
			Y: (forward.Y + trueUp.Z) / s,
			Z: 0.25 * s,
		}
	}

	return At{
		Move: eye,
		Turn: q.Normalize(),
	}
}

// Normalized returns a copy of `a` whose Turn quaternion is guaranteed to be unit-length
// (defaulting a zero-initialized Quat to IdentityQuat).
func (a At) Normalized() At {
	return At{
		Move: a.Move,
		Turn: a.Turn.Normalize(),
	}
}

// WithMove returns a copy of `a` with its Move translation replaced by `move`.
func (a At) WithMove(move Point3D) At {
	return At{
		Move: move,
		Turn: a.Turn.Normalize(),
	}
}

// WithTurn returns a copy of `a` with its orientation replaced by `turn`.
func (a At) WithTurn(turn Quat) At {
	return At{
		Move: a.Move,
		Turn: turn.Normalize(),
	}
}

// At composes `local` inside `a`'s coordinate frame:
// first rotating and translating `local.Move` by `a`, then combining orientations `a.Turn * local.Turn`.
func (a At) At(local At) At {
	qa := a.Turn.Normalize()
	qb := local.Turn.Normalize()
	return At{
		Move: a.Move.Add(qa.Rotate(local.Move)),
		Turn: qa.Mul(qb).Normalize(),
	}
}

// Inverse returns the inverse rigid transform `a^-1` such that `a.At(a.Inverse()) == IdentityAt()`.
func (a At) Inverse() At {
	invTurn := a.Turn.Normalize().Conjugate()
	return At{
		Move: invTurn.Rotate(a.Move.Scale(-1.0)),
		Turn: invTurn,
	}
}

// RelativeTo expresses `a` in `base`'s local coordinate frame (`base.Inverse().At(a)`).
func (a At) RelativeTo(base At) At {
	return base.Inverse().At(a)
}

// TransformPoint maps a 3D point from `a`'s local space into parent/world space.
func (a At) TransformPoint(p Point3D) Point3D {
	return a.Move.Add(a.Turn.Normalize().Rotate(p))
}

// TransformDir rotates a 3D direction or velocity vector from `a`'s local space into parent/world space
// without applying translation.
func (a At) TransformDir(v Point3D) Point3D {
	return a.Turn.Normalize().Rotate(v)
}

// InverseTransformPoint maps a 3D point from parent/world space into `a`'s local space.
func (a At) InverseTransformPoint(p Point3D) Point3D {
	return a.Turn.Normalize().Conjugate().Rotate(p.Sub(a.Move))
}

// InverseTransformDir rotates a 3D direction or velocity vector from parent/world space into `a`'s local space
// without applying translation.
func (a At) InverseTransformDir(v Point3D) Point3D {
	return a.Turn.Normalize().Conjugate().Rotate(v)
}

// Translate returns a copy of `a` shifted by `delta` in parent/world coordinates.
func (a At) Translate(delta Point3D) At {
	return At{
		Move: a.Move.Add(delta),
		Turn: a.Turn.Normalize(),
	}
}

// TranslateLocal returns a copy of `a` shifted by `localDelta` along `a`'s own rotated axes.
func (a At) TranslateLocal(localDelta Point3D) At {
	q := a.Turn.Normalize()
	return At{
		Move: a.Move.Add(q.Rotate(localDelta)),
		Turn: q,
	}
}

// Rotate returns a copy of `a` rotated by `q` in parent/world orientation space.
func (a At) Rotate(q Quat) At {
	return At{
		Move: a.Move,
		Turn: q.Normalize().Mul(a.Turn.Normalize()).Normalize(),
	}
}

// RotateLocal returns a copy of `a` rotated by `q` around `a`'s local axes.
func (a At) RotateLocal(q Quat) At {
	return At{
		Move: a.Move,
		Turn: a.Turn.Normalize().Mul(q.Normalize()).Normalize(),
	}
}

// RotateAround orbits both `a.Move` and `a.Turn` around `pivot` by quaternion `q`.
func (a At) RotateAround(pivot Point3D, q Quat) At {
	qn := q.Normalize()
	return At{
		Move: RotateAround(a.Move, pivot, qn),
		Turn: qn.Mul(a.Turn.Normalize()).Normalize(),
	}
}

// --- Transform Composition & Interpolation ---

// CombineAt combines any number of At transforms in hierarchical parent-to-child order,
// yielding a single equivalent At transform. Passing zero arguments returns IdentityAt().
func CombineAt(transforms ...At) At {
	acc := IdentityAt()
	for _, tr := range transforms {
		acc = acc.At(tr)
	}
	return acc
}

// ChainAt computes the cumulative world-space transforms for a parent-to-child chain,
// where result[i] = CombineAt(transforms[:i+1]...).
func ChainAt(transforms ...At) []At {
	if len(transforms) == 0 {
		return nil
	}
	out := make([]At, len(transforms))
	acc := IdentityAt()
	for i, tr := range transforms {
		acc = acc.At(tr)
		out[i] = acc
	}
	return out
}

// LerpAt interpolates smoothly between `a` (t = 0) and `b` (t = 1),
// using linear interpolation for Move and spherical linear interpolation (SlerpQuat) for Turn.
func LerpAt(a, b At, t float32) At {
	return At{
		Move: a.Move.Add(b.Move.Sub(a.Move).Scale(t)),
		Turn: SlerpQuat(a.Turn, b.Turn, t),
	}
}

// BlendAt computes a weighted combination of multiple At transforms into one.
// Weights do not need to sum to 1.0; if total weight is near zero, IdentityAt() is returned.
func BlendAt(transforms []At, weights []float32) At {
	n := len(transforms)
	if len(weights) < n {
		n = len(weights)
	}
	if n == 0 {
		return IdentityAt()
	}

	var totalW float32
	for i := 0; i < n; i++ {
		if weights[i] > 0 {
			totalW += weights[i]
		}
	}
	if totalW < 1e-12 {
		return IdentityAt()
	}

	var moveSum Point3D
	var qAcc Quat
	var accW float32

	for i := 0; i < n; i++ {
		w := weights[i]
		if w <= 0 {
			continue
		}
		nw := w / totalW
		ti := transforms[i]
		moveSum = moveSum.Add(ti.Move.Scale(nw))

		qi := ti.Turn.Normalize()
		if accW == 0 {
			qAcc = qi
			accW = nw
		} else {
			accW += nw
			qAcc = SlerpQuat(qAcc, qi, nw/accW)
		}
	}

	return At{
		Move: moveSum,
		Turn: qAcc.Normalize(),
	}
}

// TransformPoints maps a slice of 3D points through `a` into parent/world space.
func TransformPoints(a At, pts []Point3D) []Point3D {
	if len(pts) == 0 {
		return nil
	}
	norm := a.Normalized()
	out := make([]Point3D, len(pts))
	for i, p := range pts {
		out[i] = norm.Move.Add(norm.Turn.Rotate(p))
	}
	return out
}

// --- G3N Scene Node & Shape Helpers ---

// SetNodeAt applies both position and rotation from `at` to a *core.Node.
func SetNodeAt(node *core.Node, at At) {
	norm := at.Normalized()
	SetNodePosition(node, norm.Move)
	SetNodeRotation(node, norm.Turn)
}

// GetNodeAt reads a *core.Node's current position and Euler rotation as an At transform.
func GetNodeAt(node *core.Node) At {
	return At{
		Move: GetNodePosition(node),
		Turn: NewQuatFromEuler(Point3DFromMath32(node.Rotation())),
	}
}

// PlaceNodeAt combines one or more At transforms, applies the result to `node`, and returns `node`.
func PlaceNodeAt(node *core.Node, transforms ...At) *core.Node {
	SetNodeAt(node, CombineAt(transforms...))
	return node
}

// SetShapeAt applies both position and rotation from `at` to a *Shape.
func SetShapeAt(s *Shape, at At) {
	SetNodeAt(&s.Node, at)
}

// GetShapeAt reads a *Shape's current position and Euler rotation as an At transform.
func GetShapeAt(s *Shape) At {
	return GetNodeAt(&s.Node)
}

// PlaceShapeAt combines one or more At transforms, applies the result to `s`, and returns `s`.
func PlaceShapeAt(s *Shape, transforms ...At) *Shape {
	SetShapeAt(s, CombineAt(transforms...))
	return s
}
