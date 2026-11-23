# `rotate.go` — Quaternion Rotation & Node Orientation

`rotate.go` provides a lightweight unit quaternion type (`Quat`) built on top of `Point3D`, along with helpers for composing 3D rotations, rotating vectors, and applying orientations directly to G3N scene nodes without gimbal lock.

---

## Coordinate & Rotation Conventions

* **Right-Handed Coordinate System:**
  * `+X` — Right
  * `+Y` — Up (vertical axis)
  * `+Z` — Forward
* **Right-Hand Rule:** Positive angles (in **radians**) rotate counter-clockwise when looking down the positive axis toward the origin:
  * Around `+Y` (`Point3D{Y: 1}`): positive yaw turns left (`+Z` toward `+X`).
  * Around `+X` (`Point3D{X: 1}`): positive pitch tilts `+Y` toward `+Z` ($-\pi / 2$ stands a `+Z`-extruded shape upright along `+Y`).
  * Around `+Z` (`Point3D{Z: 1}`): positive roll tilts `+X` toward `+Y`.

---

## `Quat` Type

```go
type Quat struct {
	X float32
	Y float32
	Z float32
	W float32
}
```

### Constructors

| Function | Signature | Description |
| :--- | :--- | :--- |
| `IdentityQuat` | `IdentityQuat() Quat` | Returns the identity quaternion `{X: 0, Y: 0, Z: 0, W: 1}` (no rotation). |
| `NewQuatFromAxisAngle` | `NewQuatFromAxisAngle(axis Point3D, angle float32) Quat` | Constructs a unit quaternion representing a rotation of `angle` **radians** around `axis` (automatically normalized). Falls back to `IdentityQuat()` if `axis` is zero-length. |

### Quaternion Operations

| Method | Signature | Description |
| :--- | :--- | :--- |
| `Normalize` | `(q Quat) Normalize() Quat` | Returns a unit-length quaternion (or `IdentityQuat()` if norm is near zero). |
| `Conjugate` | `(q Quat) Conjugate() Quat` | Returns `{-q.X, -q.Y, -q.Z, q.W}` (equal to the inverse for unit quaternions). |
| `Mul` | `(q Quat) Mul(o Quat) Quat` | Hamilton product `q * o`. Applying `(q.Mul(o)).Rotate(v)` applies rotation `o` first, followed by rotation `q`. |
| `Rotate` | `(q Quat) Rotate(v Point3D) Point3D` | Rotates 3D vector `v` by unit quaternion `q` using $v' = v + 2 w (u \times v) + 2 (u \times (u \times v))$. |
| `Slerp` | `(q Quat) Slerp(o Quat, t float32) Quat` | Spherical linear interpolation between `q` (`t = 0`) and `o` (`t = 1`) along the shortest arc. |
| `ToMath32` | `(q Quat) ToMath32() *math32.Quaternion` | Converts to a G3N `*math32.Quaternion`. |
| `QuatFromMath32` | `QuatFromMath32(mq math32.Quaternion) Quat` | Converts from a G3N `math32.Quaternion`. |

---

## G3N Node Helpers

| Function | Signature | Description |
| :--- | :--- | :--- |
| `SetNodeRotation` | `SetNodeRotation(node, q Quat)` | Sets a G3N `core.Node`'s orientation from `q` via `SetQuaternion(q.X, q.Y, q.Z, q.W)`. |
| `GetNodeRotation` | `GetNodeRotation(node) Quat` | Retrieves a G3N `core.Node`'s current orientation as a `Quat`. |

---

## Usage Examples

### 1. Standing a Z-Swept Shape Upright
Shapes swept with `move` advance along `+Z`. Rotating by $-\pi / 2$ around `+X` aligns the sweep axis with `+Y`:
```go
upright := NewQuatFromAxisAngle(Point3D{X: 1}, -math.Pi/2)
SetNodeRotation(pillarShape, upright)
```

### 2. Composing Camera Yaw and Pitch
```go
qYaw := NewQuatFromAxisAngle(Point3D{Y: 1}, camYaw)
qPitch := NewQuatFromAxisAngle(Point3D{X: 1}, -camPitch)

// Apply pitch in local space first, then rotate around world Y by yaw
camOffset := qYaw.Mul(qPitch).Rotate(Point3D{Z: camDistance})
```

### 3. Transforming Local Velocity to World Space
```go
robotQuat := NewQuatFromAxisAngle(Point3D{Y: 1}, robotHeading)
localVel := Point3D{X: velStrafe, Y: 0, Z: velForward}
worldVel := robotQuat.Rotate(localVel)
robotPos = robotPos.Add(worldVel.Scale(dt))
```