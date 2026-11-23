# `shape.go` — 2D Profile & 3D Swept Shape Geometry

`shape.go` provides lightweight 2D/3D math primitives (`Point2D`, `Point3D`), 2D polyline generators, a procedural 3D swept-mesh builder (`Shape`), and reusable configuration/interpolation descriptors (`ShapeInfo`).

---

## Core Coordinate Types

### `Point2D`
Represents a 2D coordinate in the profile plane (`X` = radial/horizontal offset, `Y` = vertical/profile axis).

```go
type Point2D struct {
	X float32
	Y float32
}
```

### `Point3D`
Represents a 3D coordinate or vector `(X, Y, Z)` with value-semantics vector math and G3N interoperability.

| Method / Function | Signature | Description |
| :--- | :--- | :--- |
| `Add` | `(p Point3D) Add(o Point3D) Point3D` | Vector addition `p + o`. |
| `Sub` | `(p Point3D) Sub(o Point3D) Point3D` | Vector subtraction `p - o`. |
| `Scale` | `(p Point3D) Scale(s float32) Point3D` | Scalar multiplication `p * s`. |
| `Dot` | `(p Point3D) Dot(o Point3D) float32` | Dot product. |
| `Cross` | `(p Point3D) Cross(o Point3D) Point3D` | Cross product $p \times o$. |
| `LengthSq` | `(p Point3D) LengthSq() float32` | Squared Euclidean length. |
| `Length` | `(p Point3D) Length() float32` | Euclidean length. |
| `Normalize` | `(p Point3D) Normalize() Point3D` | Unit-length vector (returns `{0, 0, 0}` if length < `1e-12`). |
| `ToMath32` | `(p Point3D) ToMath32() *math32.Vector3` | Converts to a G3N `*math32.Vector3`. |
| `Point3DFromMath32` | `Point3DFromMath32(v math32.Vector3) Point3D` | Converts from a G3N `math32.Vector3`. |
| `SetNodePosition` | `SetNodePosition(node, p Point3D)` | Sets a G3N `core.Node` position from a `Point3D`. |
| `GetNodePosition` | `GetNodePosition(node) Point3D` | Reads a G3N `core.Node` position as a `Point3D`. |

---

## `Shape` & `NewShape`

`Shape` embeds G3N's `core.Node` and generates both a closed, outward-wound triangle mesh (`solidMesh`) with UV texture coordinates and a toggleable wireframe overlay (`wireLines`).

### Constructor

```go
func NewShape(
	mat material.IMaterial,
	steps int,
	rotate []float32,
	move []float32,
	lines ...[]Point2D,
) *Shape
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `mat` | `material.IMaterial` | Surface material for the solid mesh (defaults to `MatDefaultShape()` if `nil`). |
| `steps` | `int` | Number of sweep steps $N$ (clamped to a minimum of `1`). |
| `rotate` | `[]float32` | Per-step rotation increments around the `+Y` axis, expressed in **full turns** (`0.0` to `1.0`, where `1.0` = $360^\circ$). Indexed via `step % len(rotate)`. |
| `move` | `[]float32` | Per-step lateral shift distances along the rotated `+Z` direction. Indexed via `step % len(move)`. |
| `lines` | `...[]Point2D` | One or more 2D profile polylines. Step `k` selects `lines[k % len(lines)]`. |

### Stepping, Defaults & Normalization Rules
1. **Slice Truncation:** Any entries in `rotate` or `move` at index $\ge \text{steps}$ are ignored.
2. **Default Full Turn:** If both `rotate` and `move` are `nil` or empty, `rotate` defaults to `[]float32{1.0 / float32(steps)}` so $N$ steps complete one full $360^\circ$ revolution.
3. **Automatic Line Length Equalization:** When multiple `lines` of different vertex counts are supplied, `NewShape` finds the maximum point count `maxPts` and proportionally repeats points in shorter lines so every ring has `maxPts` vertices (e.g., `[p1, p2, p3, p4]` extended to `8` becomes `[p1, p1, p2, p2, p3, p3, p4, p4]`).
4. **Origin Alignment:** After generating all `steps + 1` rings, all vertices are translated so `rings[0][0]` sits at `(0, 0, 0)`.
5. **End Capping & Winding:** When `CloseEnds` is `true` (default), open ends are automatically capped (via ear-clipping triangulation for shifted shapes or axis hub fans for revolved shapes), and triangle winding is verified via signed volume to guarantee outward-facing normals.
6. **Crease-Aware Smooth Normals:** Adjacent triangles whose face normals differ by less than $\approx 50^\circ$ ($\cos\theta \ge 0.64$) are averaged for smooth shading while preserving crisp edges on sharp corners.

### Methods on `*Shape`
* `(s *Shape) Rebuild()` — Recomputes rings, UVs, normals, solid mesh, and wireframe lines from the struct's current fields.
* `(s *Shape) SetWireframe(wireframe bool)` — Toggles visibility between the solid mesh (`!wireframe`) and the line wireframe (`wireframe`).

---

## `ShapeInfo`

`ShapeInfo` stores all parameters required to construct a `Shape`, enabling cloning, scaling, and morphing (`Lerp`) before instantiation.

```go
type ShapeInfo struct {
	Name       string
	Material   material.IMaterial
	Steps      int
	Rotate     []float32
	Move       []float32
	Lines      [][]Point2D
	ShapeScale Point3D
	// contains filtered or unexported fields
}
```

### Methods

#### `(si ShapeInfo) New() *Shape`
Calls `NewShape(si.Material, si.Steps, si.Rotate, si.Move, si.Lines...)` and applies `ShapeScale` via `SetScale(x3, y3, z3)` before returning the constructed `*Shape`.

#### `(si *ShapeInfo) Scale(s *ShapeInfo, x2, y2, z2, x3, y3, z3 float32) *ShapeInfo`
Scales `s` into `si` and returns `si`. Pass `1.0` for any unused scale factor:
* `x2, y2` — Scales the 2D `X` and `Y` coordinates of every profile polyline in `Lines` using `LineScale`.
* `z2` — Multiplies each step distance in `Move`.
* `x3, y3, z3` — Multiplies the post-construction 3D scale (`ShapeScale`) applied when `New()` is called.

#### `(si *ShapeInfo) Lerp(s1, s2 *ShapeInfo, ratio float32) *ShapeInfo`
Linearly interpolates `Steps`, `Rotate`, `Move`, `Lines`, and `ShapeScale` between `s1` (`ratio = 0.0`) and `s2` (`ratio = 1.0`), writing the result into `si` and returning `si`.
* `Name` and `Material` switch from `s1` to `s2` when `ratio > 0.5`.
* **Constraint:** All variable-length slices (`Rotate`, `Move`, `Lines`, and each `Lines[i]`) must have identical lengths in `s1` and `s2`, or `Lerp` panics with a descriptive mismatch message.

---

## 2D Profile Builders

| Function | Signature | Description |
| :--- | :--- | :--- |
| `LineScale` | `LineScale(line []Point2D, x, y float32) []Point2D` | Returns a copy of `line` with coordinates `(pt.X * x, pt.Y * y)`. |
| `LineCircle` | `LineCircle(steps int, fromRotation, toRotation float32, center, radius Point2D) []Point2D` | Generates an elliptical/circular arc with `steps` segments (`steps + 1` points) from `fromRotation` to `toRotation` (in turns `0.0..1.0`). |
| `NewLineCircle` | `NewLineCircle(radius float32, segments int) []Point2D` | Closed 2D circle centered at `(0, 0)` with `segments` subdivisions. |
| `NewLineHalfCircle` | `NewLineHalfCircle(radius float32, segments int) []Point2D` | Open half-circle from `(0, -radius)` to `(0, +radius)` for revolving a sphere. |
| `NewLineRect` | `NewLineRect(width, height float32) []Point2D` | Closed 2D rectangle starting at `(0, 0)`. |
| `NewLineCenteredSquare` | `NewLineCenteredSquare(halfSize float32) []Point2D` | Closed 2D square centered at `(0, 0)` from `-halfSize` to `+halfSize`. |

---

## Usage Examples

### 1. Revolved Sphere (Default Full Rotation)
```go
sphere := NewShape(MatGoldMetal(), 28, nil, nil, NewLineHalfCircle(0.65, 20))
```

### 2. Extruded Pillar with Variable Step Distances & Morphing Profiles
```go
sqCap := NewLineCenteredSquare(0.46)
roundShaft := LineCircle(24, -0.375, 0.625, Point2D{}, Point2D{X: 0.28, Y: 0.28})

moves := []float32{0.26, 0.14, 2.15, 0.14, 0.26}
profiles := [][]Point2D{sqCap, sqCap, roundShaft, roundShaft, sqCap, sqCap}

pillar := NewShape(MatSandstone(2, 3), 5, nil, moves, profiles...)
```

### 3. Scaling and Interpolating `ShapeInfo`
```go
base := ShapeInfo{
	Name:     "EllipticalTorus",
	Material: MatCyanGloss(),
	Steps:    32,
	Rotate:   []float32{1.0 / 32.0},
	Move:     []float32{0.18},
	Lines:    [][]Point2D{LineCircle(24, 0.0, 1.0, Point2D{}, Point2D{X: 0.32, Y: 0.16})},
}

var stretched ShapeInfo
stretched.Scale(&base, 0.7, 1.5, 1.2, 1.0, 1.0, 1.0)
shape := stretched.New()
```