### Project Overview

**Z-Bot 3D** is a Go 3D robotics and procedural geometry simulation built with the **G3N** engine (`[github.com/g3n/engine](https://github.com/g3n/engine)`). It features a 4-wheel-steering (4WS) robot navigating a 3D environment populated with procedurally swept shapes, textured buildings, pillars, and a brick maze.

---

### Codebase File Architecture

| File | Responsibility |
| --- | --- |
| `main.go` | G3N window/scene/lighting setup, orbit/freeze camera, keyboard driving/mode/fullscreen controls, infinite floor grid, and Z-Bot hierarchy (`chassis`, 4 `wheelSteer`/`wheelRoll` nodes, `centralArm`, `sideArmL`, `sideArmR`). |
| `rotate.go` | `Quat` unit quaternion math (`IdentityQuat`, `NewQuatFromAxisAngle`, `Mul`, `Rotate`, `Slerp`, `Normalize`, `Conjugate`) and G3N node orientation helpers (`SetNodeRotation`, `GetNodeRotation`). |
| `shape.go` | `Point2D` / `Point3D` math, `Shape` swept-mesh generator (`NewShape`, `Rebuild`, `SetWireframe`), `ShapeInfo` (`New`, `Scale`, `Lerp`), 2D polyline builders, and ear-clipping polygon triangulation. |
| `material.go` | Generic material builders (solid, metallic, emissive, wireframe, bitmap from RGBA/files), procedural bitmap generators (brick, roof tile, wood, stone, brushed metal, checker), and core default/robot materials (`MatDefaultShape`, `MatFloorGrid`, `MatRobot*`). |
| `myMaterial.go` | Project-specific world materials (`MatGoldMetal`, `MatSteelBlueMetal`, `MatBronzeMetal`, `MatBrick01`–`MatBrick04`, `MatBrickByIndex`, `MatBrickWall`, `MatTileRoof`, `MatSandstone`, `MatGranite`, `MatMarble`, etc.). |
| `world.go` | `World` scene manager, generic placement helpers (`AddShape`, `AddUprightShape`, `AddWallBox`, `ToggleWireframe`), and `Build()` for standard world objects and feature demos. |
| `myWorld.go` | Project-specific world structures—currently `(w *World) BuildMaze()`, which builds a 6×6 navigable maze using `material/brick 01.jpg`..`04.jpg`. |
| `shape.md` & `rotate.md` | Up-to-date Markdown API documentation for `shape.go` and `rotate.go`. |

---

### Current Key Signatures & Conventions (`shape.go`, `material.go`, `world.go`)

#### `Shape` & `NewShape` (`shape.go`)

```go
func NewShape(
	mat material.IMaterial,
	steps int,
	rotate []float32,
	move []float32,
	lines ...[]Point2D,
) *Shape

```

* **`mat`**: First parameter (`material.IMaterial`); falls back to `MatDefaultShape()` if `nil`.
* **`rotate` & `move**`:
* `rotate` is expressed in **full turns (`0.0` to `1.0`)** around `+Y` (not degrees or radians).
* `move` is lateral translation distance per step along the rotated `+Z` axis.
* Both are indexed via `step % len(...)`; slice elements beyond `steps` are ignored.
* If both `rotate` and `move` are `nil` or empty, `rotate` defaults to `[]float32{1.0 / float32(steps)}` (1 full rotation over `steps`).


* **`lines`**: One or more `[]Point2D` polylines cycled via `step % len(lines)`. Shorter polylines are automatically extended to match the longest polyline via proportional point repetition (`extendLineByRepeating`).
* **Mesh details**: Generates outward-wound triangles with UV coordinates (`gls.VertexTexcoord`), crease-aware smooth normals (~50° threshold), automatic end-capping (`CloseEnds: true`), origin anchored at `rings[0][0] == (0, 0, 0)`, and a toggleable wireframe (`SetWireframe`).

#### `ShapeInfo` & 2D Line Helpers (`shape.go`)

```go
type ShapeInfo struct {
	Name       string
	Material   material.IMaterial
	Steps      int
	Rotate     []float32
	Move       []float32
	Lines      [][]Point2D
	ShapeScale Point3D
	// unexported hasScale3D bool
}

func (si ShapeInfo) New() *Shape
func (si *ShapeInfo) Scale(s *ShapeInfo, x2, y2, z2, x3, y3, z3 float32) *ShapeInfo
func (si *ShapeInfo) Lerp(s1, s2 *ShapeInfo, ratio float32) *ShapeInfo

func LineScale(line []Point2D, x, y float32) []Point2D
func LineCircle(steps int, fromRotation, toRotation float32, center, radius Point2D) []Point2D
func NewLineCircle(radius float32, segments int) []Point2D
func NewLineHalfCircle(radius float32, segments int) []Point2D
func NewLineRect(width, height float32) []Point2D
func NewLineCenteredSquare(halfSize float32) []Point2D

```

* `ShapeInfo.Scale` requires all 6 scale factors (pass `1.0` for unused): `(x2, y2)` scales 2D line points, `z2` scales `Move`, and `(x3, y3, z3)` applies `SetScale` to the constructed `*Shape` inside `New()`.
* `ShapeInfo.Lerp` requires equal lengths on all variable-length slices (`Rotate`, `Move`, `Lines`, and each `Lines[i]`).

#### `World` & `Material` Helpers

* `World` (`world.go`): `AddShape(s, pos)`, `AddUprightShape(s, centerBottom, offsetX, offsetZ)` (rotates `-math.Pi/2` around `X` to stand `Z`-swept shapes upright along `+Y`), `AddWallBox(x0, y0, z0, lengthX, heightY, depthZ, mat)`.
* `material.go`: `NewSolidMaterial`, `NewSolidMaterialWithOptions`, `NewMetallicMaterial`, `NewEmissiveMaterial`, `NewWireframeMaterialFrom`, `NewBitmapMaterialFromRGBA`, `NewBitmapMaterialFromFile`, `NewBitmapMaterialFromFiles`, `ResolveAssetPath`.

---

### Current Scene & Controls

* **Controls (`main.go`)**:
* `W`/`S`: Drive forward/backward; `Q`/`E`: Strafe left/right; `A`/`D`: 4WS steer (while driving) or spin in place (when stationary); `Shift`: Speed boost.
* `1`–`4`: Robot posture modes (Nested, SUV/Platform, Bimanual, Hand-locomotion).
* `M`: Toggle wireframe mode on all `World` shapes (`w.ToggleWireframe()`).
* `F`: Toggle window / borderless fullscreen (`application.SetFullScreen(!application.FullScreen())`).
* Mouse move / scroll: Orbit & zoom camera; Left-click: Freeze/unfreeze camera.


* **World Contents (`world.go` & `myWorld.go`)**:
* Pine tree (bark bitmap trunk, foliage cones, gold sphere), brick house with barrel-tile roof, steel-blue cube, brushed-steel block, gold & crimson spheres, flat & upright tori, fluted marble column, square-to-round-to-square pillars (sandstone, granite, bronze), `ShapeInfo.Lerp` & `ShapeInfo.Scale` demo shapes, and the 6×6 brick maze (`BuildMaze` in `myWorld.go`).