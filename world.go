package main

import (
	"math"

	"github.com/g3n/engine/core"
	"github.com/g3n/engine/material"
)

// World manages the 3D Shape objects placed in the robot's environment.
type World struct {
	scene     *core.Node
	shapes    []*Shape
	wireframe bool
}

func NewWorld(scene *core.Node) *World {
	return &World{
		scene: scene,
	}
}

func (w *World) AddShape(s *Shape, pos Point3D) *Shape {
	SetNodePosition(s, pos)
	w.scene.Add(s)
	w.shapes = append(w.shapes, s)
	return s
}

// AddUprightShape rotates a Z-swept shape by -90° around X so it stands vertically along +Y,
// and positions it at `centerBottom` offset by `(offsetX, 0, offsetZ)`.
func (w *World) AddUprightShape(s *Shape, centerBottom Point3D, offsetX, offsetZ float32) *Shape {
	SetNodeRotation(s, NewQuatFromAxisAngle(Point3D{X: 1}, -math.Pi/2))
	return w.AddShape(s, centerBottom.Add(Point3D{X: offsetX, Z: offsetZ}))
}

// AddWallBox creates and places an axis-aligned rectangular wall or block starting at (x0, y0, z0).
func (w *World) AddWallBox(x0, y0, z0, lengthX, heightY, depthZ float32, mat material.IMaterial) *Shape {
	wall := NewShape(mat, 1, nil, []float32{depthZ}, NewLineRect(lengthX, heightY))
	return w.AddShape(wall, Point3D{X: x0, Y: y0, Z: z0})
}

func (w *World) ToggleWireframe() {
	w.wireframe = !w.wireframe
	for _, s := range w.shapes {
		s.SetWireframe(w.wireframe)
	}
}

func (w *World) Build() {
	// --- 1. Kerstboom ---
	treePos := Point3D{X: 3.8, Y: floorY, Z: 3.8}

	trunkLine := []Point2D{
		{X: 0.0, Y: 0.0},
		{X: 0.22, Y: 0.0},
		{X: 0.19, Y: 0.75},
		{X: 0.0, Y: 0.75},
	}
	w.AddShape(NewShape(MatTreeBark(2, 2), 20, nil, nil, trunkLine), treePos)

	treeTopLine := []Point2D{
		{X: 0.0, Y: 0.0},
		{X: 1.15, Y: 0.0},
		{X: 0.55, Y: 0.85},
		{X: 0.90, Y: 0.85},
		{X: 0.38, Y: 1.65},
		{X: 0.65, Y: 1.65},
		{X: 0.0, Y: 2.55},
	}
	w.AddShape(NewShape(MatPineFoliage(), 24, nil, nil, treeTopLine), treePos.Add(Point3D{Y: 0.65}))

	w.AddShape(NewShape(MatGoldMetal(), 16, nil, nil, NewLineHalfCircle(0.16, 12)), treePos.Add(Point3D{Y: 0.65 + 2.50}))

	// --- 2. Bakstenen Huisje met Pannendak ---
	houseOrigin := Point3D{X: -6.8, Y: floorY, Z: 4.5}
	wallWidth := float32(2.4)
	wallHeight := float32(1.5)
	wallDepth := float32(2.0)

	w.AddWallBox(houseOrigin.X, floorY, houseOrigin.Z, wallWidth, wallHeight, wallDepth, MatBrickWall(3, 2))

	roofOverhang := float32(0.25)
	roofPitch := float32(0.95)
	roofTriangle := []Point2D{
		{X: -roofOverhang, Y: 0.0},
		{X: wallWidth + roofOverhang, Y: 0.0},
		{X: wallWidth * 0.5, Y: roofPitch},
		{X: -roofOverhang, Y: 0.0},
	}
	tileRoof := NewShape(MatTileRoof(3, 2), 1, nil, []float32{wallDepth + 2*roofOverhang}, roofTriangle)
	w.AddShape(tileRoof, houseOrigin.Add(Point3D{X: -roofOverhang, Y: wallHeight, Z: -roofOverhang}))

	// --- 3. Kubussen & Balken ---
	w.AddShape(NewShape(MatSteelBlueMetal(), 1, nil, []float32{1.0}, NewLineRect(1.0, 1.0)), Point3D{X: -3.5, Y: floorY, Z: 3.2})
	w.AddShape(NewShape(MatBrushedSteel(2, 2), 2, nil, []float32{0.35, 0.55}, NewLineRect(0.8, 1.8)), Point3D{X: 4.5, Y: floorY, Z: -2.8})

	// --- 4. Bollen ---
	w.AddShape(NewShape(MatGoldMetal(), 28, nil, nil, NewLineHalfCircle(0.65, 20)), Point3D{X: -2.8, Y: floorY, Z: -3.5})
	w.AddShape(NewShape(MatCrimsonGloss(), 24, nil, nil, NewLineHalfCircle(0.45, 18)), Point3D{X: -4.8, Y: floorY, Z: 0.5})

	// --- 5. Torussen ---
	w.AddShape(NewShape(MatVioletGloss(), 32, []float32{1.0 / 32.0}, []float32{0.18}, NewLineCircle(0.24, 18)), Point3D{X: 2.2, Y: floorY + 0.24, Z: -4.2})

	archTorus := NewShape(MatCyanGloss(), 28, []float32{1.0 / 28.0}, []float32{0.20}, NewLineCircle(0.20, 16))
	SetNodeRotation(archTorus, NewQuatFromAxisAngle(Point3D{X: 1}, math.Pi/2))
	w.AddShape(archTorus, Point3D{X: 0.0, Y: floorY + 1.09, Z: 6.0})

	// --- 6. Gecanneleerde marmeren zuil ---
	fluteOuter := []Point2D{
		{X: 0.0, Y: 0.0},
		{X: 0.46, Y: 0.0},
		{X: 0.34, Y: 0.45},
		{X: 0.50, Y: 0.95},
		{X: 0.36, Y: 1.35},
		{X: 0.0, Y: 1.35},
	}
	fluteInner := []Point2D{
		{X: 0.0, Y: 0.0},
		{X: 0.46, Y: 0.0},
		{X: 0.24, Y: 0.45},
		{X: 0.38, Y: 0.95},
		{X: 0.28, Y: 1.35},
		{X: 0.0, Y: 1.35},
	}
	w.AddShape(NewShape(MatMarble(3, 2), 32, nil, nil, fluteOuter, fluteInner), Point3D{X: 0.0, Y: floorY, Z: -5.5})

	// --- 7. Pilaren (vierkant onder/boven, rond in het midden) ---
	sqCap := NewLineCenteredSquare(0.46)
	roundShaft := LineCircle(24, -0.375, 0.625, Point2D{}, Point2D{X: 0.28, Y: 0.28})
	classicMoves := []float32{0.26, 0.14, 2.15, 0.14, 0.26}
	classicProfiles := [][]Point2D{sqCap, sqCap, roundShaft, roundShaft, sqCap, sqCap}

	w.AddUprightShape(NewShape(MatSandstone(2, 3), 5, nil, classicMoves, classicProfiles...), Point3D{X: -2.2, Y: floorY, Z: 5.5}, -0.46, 0.46)
	w.AddUprightShape(NewShape(MatSandstone(2, 3), 5, nil, classicMoves, classicProfiles...), Point3D{X: 2.2, Y: floorY, Z: 5.5}, -0.46, 0.46)

	widePlinth := NewLineCenteredSquare(0.58)
	midNeck := NewLineCenteredSquare(0.42)
	slimShaft := LineCircle(24, -0.375, 0.625, Point2D{}, Point2D{X: 0.26, Y: 0.26})
	tieredMoves := []float32{0.22, 0.13, 0.09, 2.45, 0.09, 0.13, 0.22}
	tieredProfiles := [][]Point2D{
		widePlinth, widePlinth, midNeck,
		slimShaft, slimShaft,
		midNeck, widePlinth, widePlinth,
	}
	w.AddUprightShape(NewShape(MatGranite(2, 3), 7, nil, tieredMoves, tieredProfiles...), Point3D{X: 5.8, Y: floorY, Z: 0.8}, -0.58, 0.58)

	sqEnd := NewLineCenteredSquare(0.48)
	roundWide := LineCircle(24, -0.375, 0.625, Point2D{}, Point2D{X: 0.34, Y: 0.34})
	roundNarrow := LineCircle(24, -0.375, 0.625, Point2D{}, Point2D{X: 0.24, Y: 0.24})
	modulusMoves := []float32{0.17, 0.61, 0.13}
	modulusProfiles := [][]Point2D{
		sqEnd, sqEnd,
		roundNarrow, roundWide, roundWide, roundNarrow, roundNarrow, roundWide,
		sqEnd, sqEnd,
	}
	w.AddUprightShape(NewShape(MatBronzeMetal(), 9, nil, modulusMoves, modulusProfiles...), Point3D{X: -5.8, Y: floorY, Z: -2.2}, -0.48, 0.48)

	// --- 8. ShapeInfo + Lerp + Scale demonstratie ---
	infoStart := ShapeInfo{
		Name:     "StraightColumn",
		Material: MatSteelBlueMetal(),
		Steps:    12,
		Rotate:   []float32{0.0, 0.0},
		Move:     []float32{0.14},
		Lines:    [][]Point2D{NewLineCenteredSquare(0.36)},
	}
	infoEnd := ShapeInfo{
		Name:     "TwistedOctagon",
		Material: MatGoldMetal(),
		Steps:    12,
		Rotate:   []float32{0.04, 0.02},
		Move:     []float32{0.22},
		Lines:    [][]Point2D{LineCircle(4, -0.375, 0.625, Point2D{}, Point2D{X: 0.30, Y: 0.30})},
	}

	for idx, ratio := range []float32{0.0, 0.5, 1.0} {
		var morphed ShapeInfo
		morphed.Lerp(&infoStart, &infoEnd, ratio)
		w.AddUprightShape(morphed.New(), Point3D{X: -6.5 + float32(idx)*1.6, Y: floorY, Z: -5.5}, 0, 0)
	}

	ellipseProfile := LineCircle(24, 0.0, 1.0, Point2D{}, Point2D{X: 0.32, Y: 0.16})
	baseRing := ShapeInfo{
		Name:     "EllipticalTorus",
		Material: MatCyanGloss(),
		Steps:    32,
		Rotate:   []float32{1.0 / 32.0},
		Move:     []float32{0.18},
		Lines:    [][]Point2D{ellipseProfile},
	}

	var scaled2D ShapeInfo
	scaled2D.Scale(&baseRing, 0.7, 1.5, 1.2, 1.0, 1.0, 1.0)
	w.AddShape(scaled2D.New(), Point3D{X: -6.5, Y: floorY + 0.24, Z: -1.5})

	var scaled3D ShapeInfo
	scaled3D.Scale(&baseRing, 1.0, 1.0, 1.0, 1.4, 0.6, 1.0)
	w.AddShape(scaled3D.New(), Point3D{X: -4.2, Y: floorY + 0.12, Z: -5.5})

	// --- 9. Project-specifieke wereldonderdelen (myWorld.go) ---
	w.BuildMaze()
}
