package main

import (
	"math"

	"github.com/g3n/engine/core"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
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

func (w *World) ToggleWireframe() {
	w.wireframe = !w.wireframe
	for _, s := range w.shapes {
		s.SetWireframe(w.wireframe)
	}
}

// centeredSquareLoop returns a closed 2D square centered at (0, 0), starting at (-halfSize, -halfSize).
func centeredSquareLoop(halfSize float32) []Point2D {
	return []Point2D{
		{X: -halfSize, Y: -halfSize},
		{X: halfSize, Y: -halfSize},
		{X: halfSize, Y: halfSize},
		{X: -halfSize, Y: halfSize},
		{X: -halfSize, Y: -halfSize},
	}
}

// alignedCircleLoop returns a closed 2D circle centered at (0, 0) starting at -3π/4
// so its quadrants align symmetrically with centeredSquareLoop.
func alignedCircleLoop(radius float32, segments int) []Point2D {
	if segments < 4 {
		segments = 4
	}
	pts := make([]Point2D, segments+1)
	startAngle := float32(-3.0 * math.Pi / 4.0)
	for i := 0; i <= segments; i++ {
		theta := startAngle + (2.0*math.Pi*float32(i))/float32(segments)
		pts[i] = Point2D{
			X: radius * math32.Cos(theta),
			Y: radius * math32.Sin(theta),
		}
	}
	pts[segments] = pts[0]
	return pts
}

// addUprightShiftedPillar builds a shifted (Rotate=false) pillar with variable step distances,
// stands it upright along +Y, and places its bottom center on the floor at (centerX, floorY, centerZ).
func (w *World) addUprightShiftedPillar(
	centerX, centerZ float32,
	baseHalfSize float32,
	mat material.IMaterial,
	shiftDists []float32,
	profiles ...[]Point2D,
) *Shape {
	steps := len(profiles) - 1
	pillar := NewShape(steps, false, shiftDists, mat, profiles...)
	SetNodeRotation(pillar, NewQuatFromAxisAngle(Point3D{X: 1}, -math.Pi/2))
	// Offset by (-baseHalfSize, +baseHalfSize) so the centered bottom face sits at (centerX, floorY, centerZ)
	return w.AddShape(pillar, Point3D{
		X: centerX - baseHalfSize,
		Y: floorY,
		Z: centerZ + baseHalfSize,
	})
}

func (w *World) Build() {
	// --- 1. Kerstboom (bruine stam onderaan + groene gelaagde kruin bovenaan) ---
	matTrunk := material.NewStandard(&math32.Color{R: 0.36, G: 0.20, B: 0.09})
	matPine := material.NewStandard(&math32.Color{R: 0.06, G: 0.42, B: 0.14})
	matStar := material.NewStandard(&math32.Color{R: 1.0, G: 0.82, B: 0.12})
	matStar.SetShininess(90)

	treePos := Point3D{X: 3.8, Y: floorY, Z: 3.8}

	// Bruine stam (cilindervorm via rotatie rond Y)
	trunkLine := []Point2D{
		{X: 0.0, Y: 0.0},
		{X: 0.22, Y: 0.0},
		{X: 0.19, Y: 0.75},
		{X: 0.0, Y: 0.75},
	}
	trunk := NewShape(20, true, nil, matTrunk, trunkLine)
	w.AddShape(trunk, treePos)

	// Groene dennenboom-top (3 getrapte kegellagen via rotatie rond Y)
	treeTopLine := []Point2D{
		{X: 0.0, Y: 0.0},
		{X: 1.15, Y: 0.0},
		{X: 0.55, Y: 0.85},
		{X: 0.90, Y: 0.85},
		{X: 0.38, Y: 1.65},
		{X: 0.65, Y: 1.65},
		{X: 0.0, Y: 2.55},
	}
	treeTop := NewShape(24, true, nil, matPine, treeTopLine)
	w.AddShape(treeTop, treePos.Add(Point3D{Y: 0.65}))

	// Gouden piek/bol op de top van de kerstboom
	treeStar := NewShape(16, true, nil, matStar, NewLineHalfCircle(0.16, 12))
	w.AddShape(treeStar, treePos.Add(Point3D{Y: 0.65 + 2.50}))

	// --- 2. Kubussen & Balken (pure verschuiving, Rotate = false) ---
	matSteelBlue := material.NewStandard(&math32.Color{R: 0.18, G: 0.42, B: 0.68})
	matSteelBlue.SetShininess(60)
	cube := NewShape(1, false, []float32{1.0}, matSteelBlue, NewLineRect(1.0, 1.0))
	w.AddShape(cube, Point3D{X: -3.5, Y: floorY, Z: 3.2})

	matTerracotta := material.NewStandard(&math32.Color{R: 0.76, G: 0.30, B: 0.20})
	block := NewShape(2, false, []float32{0.35, 0.55}, matTerracotta, NewLineRect(0.8, 1.8))
	w.AddShape(block, Point3D{X: 4.5, Y: floorY, Z: -2.8})

	// --- 3. Bollen (halve cirkel met pure rotatie, ShiftDist = nil) ---
	matGold := material.NewStandard(&math32.Color{R: 0.88, G: 0.68, B: 0.15})
	matGold.SetShininess(100)
	goldSphere := NewShape(28, true, nil, matGold, NewLineHalfCircle(0.65, 20))
	w.AddShape(goldSphere, Point3D{X: -2.8, Y: floorY, Z: -3.5})

	matCrimson := material.NewStandard(&math32.Color{R: 0.78, G: 0.12, B: 0.24})
	matCrimson.SetShininess(75)
	crimsonSphere := NewShape(24, true, nil, matCrimson, NewLineHalfCircle(0.45, 18))
	w.AddShape(crimsonSphere, Point3D{X: -4.8, Y: floorY, Z: 0.5})

	// --- 4. Torussen (cirkel met gecombineerde rotatie + verschuiving) ---
	matViolet := material.NewStandard(&math32.Color{R: 0.48, G: 0.22, B: 0.72})
	matViolet.SetShininess(80)
	flatTorus := NewShape(32, true, []float32{0.18}, matViolet, NewLineCircle(0.24, 18))
	w.AddShape(flatTorus, Point3D{X: 2.2, Y: floorY + 0.24, Z: -4.2})

	matCyan := material.NewStandard(&math32.Color{R: 0.12, G: 0.72, B: 0.72})
	matCyan.SetShininess(85)
	archTorus := NewShape(28, true, []float32{0.20}, matCyan, NewLineCircle(0.20, 16))
	SetNodeRotation(archTorus, NewQuatFromAxisAngle(Point3D{X: 1}, math.Pi/2))
	w.AddShape(archTorus, Point3D{X: 0.0, Y: floorY + 1.09, Z: 6.0})

	// --- 5. Gecanneleerde zuil met herhalende lijnen (line[0], line[1], line[0], ...) ---
	matMarble := material.NewStandard(&math32.Color{R: 0.82, G: 0.80, B: 0.76})
	matMarble.SetShininess(50)
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
	flutedColumn := NewShape(32, true, nil, matMarble, fluteOuter, fluteInner)
	w.AddShape(flutedColumn, Point3D{X: 0.0, Y: floorY, Z: -5.5})

	// --- 6. Pilaren met variabele shiftDist []float32 (vierkant onder/boven, rond in het midden) ---
	// Dankzij []float32 hoeft de lange ronde schacht maar 1 stap te zijn (geen N herhaalde ringlijnen),
	// en hoeven de hoogtes van voet, schuine overgang, schacht en kapiteel geen veelvouden te zijn.

	// A. Klassieke zandstenen pilaren: 5 stappen met ongelijke afstanden [0.26, 0.14, 2.15, 0.14, 0.26]
	matSandstone := material.NewStandard(&math32.Color{R: 0.84, G: 0.74, B: 0.56})
	matSandstone.SetShininess(45)

	sqCap := centeredSquareLoop(0.46)
	roundShaft := alignedCircleLoop(0.28, 24)
	classicShifts := []float32{0.26, 0.14, 2.15, 0.14, 0.26}
	classicProfiles := [][]Point2D{sqCap, sqCap, roundShaft, roundShaft, sqCap, sqCap}

	w.addUprightShiftedPillar(-2.2, 5.5, 0.46, matSandstone, classicShifts, classicProfiles...)
	w.addUprightShiftedPillar(2.2, 5.5, 0.46, matSandstone, classicShifts, classicProfiles...)

	// B. Getrapte granieten zuil met brede plint en brede kroon: 7 stappen [0.22, 0.13, 0.09, 2.45, 0.09, 0.13, 0.22]
	matGranite := material.NewStandard(&math32.Color{R: 0.42, G: 0.45, B: 0.48})
	matGranite.SetShininess(70)

	widePlinth := centeredSquareLoop(0.58)
	midNeck := centeredSquareLoop(0.42)
	slimShaft := alignedCircleLoop(0.26, 24)
	tieredShifts := []float32{0.22, 0.13, 0.09, 2.45, 0.09, 0.13, 0.22}
	tieredProfiles := [][]Point2D{
		widePlinth, widePlinth, midNeck,
		slimShaft, slimShaft, // Eén enkele stap van 2.45 voor de hele ronde middenschacht
		midNeck, widePlinth, widePlinth,
	}

	w.addUprightShiftedPillar(5.8, 0.8, 0.58, matGranite, tieredShifts, tieredProfiles...)

	// C. Bronzen modulus-pilaar: 3 afstanden [0.17, 0.61, 0.13] herhaald via modulus over 9 stappen
	matBronze := material.NewStandard(&math32.Color{R: 0.62, G: 0.44, B: 0.24})
	matBronze.SetShininess(85)

	sqEnd := centeredSquareLoop(0.48)
	roundWide := alignedCircleLoop(0.34, 24)
	roundNarrow := alignedCircleLoop(0.24, 24)
	modulusShifts := []float32{0.17, 0.61, 0.13} // Cycled via step % 3 over 9 stappen
	modulusProfiles := [][]Point2D{
		sqEnd, sqEnd,
		roundNarrow, roundWide, roundWide, roundNarrow, roundNarrow, roundWide,
		sqEnd, sqEnd,
	}

	w.addUprightShiftedPillar(-5.8, -2.2, 0.48, matBronze, modulusShifts, modulusProfiles...)
}
