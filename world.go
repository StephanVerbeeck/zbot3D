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

// newPillarLines builds 4 repeating 2D profiles (0°, 22.5°, 45°, 67.5°) for a 16-step revolved pillar
// with a square base on the bottom, a round cylindrical shaft in the middle, and a square capital on top.
func newPillarLines(squareHalf, roundRadius, totalHeight, capHeight float32) [][]Point2D {
	bevelHeight := float32(0.10)
	if capHeight*2+bevelHeight*2 >= totalHeight {
		bevelHeight = totalHeight * 0.05
	}

	angles := [4]float32{
		0,
		math.Pi / 8,     // 22.5°
		math.Pi / 4,     // 45° (square corners)
		3 * math.Pi / 8, // 67.5°
	}

	lines := make([][]Point2D, 4)
	for idx, ang := range angles {
		cosOffset := math32.Cos(ang)
		if ang > math.Pi/4 {
			cosOffset = math32.Cos(math.Pi/2 - ang)
		}
		rSq := squareHalf / cosOffset

		if idx == 2 {
			// Corner profile (45°): 16 points; NewShape automatically extends the 8-point profiles to match
			midCap := capHeight * 0.5
			lines[idx] = []Point2D{
				{X: 0, Y: 0},
				{X: rSq * 0.5, Y: 0},
				{X: rSq, Y: 0},
				{X: rSq, Y: midCap},
				{X: rSq, Y: capHeight},
				{X: roundRadius, Y: capHeight + bevelHeight},
				{X: roundRadius, Y: totalHeight * 0.35},
				{X: roundRadius, Y: totalHeight * 0.50},
				{X: roundRadius, Y: totalHeight * 0.65},
				{X: roundRadius, Y: totalHeight - capHeight - bevelHeight},
				{X: rSq, Y: totalHeight - capHeight},
				{X: rSq, Y: totalHeight - midCap},
				{X: rSq, Y: totalHeight},
				{X: rSq * 0.5, Y: totalHeight},
				{X: 0, Y: totalHeight},
			}
		} else {
			// Flat-side profiles (0°, 22.5°, 67.5°): 8 points
			lines[idx] = []Point2D{
				{X: 0, Y: 0},
				{X: rSq, Y: 0},
				{X: rSq, Y: capHeight},
				{X: roundRadius, Y: capHeight + bevelHeight},
				{X: roundRadius, Y: totalHeight - capHeight - bevelHeight},
				{X: rSq, Y: totalHeight - capHeight},
				{X: rSq, Y: totalHeight},
				{X: 0, Y: totalHeight},
			}
		}
	}
	return lines
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
	trunk := NewShape(20, true, 0, matTrunk, trunkLine)
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
	treeTop := NewShape(24, true, 0, matPine, treeTopLine)
	w.AddShape(treeTop, treePos.Add(Point3D{Y: 0.65}))

	// Gouden piek/bol op de top van de kerstboom
	treeStar := NewShape(16, true, 0, matStar, NewLineHalfCircle(0.16, 12))
	w.AddShape(treeStar, treePos.Add(Point3D{Y: 0.65 + 2.50}))

	// --- 2. Kubussen & Balken (pure verschuiving, Rotate = false) ---
	matSteelBlue := material.NewStandard(&math32.Color{R: 0.18, G: 0.42, B: 0.68})
	matSteelBlue.SetShininess(60)
	cube := NewShape(1, false, 1.0, matSteelBlue, NewLineRect(1.0, 1.0))
	w.AddShape(cube, Point3D{X: -3.5, Y: floorY, Z: 3.2})

	matTerracotta := material.NewStandard(&math32.Color{R: 0.76, G: 0.30, B: 0.20})
	block := NewShape(2, false, 0.4, matTerracotta, NewLineRect(0.8, 1.8))
	w.AddShape(block, Point3D{X: 4.5, Y: floorY, Z: -2.8})

	// --- 3. Bollen (halve cirkel met pure rotatie, ShiftDist = 0) ---
	matGold := material.NewStandard(&math32.Color{R: 0.88, G: 0.68, B: 0.15})
	matGold.SetShininess(100)
	goldSphere := NewShape(28, true, 0, matGold, NewLineHalfCircle(0.65, 20))
	w.AddShape(goldSphere, Point3D{X: -2.8, Y: floorY, Z: -3.5})

	matCrimson := material.NewStandard(&math32.Color{R: 0.78, G: 0.12, B: 0.24})
	matCrimson.SetShininess(75)
	crimsonSphere := NewShape(24, true, 0, matCrimson, NewLineHalfCircle(0.45, 18))
	w.AddShape(crimsonSphere, Point3D{X: -4.8, Y: floorY, Z: 0.5})

	// --- 4. Torussen (cirkel met gecombineerde rotatie + verschuiving) ---
	matViolet := material.NewStandard(&math32.Color{R: 0.48, G: 0.22, B: 0.72})
	matViolet.SetShininess(80)
	flatTorus := NewShape(32, true, 0.18, matViolet, NewLineCircle(0.24, 18))
	w.AddShape(flatTorus, Point3D{X: 2.2, Y: floorY + 0.24, Z: -4.2})

	matCyan := material.NewStandard(&math32.Color{R: 0.12, G: 0.72, B: 0.72})
	matCyan.SetShininess(85)
	archTorus := NewShape(28, true, 0.20, matCyan, NewLineCircle(0.20, 16))
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
	flutedColumn := NewShape(32, true, 0, matMarble, fluteOuter, fluteInner)
	w.AddShape(flutedColumn, Point3D{X: 0.0, Y: floorY, Z: -5.5})

	// --- 6. Pilaren (vierkant onder en boven, rond in het midden) ---
	matSandstone := material.NewStandard(&math32.Color{R: 0.84, G: 0.74, B: 0.56})
	matSandstone.SetShininess(45)
	tallPillarLines := newPillarLines(0.45, 0.30, 2.6, 0.32)

	pillarLeft := NewShape(16, true, 0, matSandstone, tallPillarLines...)
	w.AddShape(pillarLeft, Point3D{X: -2.2, Y: floorY, Z: 5.5})

	pillarRight := NewShape(16, true, 0, matSandstone, tallPillarLines...)
	w.AddShape(pillarRight, Point3D{X: 2.2, Y: floorY, Z: 5.5})

	matGranite := material.NewStandard(&math32.Color{R: 0.42, G: 0.45, B: 0.48})
	matGranite.SetShininess(70)
	widePillarLines := newPillarLines(0.55, 0.36, 1.9, 0.30)

	pillarEast := NewShape(16, true, 0, matGranite, widePillarLines...)
	w.AddShape(pillarEast, Point3D{X: 5.8, Y: floorY, Z: 0.8})

	pillarWest := NewShape(16, true, 0, matGranite, widePillarLines...)
	w.AddShape(pillarWest, Point3D{X: -5.8, Y: floorY, Z: -2.2})
}
