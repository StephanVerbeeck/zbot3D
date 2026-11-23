package main

import (
	"fmt"
	"math"

	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/gls"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
)

// Point2D represents a coordinate in the 2D profile line (X = radial offset, Y = length axis).
type Point2D struct {
	X float32
	Y float32
}

// Point3D represents a 3D coordinate or vector (X, Y, Z).
type Point3D struct {
	X float32
	Y float32
	Z float32
}

func (p Point3D) Add(o Point3D) Point3D {
	return Point3D{X: p.X + o.X, Y: p.Y + o.Y, Z: p.Z + o.Z}
}

func (p Point3D) Sub(o Point3D) Point3D {
	return Point3D{X: p.X - o.X, Y: p.Y - o.Y, Z: p.Z - o.Z}
}

func (p Point3D) Scale(s float32) Point3D {
	return Point3D{X: p.X * s, Y: p.Y * s, Z: p.Z * s}
}

func (p Point3D) Dot(o Point3D) float32 {
	return p.X*o.X + p.Y*o.Y + p.Z*o.Z
}

func (p Point3D) Cross(o Point3D) Point3D {
	return Point3D{
		X: p.Y*o.Z - p.Z*o.Y,
		Y: p.Z*o.X - p.X*o.Z,
		Z: p.X*o.Y - p.Y*o.X,
	}
}

func (p Point3D) LengthSq() float32 {
	return p.X*p.X + p.Y*p.Y + p.Z*p.Z
}

func (p Point3D) Length() float32 {
	return math32.Sqrt(p.LengthSq())
}

func (p Point3D) Normalize() Point3D {
	l := p.Length()
	if l < 1e-12 {
		return Point3D{}
	}
	return p.Scale(1.0 / l)
}

func (p Point3D) ToMath32() *math32.Vector3 {
	return &math32.Vector3{X: p.X, Y: p.Y, Z: p.Z}
}

func Point3DFromMath32(v math32.Vector3) Point3D {
	return Point3D{X: v.X, Y: v.Y, Z: v.Z}
}

// SetNodePosition sets a G3N node's position from a Point3D.
func SetNodePosition(node interface{ SetPosition(float32, float32, float32) }, p Point3D) {
	node.SetPosition(p.X, p.Y, p.Z)
}

// GetNodePosition retrieves a G3N node's position as a Point3D.
func GetNodePosition(node interface{ Position() math32.Vector3 }) Point3D {
	return Point3DFromMath32(node.Position())
}

// ShapeInfo holds all parameters needed to call NewShape, plus a descriptive Name
// and a 3D post-construction scale applied in New().
type ShapeInfo struct {
	Name       string
	Material   material.IMaterial
	Steps      int
	Rotate     []float32
	Move       []float32
	Lines      [][]Point2D
	ShapeScale Point3D // 3D scale applied to the finished Shape in New() (unset {0,0,0} defaults to {1,1,1})
	hasScale3D bool
}

func (si ShapeInfo) effectiveScale3D() Point3D {
	if !si.hasScale3D && si.ShapeScale == (Point3D{}) {
		return Point3D{X: 1, Y: 1, Z: 1}
	}
	return si.ShapeScale
}

// New constructs a *Shape from the parameters stored in ShapeInfo and applies any 3D ShapeScale.
func (si ShapeInfo) New() *Shape {
	s := NewShape(si.Material, si.Steps, si.Rotate, si.Move, si.Lines...)
	sc := si.effectiveScale3D()
	if sc.X != 1.0 || sc.Y != 1.0 || sc.Z != 1.0 {
		s.SetScale(sc.X, sc.Y, sc.Z)
	}
	return s
}

// Scale scales s into si and returns si:
//   - x2, y2 scale the 2D line X and Y points
//   - z2 scales the Move distances
//   - x3, y3, z3 scale the finished constructed Shape before it is returned by New()
//
// All parameters are required; pass 1.0 for any scale factor that is not used.
func (si *ShapeInfo) Scale(s *ShapeInfo, x2, y2, z2, x3, y3, z3 float32) *ShapeInfo {
	if s == nil {
		panic("ShapeInfo.Scale: s must not be nil")
	}

	si.Name = s.Name
	si.Material = s.Material
	si.Steps = s.Steps

	if len(s.Rotate) > 0 {
		rot := make([]float32, len(s.Rotate))
		copy(rot, s.Rotate)
		si.Rotate = rot
	} else {
		si.Rotate = nil
	}

	if len(s.Move) > 0 {
		mv := make([]float32, len(s.Move))
		for i, m := range s.Move {
			mv[i] = m * z2
		}
		si.Move = mv
	} else {
		si.Move = nil
	}

	if len(s.Lines) > 0 {
		lines := make([][]Point2D, len(s.Lines))
		for i, ln := range s.Lines {
			lines[i] = LineScale(ln, x2, y2)
		}
		si.Lines = lines
	} else {
		si.Lines = nil
	}

	base3D := s.effectiveScale3D()
	si.ShapeScale = Point3D{
		X: base3D.X * x3,
		Y: base3D.Y * y3,
		Z: base3D.Z * z3,
	}
	si.hasScale3D = true

	return si
}

// Lerp linearly interpolates all fields between s1 and s2 at factor ratio (0.0 = s1, 1.0 = s2),
// writing the result into si and returning si.
// Any variable-length arrays (Rotate, Move, Lines, and each polyline in Lines) must have equal lengths in s1 and s2.
func (si *ShapeInfo) Lerp(s1, s2 *ShapeInfo, ratio float32) *ShapeInfo {
	if s1 == nil || s2 == nil {
		panic("ShapeInfo.Lerp: s1 and s2 must not be nil")
	}
	if len(s1.Rotate) != len(s2.Rotate) {
		panic(fmt.Sprintf("ShapeInfo.Lerp: Rotate length mismatch (%d != %d)", len(s1.Rotate), len(s2.Rotate)))
	}
	if len(s1.Move) != len(s2.Move) {
		panic(fmt.Sprintf("ShapeInfo.Lerp: Move length mismatch (%d != %d)", len(s1.Move), len(s2.Move)))
	}
	if len(s1.Lines) != len(s2.Lines) {
		panic(fmt.Sprintf("ShapeInfo.Lerp: Lines count mismatch (%d != %d)", len(s1.Lines), len(s2.Lines)))
	}
	for i := range s1.Lines {
		if len(s1.Lines[i]) != len(s2.Lines[i]) {
			panic(fmt.Sprintf("ShapeInfo.Lerp: Lines[%d] length mismatch (%d != %d)", i, len(s1.Lines[i]), len(s2.Lines[i])))
		}
	}

	if ratio <= 0.5 {
		si.Name = s1.Name
		si.Material = s1.Material
	} else {
		si.Name = s2.Name
		si.Material = s2.Material
	}

	si.Steps = int(math.Round(float64(s1.Steps) + float64(s2.Steps-s1.Steps)*float64(ratio)))

	if len(s1.Rotate) > 0 {
		si.Rotate = make([]float32, len(s1.Rotate))
		for i := range s1.Rotate {
			si.Rotate[i] = s1.Rotate[i] + (s2.Rotate[i]-s1.Rotate[i])*ratio
		}
	} else {
		si.Rotate = nil
	}

	if len(s1.Move) > 0 {
		si.Move = make([]float32, len(s1.Move))
		for i := range s1.Move {
			si.Move[i] = s1.Move[i] + (s2.Move[i]-s1.Move[i])*ratio
		}
	} else {
		si.Move = nil
	}

	if len(s1.Lines) > 0 {
		si.Lines = make([][]Point2D, len(s1.Lines))
		for i := range s1.Lines {
			line := make([]Point2D, len(s1.Lines[i]))
			for j := range s1.Lines[i] {
				p1 := s1.Lines[i][j]
				p2 := s2.Lines[i][j]
				line[j] = Point2D{
					X: p1.X + (p2.X-p1.X)*ratio,
					Y: p1.Y + (p2.Y-p1.Y)*ratio,
				}
			}
			si.Lines[i] = line
		}
	} else {
		si.Lines = nil
	}

	sc1 := s1.effectiveScale3D()
	sc2 := s2.effectiveScale3D()
	si.ShapeScale = Point3D{
		X: sc1.X + (sc2.X-sc1.X)*ratio,
		Y: sc1.Y + (sc2.Y-sc1.Y)*ratio,
		Z: sc1.Z + (sc2.Z-sc1.Z)*ratio,
	}
	si.hasScale3D = true

	return si
}

// Shape is a 3D closed object built by rotating and/or laterally moving 2D lines over N steps.
// When multiple lines are provided, step k selects lines[k % len(lines)].
// For each step s, rotation is rotate[s % len(rotate)] (in full turns 0..1) and lateral shift is move[s % len(move)].
type Shape struct {
	core.Node

	Lines     [][]Point2D        // One or more 2D XY polyline coordinates (cycled via step % len(Lines))
	Steps     int                // Number of rotation/move steps N
	Rotate    []float32          // Rotations per step in turns (0.0..1.0), cycled via step % len(Rotate)
	Move      []float32          // Lateral move distances per step, cycled via step % len(Move)
	CloseEnds bool               // Automatically close open ends into a solid volume
	Wireframe bool               // When true, display only as a mesh of lines (false by default)
	Material  material.IMaterial // Surface material for the closed solid shape

	solidMesh *graphic.Mesh
	wireLines *graphic.Lines
}

type triangle3D struct {
	a, b, c    Point3D
	ua, ub, uc Point2D
}

// NewShape creates a new 3D Shape positioned at (0, 0, 0) with Wireframe=false by default.
// `mat` sets the surface material (defaults to MatDefaultShape() if nil).
// For each step s, rotation (in 0.0..1.0 turns) is rotate[s % len(rotate)] and lateral move is move[s % len(move)].
// Elements beyond `steps` are not used. If both rotate and move are nil or empty, rotate defaults to 1.0/steps per step (1 full turn).
func NewShape(mat material.IMaterial, steps int, rotate []float32, move []float32, lines ...[]Point2D) *Shape {
	maxPts := 0
	for _, ln := range lines {
		if len(ln) > maxPts {
			maxPts = len(ln)
		}
	}

	extendedLines := make([][]Point2D, len(lines))
	for idx, ln := range lines {
		extendedLines[idx] = extendLineByRepeating(ln, maxPts)
	}

	var rotCopy []float32
	if len(rotate) > 0 {
		rotCopy = make([]float32, len(rotate))
		copy(rotCopy, rotate)
	}

	var moveCopy []float32
	if len(move) > 0 {
		moveCopy = make([]float32, len(move))
		copy(moveCopy, move)
	}

	s := &Shape{
		Lines:     extendedLines,
		Steps:     steps,
		Rotate:    rotCopy,
		Move:      moveCopy,
		CloseEnds: true,
		Wireframe: false,
		Material:  mat,
	}
	s.Init(s)
	SetNodePosition(s, Point3D{})
	s.Rebuild()
	return s
}

// extendLineByRepeating extends `line` to `targetLen` by repeating each point proportionally.
func extendLineByRepeating(line []Point2D, targetLen int) []Point2D {
	n := len(line)
	if n == 0 || targetLen <= 0 {
		return nil
	}
	if n >= targetLen {
		cp := make([]Point2D, n)
		copy(cp, line)
		return cp
	}

	out := make([]Point2D, 0, targetLen)
	for i := 0; i < n; i++ {
		repeatCount := ((i+1)*targetLen)/n - (i*targetLen)/n
		for r := 0; r < repeatCount; r++ {
			out = append(out, line[i])
		}
	}
	return out
}

// SetWireframe switches between rendering the closed solid material and rendering only a mesh of lines.
func (s *Shape) SetWireframe(wireframe bool) {
	s.Wireframe = wireframe
	if s.solidMesh != nil {
		s.solidMesh.SetVisible(!wireframe)
	}
	if s.wireLines != nil {
		s.wireLines.SetVisible(wireframe)
	}
}

// Rebuild generates the 3D vertices, UV coordinates, closed solid mesh, and wireframe lines.
func (s *Shape) Rebuild() {
	if s.solidMesh != nil {
		s.Remove(s.solidMesh)
		s.solidMesh = nil
	}
	if s.wireLines != nil {
		s.Remove(s.wireLines)
		s.wireLines = nil
	}

	if len(s.Lines) == 0 {
		return
	}

	var validLines [][]Point2D
	maxPts := 0
	for _, ln := range s.Lines {
		if len(ln) >= 2 {
			validLines = append(validLines, ln)
			if len(ln) > maxPts {
				maxPts = len(ln)
			}
		}
	}
	if len(validLines) == 0 || maxPts < 2 {
		return
	}

	normLines := make([][]Point2D, len(validLines))
	for idx, ln := range validLines {
		normLines[idx] = extendLineByRepeating(ln, maxPts)
	}

	steps := s.Steps
	if steps < 1 {
		steps = 1
	}

	// Clamp effective rotate and move arrays so parts larger than `steps` are not used
	effRotate := s.Rotate
	if len(effRotate) > steps {
		effRotate = effRotate[:steps]
	}
	effMove := s.Move
	if len(effMove) > steps {
		effMove = effMove[:steps]
	}

	// Default: if both rotate and move are nil or empty, rotate N steps for 1 full rotation (1.0 turn)
	if len(effRotate) == 0 && len(effMove) == 0 {
		effRotate = []float32{1.0 / float32(steps)}
	}

	hasRotate := false
	for _, r := range effRotate {
		if math32.Abs(r) > 1e-7 {
			hasRotate = true
			break
		}
	}

	hasMove := false
	for _, d := range effMove {
		if math32.Abs(d) > 1e-7 {
			hasMove = true
			break
		}
	}

	lengthAxis := Point3D{X: 0, Y: 1, Z: 0}
	baseShiftDir := Point3D{X: 0, Y: 0, Z: 1}

	rings := make([][]Point3D, steps+1)
	ringUVs := make([][]Point2D, steps+1)
	rotTurnsAt := make([]float32, steps+1)
	framePos := Point3D{}
	lineCount := len(normLines)
	rotCount := len(effRotate)
	moveCount := len(effMove)

	// Cumulative sweep progress for V texture coordinate
	sweepPos := make([]float32, steps+1)
	for k := 1; k <= steps; k++ {
		stepIdx := k - 1
		stepLen := float32(1.0) / float32(steps)
		if moveCount > 0 {
			d := math32.Abs(effMove[stepIdx%moveCount])
			if d > 1e-5 {
				stepLen = d
			}
		} else if rotCount > 0 {
			r := math32.Abs(effRotate[stepIdx%rotCount])
			if r > 1e-5 {
				stepLen = r
			}
		}
		sweepPos[k] = sweepPos[k-1] + stepLen
	}
	totalSweep := sweepPos[steps]
	if totalSweep < 1e-6 {
		totalSweep = 1.0
	}

	var cumRotTurns float32
	for k := 0; k <= steps; k++ {
		if k > 0 {
			stepIdx := k - 1
			if rotCount > 0 {
				cumRotTurns += effRotate[stepIdx%rotCount]
			}
		}
		rotTurnsAt[k] = cumRotTurns
		theta := cumRotTurns * 2.0 * math.Pi
		qStep := NewQuatFromAxisAngle(lengthAxis, theta)

		if k > 0 && moveCount > 0 {
			stepIdx := k - 1
			dist := effMove[stepIdx%moveCount]
			if dist != 0 {
				shiftDir := qStep.Rotate(baseShiftDir)
				framePos = framePos.Add(shiftDir.Scale(dist))
			}
		}

		lineIdx := k % lineCount
		isFullRotation := math32.Abs(math32.Abs(cumRotTurns)-1.0) < 1e-4
		if k == steps && isFullRotation && steps%lineCount != 0 {
			lineIdx = 0
		}
		activeLine := normLines[lineIdx]

		// Normalized arc-length U coordinate along the active 2D profile
		cumU := make([]float32, maxPts)
		for i := 1; i < maxPts; i++ {
			cumU[i] = cumU[i-1] + dist2D(activeLine[i-1], activeLine[i])
		}
		totalU := cumU[maxPts-1]
		if totalU < 1e-6 {
			totalU = 1.0
		}
		vCoord := sweepPos[k] / totalSweep

		ring := make([]Point3D, maxPts)
		uvs := make([]Point2D, maxPts)
		for i, pt := range activeLine {
			p0 := Point3D{X: pt.X, Y: pt.Y, Z: 0}
			pRot := qStep.Rotate(p0)
			ring[i] = framePos.Add(pRot)
			uvs[i] = Point2D{
				X: cumU[i] / totalU,
				Y: vCoord,
			}
		}
		rings[k] = ring
		ringUVs[k] = uvs
	}

	// Offset all points so the first coordinate of the 2D line at step 0 sits at (0, 0, 0)
	origin := rings[0][0]
	for k := 0; k <= steps; k++ {
		for i := 0; i < maxPts; i++ {
			rings[k][i] = rings[k][i].Sub(origin)
		}
	}

	// 1. Build wireframe lines
	linePositions := math32.NewArrayF32(0, 0)
	for k := 0; k <= steps; k++ {
		for i := 0; i < maxPts-1; i++ {
			p0 := rings[k][i]
			p1 := rings[k][i+1]
			linePositions.Append(p0.X, p0.Y, p0.Z, p1.X, p1.Y, p1.Z)
		}
	}
	for i := 0; i < maxPts; i++ {
		for k := 0; k < steps; k++ {
			p0 := rings[k][i]
			p1 := rings[k+1][i]
			linePositions.Append(p0.X, p0.Y, p0.Z, p1.X, p1.Y, p1.Z)
		}
	}

	lineGeom := geometry.NewGeometry()
	lineGeom.AddVBO(gls.NewVBO(linePositions).AddAttrib(gls.VertexPosition))

	lineMat := NewWireframeMaterialFrom(s.Material)
	s.wireLines = graphic.NewLines(lineGeom, lineMat)
	s.wireLines.SetVisible(s.Wireframe)
	s.Add(s.wireLines)

	// 2. Build closed solid triangle surface with UVs
	var tris []triangle3D
	addTri := func(a, b, c Point3D, ua, ub, uc Point2D) {
		e1 := b.Sub(a)
		e2 := c.Sub(a)
		if e1.Cross(e2).LengthSq() > 1e-12 {
			tris = append(tris, triangle3D{a: a, b: b, c: c, ua: ua, ub: ub, uc: uc})
		}
	}

	// Sweep quads between consecutive steps
	for k := 0; k < steps; k++ {
		for i := 0; i < maxPts-1; i++ {
			p00 := rings[k][i]
			p10 := rings[k+1][i]
			p11 := rings[k+1][i+1]
			p01 := rings[k][i+1]

			u00 := ringUVs[k][i]
			u10 := ringUVs[k+1][i]
			u11 := ringUVs[k+1][i+1]
			u01 := ringUVs[k][i+1]

			addTri(p00, p01, p11, u00, u01, u11)
			addTri(p00, p11, p10, u00, u11, u10)
		}
	}

	// Helper to compute planar 2D UVs for end-cap polygons
	capUVsForLine := func(ln []Point2D) []Point2D {
		minX, maxX := ln[0].X, ln[0].X
		minY, maxY := ln[0].Y, ln[0].Y
		for _, p := range ln {
			if p.X < minX {
				minX = p.X
			}
			if p.X > maxX {
				maxX = p.X
			}
			if p.Y < minY {
				minY = p.Y
			}
			if p.Y > maxY {
				maxY = p.Y
			}
		}
		spanX := maxX - minX
		if spanX < 1e-5 {
			spanX = 1.0
		}
		spanY := maxY - minY
		if spanY < 1e-5 {
			spanY = 1.0
		}
		out := make([]Point2D, len(ln))
		for i, p := range ln {
			out[i] = Point2D{
				X: (p.X - minX) / spanX,
				Y: (p.Y - minY) / spanY,
			}
		}
		return out
	}

	// Close open ends to form a closed 3D volume
	if s.CloseEnds {
		firstLine := normLines[0]
		endLine := normLines[steps%lineCount]
		profileClosed := dist2D(firstLine[0], firstLine[maxPts-1]) < 1e-5

		if !hasRotate {
			startUVs := capUVsForLine(firstLine)
			startIndices := triangulatePolygon2D(firstLine)
			for idx := 0; idx+2 < len(startIndices); idx += 3 {
				i0, i1, i2 := startIndices[idx], startIndices[idx+1], startIndices[idx+2]
				addTri(rings[0][i0], rings[0][i2], rings[0][i1], startUVs[i0], startUVs[i2], startUVs[i1])
			}

			endUVs := capUVsForLine(endLine)
			endIndices := triangulatePolygon2D(endLine)
			for idx := 0; idx+2 < len(endIndices); idx += 3 {
				i0, i1, i2 := endIndices[idx], endIndices[idx+1], endIndices[idx+2]
				addTri(rings[steps][i0], rings[steps][i1], rings[steps][i2], endUVs[i0], endUVs[i1], endUVs[i2])
			}
		} else if !hasMove {
			hasOpenStart := false
			for _, ln := range normLines {
				if math32.Abs(ln[0].X) > 1e-5 {
					hasOpenStart = true
					break
				}
			}
			if hasOpenStart {
				hubStart := Point3D{X: 0, Y: firstLine[0].Y, Z: 0}.Sub(origin)
				hubUV := Point2D{X: 0.5, Y: 0.5}
				for k := 0; k < steps; k++ {
					a0 := rotTurnsAt[k] * 2.0 * math.Pi
					a1 := rotTurnsAt[k+1] * 2.0 * math.Pi
					u0 := Point2D{X: 0.5 + 0.5*math32.Cos(a0), Y: 0.5 + 0.5*math32.Sin(a0)}
					u1 := Point2D{X: 0.5 + 0.5*math32.Cos(a1), Y: 0.5 + 0.5*math32.Sin(a1)}
					addTri(hubStart, rings[k+1][0], rings[k][0], hubUV, u1, u0)
				}
			}

			hasOpenEnd := false
			for _, ln := range normLines {
				if math32.Abs(ln[maxPts-1].X) > 1e-5 {
					hasOpenEnd = true
					break
				}
			}
			if hasOpenEnd {
				hubEnd := Point3D{X: 0, Y: firstLine[maxPts-1].Y, Z: 0}.Sub(origin)
				hubUV := Point2D{X: 0.5, Y: 0.5}
				for k := 0; k < steps; k++ {
					a0 := rotTurnsAt[k] * 2.0 * math.Pi
					a1 := rotTurnsAt[k+1] * 2.0 * math.Pi
					u0 := Point2D{X: 0.5 + 0.5*math32.Cos(a0), Y: 0.5 + 0.5*math32.Sin(a0)}
					u1 := Point2D{X: 0.5 + 0.5*math32.Cos(a1), Y: 0.5 + 0.5*math32.Sin(a1)}
					addTri(hubEnd, rings[k][maxPts-1], rings[k+1][maxPts-1], hubUV, u0, u1)
				}
			}
		} else if !profileClosed {
			for k := 0; k < steps; k++ {
				p00 := rings[k][maxPts-1]
				p10 := rings[k+1][maxPts-1]
				p11 := rings[k+1][0]
				p01 := rings[k][0]
				u00 := ringUVs[k][maxPts-1]
				u10 := ringUVs[k+1][maxPts-1]
				u11 := ringUVs[k+1][0]
				u01 := ringUVs[k][0]
				addTri(p00, p01, p11, u00, u01, u11)
				addTri(p00, p11, p10, u00, u11, u10)
			}
		} else if rings[0][0].Sub(rings[steps][0]).Length() > 1e-3 {
			startUVs := capUVsForLine(firstLine)
			startIndices := triangulatePolygon2D(firstLine)
			for idx := 0; idx+2 < len(startIndices); idx += 3 {
				i0, i1, i2 := startIndices[idx], startIndices[idx+1], startIndices[idx+2]
				addTri(rings[0][i0], rings[0][i2], rings[0][i1], startUVs[i0], startUVs[i2], startUVs[i1])
			}
			endUVs := capUVsForLine(endLine)
			endIndices := triangulatePolygon2D(endLine)
			for idx := 0; idx+2 < len(endIndices); idx += 3 {
				i0, i1, i2 := endIndices[idx], endIndices[idx+1], endIndices[idx+2]
				addTri(rings[steps][i0], rings[steps][i1], rings[steps][i2], endUVs[i0], endUVs[i1], endUVs[i2])
			}
		}
	}

	if len(tris) == 0 {
		return
	}

	// Guarantee outward-facing CCW winding by checking signed volume
	var signedVol6 float32
	for _, t := range tris {
		signedVol6 += t.a.Cross(t.b).Dot(t.c)
	}
	if signedVol6 < 0 {
		for i := range tris {
			tris[i].b, tris[i].c = tris[i].c, tris[i].b
			tris[i].ub, tris[i].uc = tris[i].uc, tris[i].ub
		}
	}

	// Compute face normals and crease-aware smooth vertex normals
	faceNormals := make([]Point3D, len(tris))
	type vKey struct{ x, y, z int32 }
	toKey := func(v Point3D) vKey {
		return vKey{
			x: int32(math.Round(float64(v.X) * 10000)),
			y: int32(math.Round(float64(v.Y) * 10000)),
			z: int32(math.Round(float64(v.Z) * 10000)),
		}
	}
	adjFaces := make(map[vKey][]int, len(tris)*3)

	for i, t := range tris {
		n := t.b.Sub(t.a).Cross(t.c.Sub(t.a)).Normalize()
		faceNormals[i] = n

		for _, v := range [3]Point3D{t.a, t.b, t.c} {
			k := toKey(v)
			adjFaces[k] = append(adjFaces[k], i)
		}
	}

	const smoothCosThreshold float32 = 0.64
	smoothNormal := func(v Point3D, baseIdx int) Point3D {
		baseN := faceNormals[baseIdx]
		sum := Point3D{}
		for _, fIdx := range adjFaces[toKey(v)] {
			fn := faceNormals[fIdx]
			if baseN.Dot(fn) >= smoothCosThreshold {
				sum = sum.Add(fn)
			}
		}
		if sum.LengthSq() > 1e-12 {
			return sum.Normalize()
		}
		return baseN
	}

	solidPositions := math32.NewArrayF32(0, len(tris)*9)
	solidNormals := math32.NewArrayF32(0, len(tris)*9)
	solidUVs := math32.NewArrayF32(0, len(tris)*6)

	for i, t := range tris {
		na := smoothNormal(t.a, i)
		nb := smoothNormal(t.b, i)
		nc := smoothNormal(t.c, i)

		solidPositions.Append(
			t.a.X, t.a.Y, t.a.Z,
			t.b.X, t.b.Y, t.b.Z,
			t.c.X, t.c.Y, t.c.Z,
		)
		solidNormals.Append(
			na.X, na.Y, na.Z,
			nb.X, nb.Y, nb.Z,
			nc.X, nc.Y, nc.Z,
		)
		solidUVs.Append(
			t.ua.X, t.ua.Y,
			t.ub.X, t.ub.Y,
			t.uc.X, t.uc.Y,
		)
	}

	solidGeom := geometry.NewGeometry()
	solidGeom.AddVBO(gls.NewVBO(solidPositions).AddAttrib(gls.VertexPosition))
	solidGeom.AddVBO(gls.NewVBO(solidNormals).AddAttrib(gls.VertexNormal))
	solidGeom.AddVBO(gls.NewVBO(solidUVs).AddAttrib(gls.VertexTexcoord))

	mat := s.Material
	if mat == nil {
		mat = MatDefaultShape()
	}

	s.solidMesh = graphic.NewMesh(solidGeom, mat)
	s.solidMesh.SetVisible(!s.Wireframe)
	s.Add(s.solidMesh)
}

// --- Helper 2D profile builders ---

// LineScale scales the X and Y coordinates of a 2D line by (x, y) and returns a new line.
func LineScale(line []Point2D, x, y float32) []Point2D {
	if len(line) == 0 {
		return nil
	}
	out := make([]Point2D, len(line))
	for i, pt := range line {
		out[i] = Point2D{
			X: pt.X * x,
			Y: pt.Y * y,
		}
	}
	return out
}

// LineCircle generates a 2D circular or elliptical arc polyline with `steps` segments
// from `fromRotation` to `toRotation` (expressed in turns, where 1.0 = one full 360° rotation),
// centered at `center` with X and Y radii `radius`.
func LineCircle(steps int, fromRotation, toRotation float32, center, radius Point2D) []Point2D {
	if steps < 1 {
		steps = 1
	}
	pts := make([]Point2D, steps+1)
	for i := 0; i <= steps; i++ {
		t := float32(i) / float32(steps)
		rot := fromRotation + (toRotation-fromRotation)*t
		theta := rot * 2.0 * math.Pi
		pts[i] = Point2D{
			X: center.X + radius.X*math32.Cos(theta),
			Y: center.Y + radius.Y*math32.Sin(theta),
		}
	}
	if math32.Abs(math32.Abs(toRotation-fromRotation)-1.0) < 1e-6 {
		pts[steps] = pts[0]
	}
	return pts
}

func NewLineHalfCircle(radius float32, segments int) []Point2D {
	if segments < 2 {
		segments = 2
	}
	pts := LineCircle(segments, -0.25, 0.25, Point2D{}, Point2D{X: radius, Y: radius})
	pts[0].X = 0
	pts[segments].X = 0
	return pts
}

func NewLineCircle(radius float32, segments int) []Point2D {
	if segments < 3 {
		segments = 3
	}
	return LineCircle(segments, 0.0, 1.0, Point2D{}, Point2D{X: radius, Y: radius})
}

func NewLineRect(width, height float32) []Point2D {
	return []Point2D{
		{X: 0, Y: 0},
		{X: width, Y: 0},
		{X: width, Y: height},
		{X: 0, Y: height},
		{X: 0, Y: 0},
	}
}

// NewLineCenteredSquare returns a closed 2D square centered at (0, 0), starting at (-halfSize, -halfSize).
func NewLineCenteredSquare(halfSize float32) []Point2D {
	return []Point2D{
		{X: -halfSize, Y: -halfSize},
		{X: halfSize, Y: -halfSize},
		{X: halfSize, Y: halfSize},
		{X: -halfSize, Y: halfSize},
		{X: -halfSize, Y: -halfSize},
	}
}

// --- 2D Geometry & Triangulation Helpers ---

func dist2D(a, b Point2D) float32 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math32.Sqrt(dx*dx + dy*dy)
}

func triangulatePolygon2D(poly []Point2D) []int {
	var indices []int
	for i := range poly {
		if len(indices) == 0 || dist2D(poly[indices[len(indices)-1]], poly[i]) >= 1e-5 {
			indices = append(indices, i)
		}
	}
	if len(indices) > 1 && dist2D(poly[indices[0]], poly[indices[len(indices)-1]]) < 1e-5 {
		indices = indices[:len(indices)-1]
	}

	n := len(indices)
	if n < 3 {
		return nil
	}

	var area float32
	for i := 0; i < n; i++ {
		p0 := poly[indices[i]]
		p1 := poly[indices[(i+1)%n]]
		area += p0.X*p1.Y - p1.X*p0.Y
	}
	ccw := area >= 0

	var result []int
	guard := 0
	for len(indices) > 3 && guard < n*n {
		guard++
		earFound := false
		m := len(indices)
		for i := 0; i < m; i++ {
			prev := indices[(i+m-1)%m]
			curr := indices[i]
			next := indices[(i+1)%m]

			a, b, c := poly[prev], poly[curr], poly[next]
			cross := (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)
			if (ccw && cross <= 1e-7) || (!ccw && cross >= -1e-7) {
				continue
			}

			containsOther := false
			for j := 0; j < m; j++ {
				idx := indices[j]
				if idx == prev || idx == curr || idx == next {
					continue
				}
				if pointInTri2D(poly[idx], a, b, c) {
					containsOther = true
					break
				}
			}
			if containsOther {
				continue
			}

			result = append(result, prev, curr, next)
			indices = append(indices[:i], indices[i+1:]...)
			earFound = true
			break
		}
		if !earFound {
			break
		}
	}
	if len(indices) == 3 {
		result = append(result, indices[0], indices[1], indices[2])
	}
	return result
}

func pointInTri2D(p, a, b, c Point2D) bool {
	d1 := (p.X-b.X)*(a.Y-b.Y) - (a.X-b.X)*(p.Y-b.Y)
	d2 := (p.X-c.X)*(b.Y-c.Y) - (b.X-c.X)*(p.Y-c.Y)
	d3 := (p.X-a.X)*(c.Y-a.Y) - (c.X-a.X)*(p.Y-a.Y)
	hasNeg := (d1 < -1e-6) || (d2 < -1e-6) || (d3 < -1e-6)
	hasPos := (d1 > 1e-6) || (d2 > 1e-6) || (d3 > 1e-6)
	return !(hasNeg && hasPos)
}