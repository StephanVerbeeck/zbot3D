package main

import (
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
func SetNodePosition(node interface {
	SetPosition(float32, float32, float32)
}, p Point3D) {
	node.SetPosition(p.X, p.Y, p.Z)
}

// GetNodePosition retrieves a G3N node's position as a Point3D.
func GetNodePosition(node interface{ Position() math32.Vector3 }) Point3D {
	return Point3DFromMath32(node.Position())
}

// Shape is a 3D closed object built by rotating and/or laterally shifting 2D lines over N steps.
// When multiple lines are provided, step k selects lines[k % len(lines)].
// It embeds core.Node at (0, 0, 0) so it can be placed directly in a scene and attached to Mounts later.
type Shape struct {
	core.Node

	Lines     [][]Point2D        // One or more 2D XY polyline coordinates (cycled via step % len(Lines))
	Steps     int                // Number of rotation/shift steps N
	Rotate    bool               // Rotate 1/N of a full turn around the length axis per step
	ShiftDist float32            // Lateral shift distance per step in the current shift direction
	CloseEnds bool               // Automatically close open ends into a solid volume
	Wireframe bool               // When true, display only as a mesh of lines (false by default)
	Material  material.IMaterial // Surface material for the closed solid shape

	solidMesh *graphic.Mesh
	wireLines *graphic.Lines
}

type triangle3D struct {
	a, b, c Point3D
}

// NewShape creates a new 3D Shape positioned at (0, 0, 0) with Wireframe=false by default.
// One or more 2D lines can be passed at the end; shorter lines are extended to match the
// longest line by repeating each point, and during stepping step k uses lines[k % len(lines)].
func NewShape(steps int, rotate bool, shiftDist float32, mat material.IMaterial, lines ...[]Point2D) *Shape {
	// Determine which line has the most points
	maxPts := 0
	for _, ln := range lines {
		if len(ln) > maxPts {
			maxPts = len(ln)
		}
	}

	// Extend each shorter line to maxPts by repeating its points
	extendedLines := make([][]Point2D, len(lines))
	for idx, ln := range lines {
		extendedLines[idx] = extendLineByRepeating(ln, maxPts)
	}

	s := &Shape{
		Lines:     extendedLines,
		Steps:     steps,
		Rotate:    rotate,
		ShiftDist: shiftDist,
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
// For example, [p1, p2, p3, p4] with targetLen=8 becomes [p1, p1, p2, p2, p3, p3, p4, p4].
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

// Rebuild generates the 3D vertices, closed solid mesh, and wireframe lines from the Shape parameters.
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

	// Filter valid profiles (each needs at least 2 points) and ensure equal point counts
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

	var dTheta float32
	if s.Rotate {
		dTheta = (2.0 * math.Pi) / float32(steps)
	}

	lengthAxis := Point3D{X: 0, Y: 1, Z: 0}
	baseShiftDir := Point3D{X: 0, Y: 0, Z: 1}

	// Build (steps + 1) rings of 3D points starting at step 0
	rings := make([][]Point3D, steps+1)
	framePos := Point3D{}
	lineCount := len(normLines)

	for k := 0; k <= steps; k++ {
		theta := float32(k) * dTheta
		qStep := NewQuatFromAxisAngle(lengthAxis, theta)

		if k > 0 && s.ShiftDist != 0 {
			shiftDir := qStep.Rotate(baseShiftDir)
			framePos = framePos.Add(shiftDir.Scale(s.ShiftDist))
		}

		lineIdx := k % lineCount
		// Close full 360-degree rotation cleanly back onto line[0] at step N
		if k == steps && s.Rotate && steps%lineCount != 0 {
			lineIdx = 0
		}
		activeLine := normLines[lineIdx]

		ring := make([]Point3D, maxPts)
		for i, pt := range activeLine {
			// Convert 2D point to 3D at step zero, then rotate and shift
			p0 := Point3D{X: pt.X, Y: pt.Y, Z: 0}
			pRot := qStep.Rotate(p0)
			ring[i] = framePos.Add(pRot)
		}
		rings[k] = ring
	}

	// Offset all points so the first coordinate of the 2D line at step 0 sits at (0, 0, 0)
	origin := rings[0][0]
	for k := 0; k <= steps; k++ {
		for i := 0; i < maxPts; i++ {
			rings[k][i] = rings[k][i].Sub(origin)
		}
	}

	// 1. Build wireframe lines (profile loops + longitudinal sweep lines)
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

	lineColor := &math32.Color{R: 0.2, G: 0.85, B: 1.0}
	if stdMat, ok := s.Material.(*material.Standard); ok {
		c := stdMat.AmbientColor()
		if c.R > 0 || c.G > 0 || c.B > 0 {
			lineColor = &c
		}
	}
	lineMat := material.NewStandard(lineColor)
	lineMat.SetEmissiveColor(lineColor)

	s.wireLines = graphic.NewLines(lineGeom, lineMat)
	s.wireLines.SetVisible(s.Wireframe)
	s.Add(s.wireLines)

	// 2. Build closed solid triangle surface
	var tris []triangle3D
	addTri := func(a, b, c Point3D) {
		e1 := b.Sub(a)
		e2 := c.Sub(a)
		if e1.Cross(e2).LengthSq() > 1e-12 {
			tris = append(tris, triangle3D{a: a, b: b, c: c})
		}
	}

	// Sweep quads between consecutive steps
	for k := 0; k < steps; k++ {
		for i := 0; i < maxPts-1; i++ {
			p00 := rings[k][i]
			p10 := rings[k+1][i]
			p11 := rings[k+1][i+1]
			p01 := rings[k][i+1]
			addTri(p00, p01, p11)
			addTri(p00, p11, p10)
		}
	}

	// Close open ends to form a closed 3D volume
	if s.CloseEnds {
		firstLine := normLines[0]
		endLine := normLines[steps%lineCount]
		profileClosed := dist2D(firstLine[0], firstLine[maxPts-1]) < 1e-5

		if !s.Rotate {
			// Pure lateral shift (e.g. rectangular prism): cap front (k=0) and back (k=steps) faces
			startIndices := triangulatePolygon2D(firstLine)
			for idx := 0; idx+2 < len(startIndices); idx += 3 {
				i0, i1, i2 := startIndices[idx], startIndices[idx+1], startIndices[idx+2]
				addTri(rings[0][i0], rings[0][i2], rings[0][i1])
			}

			endIndices := triangulatePolygon2D(endLine)
			for idx := 0; idx+2 < len(endIndices); idx += 3 {
				i0, i1, i2 := endIndices[idx], endIndices[idx+1], endIndices[idx+2]
				addTri(rings[steps][i0], rings[steps][i1], rings[steps][i2])
			}
		} else if s.ShiftDist == 0 {
			// Pure rotation around Y-axis: cap open top/bottom edges to the rotation axis
			hasOpenStart := false
			for _, ln := range normLines {
				if math32.Abs(ln[0].X) > 1e-5 {
					hasOpenStart = true
					break
				}
			}
			if hasOpenStart {
				hubStart := Point3D{X: 0, Y: firstLine[0].Y, Z: 0}.Sub(origin)
				for k := 0; k < steps; k++ {
					addTri(hubStart, rings[k+1][0], rings[k][0])
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
				for k := 0; k < steps; k++ {
					addTri(hubEnd, rings[k][maxPts-1], rings[k+1][maxPts-1])
				}
			}
		} else if !profileClosed {
			// Combined rotation + shift with an open 2D profile: bridge last point to first point
			for k := 0; k < steps; k++ {
				p00 := rings[k][maxPts-1]
				p10 := rings[k+1][maxPts-1]
				p11 := rings[k+1][0]
				p01 := rings[k][0]
				addTri(p00, p01, p11)
				addTri(p00, p11, p10)
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

	const smoothCosThreshold float32 = 0.64 // ~50 deg crease angle
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
	}

	solidGeom := geometry.NewGeometry()
	solidGeom.AddVBO(gls.NewVBO(solidPositions).AddAttrib(gls.VertexPosition))
	solidGeom.AddVBO(gls.NewVBO(solidNormals).AddAttrib(gls.VertexNormal))

	mat := s.Material
	if mat == nil {
		mat = material.NewStandard(&math32.Color{R: 0.7, G: 0.7, B: 0.75})
	}

	s.solidMesh = graphic.NewMesh(solidGeom, mat)
	s.solidMesh.SetVisible(!s.Wireframe)
	s.Add(s.solidMesh)
}

// --- Helper 2D profile builders ---

// NewLineHalfCircle returns a 2D half-circle from (0, -radius) to (0, +radius).
// Rotating this line N times around its length axis produces a closed sphere.
func NewLineHalfCircle(radius float32, segments int) []Point2D {
	if segments < 2 {
		segments = 2
	}
	pts := make([]Point2D, segments+1)
	for i := 0; i <= segments; i++ {
		phi := -math.Pi/2 + (math.Pi*float32(i))/float32(segments)
		pts[i] = Point2D{
			X: radius * math32.Cos(phi),
			Y: radius * math32.Sin(phi),
		}
	}
	pts[0].X = 0
	pts[segments].X = 0
	return pts
}

// NewLineCircle returns a closed 2D circle in the XY plane.
// Combining rotation and lateral shift with this line produces a torus.
func NewLineCircle(radius float32, segments int) []Point2D {
	if segments < 3 {
		segments = 3
	}
	pts := make([]Point2D, segments+1)
	for i := 0; i <= segments; i++ {
		theta := (2.0 * math.Pi * float32(i)) / float32(segments)
		pts[i] = Point2D{
			X: radius * math32.Cos(theta),
			Y: radius * math32.Sin(theta),
		}
	}
	pts[segments] = pts[0]
	return pts
}

// NewLineRect returns a closed 2D rectangle starting at (0, 0).
// Shifting this line with Rotate=false produces a rectangular box.
func NewLineRect(width, height float32) []Point2D {
	return []Point2D{
		{X: 0, Y: 0},
		{X: width, Y: 0},
		{X: width, Y: height},
		{X: 0, Y: height},
		{X: 0, Y: 0},
	}
}

// --- 2D Geometry & Triangulation Helpers ---

func dist2D(a, b Point2D) float32 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math32.Sqrt(dx*dx + dy*dy)
}

func triangulatePolygon2D(poly []Point2D) []int {
	// Filter out consecutive duplicate vertices (including closing vertex) while preserving original indices
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
