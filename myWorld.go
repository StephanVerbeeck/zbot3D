package main

// BuildMaze constructs a navigable brick-walled maze in the world using the
// four brick bitmap textures (brick 01 .. brick 04) and World's generic wall builder.
func (w *World) BuildMaze() {
	const (
		cols          = 6
		rows          = 6
		cellSize      = float32(4.0)
		wallThickness = float32(0.45)
		wallHeight    = float32(1.85)
		tileUnit      = float32(2.0)
	)

	originX := float32(9.0)
	originZ := -float32(rows) * cellSize * 0.5
	halfThick := wallThickness * 0.5

	hWalls := [rows + 1][cols]bool{
		{true, true, true, true, true, true},     // Z = 0 (North outer wall)
		{false, true, true, false, true, false},  // Z = 1
		{true, false, true, true, false, true},   // Z = 2
		{false, true, false, false, true, false}, // Z = 3
		{true, false, true, true, false, false},  // Z = 4
		{false, true, false, true, false, true},  // Z = 5
		{true, true, true, true, true, true},     // Z = 6 (South outer wall)
	}

	vWalls := [rows][cols + 1]bool{
		{true, false, false, true, false, false, true}, // Row 0
		{true, true, false, false, true, false, true},  // Row 1
		{true, false, true, false, false, true, false}, // Row 2 (East exit open at col 6)
		{false, true, false, true, false, true, true},  // Row 3 (West entrance open at col 0)
		{true, false, true, false, true, false, true},  // Row 4
		{true, false, false, true, false, false, true}, // Row 5
	}

	wallIdx := 0

	// Horizontal wall runs along X
	for r := 0; r <= rows; r++ {
		c := 0
		for c < cols {
			if !hWalls[r][c] {
				c++
				continue
			}
			startC := c
			for c < cols && hWalls[r][c] {
				c++
			}
			lengthX := float32(c-startC)*cellSize + wallThickness
			x0 := originX + float32(startC)*cellSize - halfThick
			z0 := originZ + float32(r)*cellSize - halfThick

			mat := MatBrickByIndex(wallIdx, lengthX/tileUnit, wallHeight/tileUnit)
			wallIdx++

			w.AddWallBox(x0, floorY, z0, lengthX, wallHeight, wallThickness, mat)
		}
	}

	// Vertical wall runs along Z
	for c := 0; c <= cols; c++ {
		r := 0
		for r < rows {
			if !vWalls[r][c] {
				r++
				continue
			}
			startR := r
			for r < rows && vWalls[r][c] {
				r++
			}
			lengthZ := float32(r-startR)*cellSize + wallThickness
			x0 := originX + float32(c)*cellSize - halfThick
			z0 := originZ + float32(startR)*cellSize - halfThick

			mat := MatBrickByIndex(wallIdx, lengthZ/tileUnit, wallHeight/tileUnit)
			wallIdx++

			w.AddWallBox(x0, floorY, z0, wallThickness, wallHeight, lengthZ, mat)
		}
	}
}
