package main

import (
	"image"
	"image/color"
	"math"

	"github.com/g3n/engine/gls"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/texture"
)

// MaterialOptions configures a general solid or metallic standard material.
type MaterialOptions struct {
	Diffuse     math32.Color
	Ambient     *math32.Color
	Specular    *math32.Color
	Emissive    *math32.Color
	Shininess   float32
	Opacity     float32 // 0 defaults to 1.0 (fully opaque)
	DoubleSided bool
}

// BitmapOptions configures how a bitmap texture is wrapped, repeated, and shaded on a material.
type BitmapOptions struct {
	RepeatX   float32
	RepeatY   float32
	Shininess float32
	Tint      *math32.Color
	Specular  *math32.Color
}

// RGB constructs a math32.Color from 0..1 float components.
func RGB(r, g, b float32) math32.Color {
	return math32.Color{R: r, G: g, B: b}
}

// RGBA8 constructs a standard library 8-bit color.RGBA.
func RGBA8(r, g, b, a uint8) color.RGBA {
	return color.RGBA{R: r, G: g, B: b, A: a}
}

// NewSolidMaterial creates a standard shaded solid-color material.
func NewSolidMaterial(diffuse math32.Color, shininess float32) *material.Standard {
	return NewSolidMaterialWithOptions(MaterialOptions{
		Diffuse:   diffuse,
		Shininess: shininess,
		Opacity:   1.0,
	})
}

// NewSolidMaterialWithOptions creates a standard material using full shading options.
func NewSolidMaterialWithOptions(opts MaterialOptions) *material.Standard {
	mat := material.NewStandard(&opts.Diffuse)
	if opts.Ambient != nil {
		mat.SetAmbientColor(opts.Ambient)
	}
	if opts.Specular != nil {
		mat.SetSpecularColor(opts.Specular)
	}
	if opts.Emissive != nil {
		mat.SetEmissiveColor(opts.Emissive)
	}
	if opts.Shininess > 0 {
		mat.SetShininess(opts.Shininess)
	}
	opacity := opts.Opacity
	if opacity <= 0 {
		opacity = 1.0
	}
	mat.SetOpacity(opacity)
	if opts.DoubleSided {
		mat.SetSide(material.SideDouble)
	}
	return mat
}

// NewMetallicMaterial creates a high-specular metallic material with colored reflections.
func NewMetallicMaterial(baseColor, specularColor math32.Color, shininess float32) *material.Standard {
	ambient := RGB(baseColor.R*0.45, baseColor.G*0.45, baseColor.B*0.45)
	if shininess <= 0 {
		shininess = 95.0
	}
	return NewSolidMaterialWithOptions(MaterialOptions{
		Diffuse:   baseColor,
		Ambient:   &ambient,
		Specular:  &specularColor,
		Shininess: shininess,
		Opacity:   1.0,
	})
}

// NewEmissiveMaterial creates a self-lit solid material (useful for grid lines and wireframes).
func NewEmissiveMaterial(col math32.Color) *material.Standard {
	return NewSolidMaterialWithOptions(MaterialOptions{
		Diffuse:  col,
		Emissive: &col,
		Opacity:  1.0,
	})
}

// NewWireframeMaterialFrom derives a self-lit line material matching an existing material's color.
func NewWireframeMaterialFrom(src material.IMaterial) *material.Standard {
	lineColor := RGB(0.2, 0.85, 1.0)
	if stdMat, ok := src.(*material.Standard); ok {
		c := stdMat.AmbientColor()
		if c.R > 0 || c.G > 0 || c.B > 0 {
			lineColor = c
		}
	}
	return NewEmissiveMaterial(lineColor)
}

// NewBitmapMaterialFromRGBA builds a textured material from an in-memory *image.RGBA bitmap.
func NewBitmapMaterialFromRGBA(img *image.RGBA, opts BitmapOptions) *material.Standard {
	tint := RGB(1.0, 1.0, 1.0)
	if opts.Tint != nil {
		tint = *opts.Tint
	}
	mat := NewSolidMaterialWithOptions(MaterialOptions{
		Diffuse:   tint,
		Specular:  opts.Specular,
		Shininess: opts.Shininess,
		Opacity:   1.0,
	})

	tex := texture.NewTexture2DFromRGBA(img)
	tex.SetWrapS(gls.REPEAT)
	tex.SetWrapT(gls.REPEAT)
	rx := opts.RepeatX
	ry := opts.RepeatY
	if rx <= 0 {
		rx = 1.0
	}
	if ry <= 0 {
		ry = 1.0
	}
	tex.SetRepeat(rx, ry)
	mat.AddTexture(tex)
	return mat
}

// NewBitmapMaterialFromFile loads an image file from disk into a repeating bitmap material.
// If the file cannot be read, it falls back to a generated checkerboard bitmap so rendering continues cleanly.
func NewBitmapMaterialFromFile(path string, opts BitmapOptions) *material.Standard {
	tex, err := texture.NewTexture2DFromImage(path)
	if err != nil {
		fallback := GenerateCheckerBitmap(128, 128, 8, RGBA8(180, 180, 180, 255), RGBA8(90, 90, 90, 255))
		return NewBitmapMaterialFromRGBA(fallback, opts)
	}

	tint := RGB(1.0, 1.0, 1.0)
	if opts.Tint != nil {
		tint = *opts.Tint
	}
	mat := NewSolidMaterialWithOptions(MaterialOptions{
		Diffuse:   tint,
		Specular:  opts.Specular,
		Shininess: opts.Shininess,
		Opacity:   1.0,
	})

	tex.SetWrapS(gls.REPEAT)
	tex.SetWrapT(gls.REPEAT)
	rx := opts.RepeatX
	ry := opts.RepeatY
	if rx <= 0 {
		rx = 1.0
	}
	if ry <= 0 {
		ry = 1.0
	}
	tex.SetRepeat(rx, ry)
	mat.AddTexture(tex)
	return mat
}

// --- General Procedural Bitmap Generators ---

func clampByte(v int) uint8 {
	if v < 0 {
		return 0
	}
	if v > 255 {
		return 255
	}
	return uint8(v)
}

func lerpColor8(a, b color.RGBA, t float64) color.RGBA {
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	return color.RGBA{
		R: clampByte(int(float64(a.R) + (float64(b.R)-float64(a.R))*t)),
		G: clampByte(int(float64(a.G) + (float64(b.G)-float64(a.G))*t)),
		B: clampByte(int(float64(a.B) + (float64(b.B)-float64(a.B))*t)),
		A: 255,
	}
}

func hash2D(x, y int) uint32 {
	h := uint32(x*374761393 + y*668265263)
	h = (h ^ (h >> 13)) * 1274126177
	return h ^ (h >> 16)
}

// GenerateBrickBitmap creates a seamless running-bond brick wall bitmap with mortar lines and per-brick tone variation.
func GenerateBrickBitmap(width, height, rows, cols int, brickBase, brickAlt, mortar color.RGBA) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	if rows < 1 {
		rows = 8
	}
	if cols < 1 {
		cols = 4
	}

	rowH := float64(height) / float64(rows)
	colW := float64(width) / float64(cols)
	mortarPx := math.Max(2.0, math.Min(rowH, colW)*0.10)

	for y := 0; y < height; y++ {
		rIdx := int(float64(y) / rowH)
		localY := float64(y) - float64(rIdx)*rowH

		// Half-brick horizontal stagger on alternating courses
		shiftX := 0.0
		if rIdx%2 == 1 {
			shiftX = colW * 0.5
		}

		for x := 0; x < width; x++ {
			sx := math.Mod(float64(x)+shiftX, float64(width))
			cIdx := int(sx / colW)
			localX := sx - float64(cIdx)*colW

			// Mortar joint along bottom and right edge of each brick cell
			if localY < mortarPx || localX < mortarPx {
				n := int(hash2D(x, y)%19) - 9
				img.SetRGBA(x, y, color.RGBA{
					R: clampByte(int(mortar.R) + n),
					G: clampByte(int(mortar.G) + n),
					B: clampByte(int(mortar.B) + n),
					A: 255,
				})
				continue
			}

			// Per-brick shade variation + fine clay grain
			brickHash := hash2D(cIdx, rIdx)
			tone := float64(brickHash%100) / 100.0
			col := lerpColor8(brickBase, brickAlt, tone)
			grain := int(hash2D(x*3, y*3)%21) - 10

			img.SetRGBA(x, y, color.RGBA{
				R: clampByte(int(col.R) + grain),
				G: clampByte(int(col.G) + grain),
				B: clampByte(int(col.B) + grain),
				A: 255,
			})
		}
	}
	return img
}

// GenerateRoofTileBitmap creates a seamless overlapping barrel/pan roof tile bitmap.
func GenerateRoofTileBitmap(width, height, rows, cols int, tileBase, tileShadow, tileHighlight color.RGBA) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	if rows < 1 {
		rows = 6
	}
	if cols < 1 {
		cols = 8
	}

	rowH := float64(height) / float64(rows)
	colW := float64(width) / float64(cols)

	for y := 0; y < height; y++ {
		rIdx := int(float64(y) / rowH)
		v := (float64(y) - float64(rIdx)*rowH) / rowH // 0..1 from top to bottom of tile course

		for x := 0; x < width; x++ {
			cIdx := int(float64(x) / colW)
			u := (float64(x) - float64(cIdx)*colW) / colW // 0..1 across curved tile width

			// Barrel profile across u: bright crest in the middle, dark trough at edges
			arch := math.Sin(u * math.Pi)
			col := lerpColor8(tileShadow, tileBase, 0.35+0.65*arch)
			if arch > 0.72 {
				col = lerpColor8(col, tileHighlight, (arch-0.72)/0.28*0.45)
			}

			// Overlapping course lip shadow near v -> 1.0
			if v > 0.84 {
				lip := (v - 0.84) / 0.16
				col = lerpColor8(col, tileShadow, lip*0.75)
			}

			// Subtle per-tile weathering
			tileTint := int(hash2D(cIdx, rIdx)%17) - 8
			grain := int(hash2D(x, y)%11) - 5
			delta := tileTint + grain

			img.SetRGBA(x, y, color.RGBA{
				R: clampByte(int(col.R) + delta),
				G: clampByte(int(col.G) + delta),
				B: clampByte(int(col.B) + delta),
				A: 255,
			})
		}
	}
	return img
}

// GenerateWoodBitmap creates a vertical wood grain / bark bitmap.
func GenerateWoodBitmap(width, height int, baseCol, grainCol color.RGBA) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	for y := 0; y < height; y++ {
		fy := float64(y) / float64(height)
		for x := 0; x < width; x++ {
			fx := float64(x) / float64(width)
			wave := math.Sin(fx*42.0 + math.Sin(fy*2.0*math.Pi)*1.6)
			t := 0.5 + 0.5*wave
			col := lerpColor8(baseCol, grainCol, t*0.75)
			noise := int(hash2D(x, y/2)%15) - 7
			img.SetRGBA(x, y, color.RGBA{
				R: clampByte(int(col.R) + noise),
				G: clampByte(int(col.G) + noise),
				B: clampByte(int(col.B) + noise),
				A: 255,
			})
		}
	}
	return img
}

// GenerateStoneBitmap creates a natural speckled stone / granite / marble bitmap.
func GenerateStoneBitmap(width, height int, baseCol, speckCol color.RGBA) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	for y := 0; y < height; y++ {
		fy := float64(y) / float64(height)
		for x := 0; x < width; x++ {
			fx := float64(x) / float64(width)
			vein := 0.5 + 0.5*math.Sin((fx*3.0+fy*2.0)*2.0*math.Pi+math.Sin(fx*6.0*math.Pi)*0.8)
			col := lerpColor8(baseCol, speckCol, vein*0.35)

			h := hash2D(x/2, y/2) % 100
			if h < 14 {
				col = lerpColor8(col, speckCol, 0.55)
			} else if h > 90 {
				col = lerpColor8(col, RGBA8(245, 245, 245, 255), 0.30)
			}
			img.SetRGBA(x, y, col)
		}
	}
	return img
}

// GenerateBrushedMetalBitmap creates a fine directional brushed-metal bitmap.
func GenerateBrushedMetalBitmap(width, height int, baseCol color.RGBA) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	for y := 0; y < height; y++ {
		streak := int(hash2D(0, y)%25) - 12
		for x := 0; x < width; x++ {
			micro := int(hash2D(x/4, y)%9) - 4
			d := streak + micro
			img.SetRGBA(x, y, color.RGBA{
				R: clampByte(int(baseCol.R) + d),
				G: clampByte(int(baseCol.G) + d),
				B: clampByte(int(baseCol.B) + d),
				A: 255,
			})
		}
	}
	return img
}

// GenerateCheckerBitmap creates a two-tone checkerboard bitmap.
func GenerateCheckerBitmap(width, height, cells int, colA, colB color.RGBA) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, width, height))
	if cells < 1 {
		cells = 4
	}
	cellW := float64(width) / float64(cells)
	cellH := float64(height) / float64(cells)
	for y := 0; y < height; y++ {
		cy := int(float64(y) / cellH)
		for x := 0; x < width; x++ {
			cx := int(float64(x) / cellW)
			if (cx+cy)%2 == 0 {
				img.SetRGBA(x, y, colA)
			} else {
				img.SetRGBA(x, y, colB)
			}
		}
	}
	return img
}
