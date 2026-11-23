package main

import (
	"github.com/g3n/engine/material"
)

// --- Project-Specific Solid & Metallic Materials ---

func MatGoldMetal() *material.Standard {
	return NewMetallicMaterial(RGB(0.88, 0.68, 0.15), RGB(1.0, 0.92, 0.55), 110)
}

func MatSteelBlueMetal() *material.Standard {
	return NewMetallicMaterial(RGB(0.18, 0.42, 0.68), RGB(0.65, 0.80, 0.95), 85)
}

func MatBronzeMetal() *material.Standard {
	return NewMetallicMaterial(RGB(0.62, 0.44, 0.24), RGB(0.92, 0.76, 0.52), 90)
}

func MatCrimsonGloss() *material.Standard {
	return NewSolidMaterial(RGB(0.78, 0.12, 0.24), 75)
}

func MatVioletGloss() *material.Standard {
	return NewSolidMaterial(RGB(0.48, 0.22, 0.72), 80)
}

func MatCyanGloss() *material.Standard {
	return NewSolidMaterial(RGB(0.12, 0.72, 0.72), 85)
}

func MatTerracotta() *material.Standard {
	return NewSolidMaterial(RGB(0.76, 0.30, 0.20), 25)
}

func MatPineFoliage() *material.Standard {
	return NewSolidMaterial(RGB(0.06, 0.42, 0.14), 20)
}

// --- Brick Image File Materials (material\brick 01..04) ---

func MatBrick01(repeatX, repeatY float32) *material.Standard {
	return NewBitmapMaterialFromFile("material/brick 01.jpg", BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 20,
	})
}

func MatBrick02(repeatX, repeatY float32) *material.Standard {
	return NewBitmapMaterialFromFiles([]string{"material/brick 02.jpg", "material/brick 02jpg"}, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 20,
	})
}

func MatBrick03(repeatX, repeatY float32) *material.Standard {
	return NewBitmapMaterialFromFile("material/brick 03.jpg", BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 20,
	})
}

func MatBrick04(repeatX, repeatY float32) *material.Standard {
	return NewBitmapMaterialFromFile("material/brick 04.jpg", BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 20,
	})
}

// MatBrickByIndex returns one of the four brick bitmap materials (0..3) with the given UV repeat.
func MatBrickByIndex(index int, repeatX, repeatY float32) *material.Standard {
	switch ((index % 4) + 4) % 4 {
	case 0:
		return MatBrick01(repeatX, repeatY)
	case 1:
		return MatBrick02(repeatX, repeatY)
	case 2:
		return MatBrick03(repeatX, repeatY)
	default:
		return MatBrick04(repeatX, repeatY)
	}
}

// --- Project-Specific Procedural Bitmap Materials ---

func MatBrickWall(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateBrickBitmap(
		256, 256, 8, 4,
		RGBA8(158, 56, 42, 255),
		RGBA8(118, 42, 34, 255),
		RGBA8(205, 200, 190, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 18,
	})
}

func MatTileRoof(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateRoofTileBitmap(
		256, 256, 6, 8,
		RGBA8(192, 78, 44, 255),
		RGBA8(95, 34, 20, 255),
		RGBA8(232, 126, 84, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 35,
	})
}

func MatSlateRoof(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateRoofTileBitmap(
		256, 256, 8, 6,
		RGBA8(68, 76, 86, 255),
		RGBA8(32, 36, 42, 255),
		RGBA8(112, 122, 134, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 45,
	})
}

func MatTreeBark(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateWoodBitmap(
		128, 128,
		RGBA8(96, 54, 26, 255),
		RGBA8(52, 28, 12, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 12,
	})
}

func MatSandstone(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateStoneBitmap(
		256, 256,
		RGBA8(216, 190, 144, 255),
		RGBA8(176, 146, 98, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 35,
	})
}

func MatGranite(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateStoneBitmap(
		256, 256,
		RGBA8(112, 118, 126, 255),
		RGBA8(64, 68, 74, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 65,
	})
}

func MatMarble(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateStoneBitmap(
		256, 256,
		RGBA8(226, 222, 214, 255),
		RGBA8(165, 162, 158, 255),
	)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 55,
	})
}

func MatBrushedSteel(repeatX, repeatY float32) *material.Standard {
	bmp := GenerateBrushedMetalBitmap(
		256, 256,
		RGBA8(175, 182, 190, 255),
	)
	spec := RGB(0.9, 0.92, 0.96)
	return NewBitmapMaterialFromRGBA(bmp, BitmapOptions{
		RepeatX:   repeatX,
		RepeatY:   repeatY,
		Shininess: 95,
		Specular:  &spec,
	})
}
