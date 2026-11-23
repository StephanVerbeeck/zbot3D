package main

import (
	"github.com/g3n/engine/material"
)

// --- Default & Environment Materials ---

func MatDefaultShape() *material.Standard {
	return NewSolidMaterial(RGB(0.70, 0.70, 0.75), 40)
}

func MatFloorGrid() *material.Standard {
	return NewEmissiveMaterial(RGB(0.0, 0.85, 0.20))
}

// --- Robot Solid & Metallic Materials ---

func MatRobotChassis() *material.Standard {
	return NewMetallicMaterial(RGB(0.15, 0.15, 0.18), RGB(0.45, 0.45, 0.52), 70)
}

func MatRobotArm() *material.Standard {
	return NewMetallicMaterial(RGB(0.20, 0.60, 0.80), RGB(0.60, 0.85, 0.95), 80)
}

func MatRobotSensor() *material.Standard {
	return NewSolidMaterial(RGB(0.90, 0.50, 0.10), 65)
}

func MatRobotWheel() *material.Standard {
	return NewSolidMaterial(RGB(0.10, 0.10, 0.10), 15)
}

func MatRobotSpoke() *material.Standard {
	return NewMetallicMaterial(RGB(0.85, 0.85, 0.88), RGB(1.0, 1.0, 1.0), 100)
}

// --- World Solid & Metallic Materials ---

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

// --- World Bitmap Materials ---

// MatBrickWall creates a classic red-brown running-bond brick wall bitmap material.
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

// MatTileRoof creates a terracotta overlapping barrel-tile roof bitmap material.
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

// MatSlateRoof creates a dark blue-grey overlapping slate roof tile bitmap material.
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

// MatTreeBark creates a brown textured wood bark bitmap material for tree trunks.
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

// MatSandstone creates a warm speckled sandstone bitmap material for pillars.
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

// MatGranite creates a speckled grey granite bitmap material for pillars.
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

// MatMarble creates a veined off-white marble bitmap material.
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

// MatBrushedSteel creates a brushed steel metallic bitmap material.
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
