package main

import (
	"math"
	"time"

	"github.com/g3n/engine/app"
	"github.com/g3n/engine/camera"
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/gls"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/light"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/renderer"
	"github.com/g3n/engine/window"
)

type ZBotApp struct {
	app        *app.Application
	scene      *core.Node
	chassis    *graphic.Mesh
	centralArm *core.Node
	sideArmL   *core.Node
	sideArmR   *core.Node
	mode       int // 0: Ingeklapt, 1: SUV/Werkplatform, 2: Volledig Uitgeklapt, 3: Hand-locomotie
}

func main() {
	// 1. Initialiseer de G3N Applicatie
	application := app.App(800, 600, "Z-Bot 3D")
	scene := core.NewNode()

	// 2. Camera en belichting
	width, height := application.GetSize()
	cam := camera.New(float32(width) / float32(height))
	cam.SetFov(60)
	cam.SetPosition(0, 3, 6)
	cam.LookAt(&math32.Vector3{X: 0, Y: 1, Z: 0}, &math32.Vector3{X: 0, Y: 1, Z: 0})
	scene.Add(cam)

	// Zorg dat viewport en camera-aspectratio meeschalen bij resizen
	onResize := func(evname string, ev interface{}) {
		w, h := application.GetSize()
		application.Gls().Viewport(0, 0, int32(w), int32(h))
		cam.SetAspect(float32(w) / float32(h))
	}
	application.Subscribe(window.OnWindowSize, onResize)
	onResize("", nil)

	// Optioneel: camera.NewOrbitControl(cam) kan hier gekoppeld worden voor muisbesturing
	ambLight := light.NewAmbient(&math32.Color{R: 0.3, G: 0.3, B: 0.3}, 1.0)
	scene.Add(ambLight)
	dirLight := light.NewDirectional(&math32.Color{R: 0.8, G: 0.8, B: 0.8}, 1.0)
	dirLight.SetPosition(5, 10, 7)
	scene.Add(dirLight)

	zApp := &ZBotApp{
		app:   application,
		scene: scene,
		mode:  0,
	}

	// 3. Bouw de Z-Bot hiërarchie
	zApp.BuildZBot()

	// 4. Toetsenbord-interactie voor modi
	application.Subscribe(window.OnKeyUp, func(evname string, ev interface{}) {
		kev := ev.(*window.KeyEvent)
		switch kev.Key {
		case window.Key1:
			zApp.setMode(0) // Ingeklapt (Nesting Cassette)
		case window.Key2:
			zApp.setMode(1) // SUV / Werkplatform Mode
		case window.Key3:
			zApp.setMode(2) // Volledig Uitgeklapt Bimanual Mode
		case window.Key4:
			zApp.setMode(3) // Hand-locomotie Mode
		}
	})

	// 5. Render loop / update
	application.Run(func(rend *renderer.Renderer, deltaTime time.Duration) {
		application.Gls().Clear(gls.DEPTH_BUFFER_BIT | gls.COLOR_BUFFER_BIT)
		rend.Render(scene, cam)
	})
}

func (z *ZBotApp) BuildZBot() {
	// Materialen
	matChassis := material.NewStandard(&math32.Color{R: 0.15, G: 0.15, B: 0.18}) // Donker carbon/metaal
	matArm := material.NewStandard(&math32.Color{R: 0.2, G: 0.6, B: 0.8})        // Industrieel blauw/teal
	matSensor := material.NewStandard(&math32.Color{R: 0.9, G: 0.5, B: 0.1})     // Accent oranje/geel

	// --- A. Basis Chassis (Skateboard Bakje) ---
	chassisGeom := geometry.NewBox(1.2, 0.3, 2.0)
	z.chassis = graphic.NewMesh(chassisGeom, matChassis)
	z.chassis.SetPosition(0, 0.15, 0)
	z.scene.Add(z.chassis)

	// Wielen (4 stuks omnidirectioneel)
	wheelGeom := geometry.NewCylinder(0.15, 0.1, 16, 1, true, true)
	matWheel := material.NewStandard(&math32.Color{R: 0.1, G: 0.1, B: 0.1})

	wheelPositions := [4][3]float32{
		{-0.65, 0.0, 0.8},  // Links-voor
		{0.65, 0.0, 0.8},   // Rechts-voor
		{-0.65, 0.0, -0.8}, // Links-achter
		{0.65, 0.0, -0.8},  // Rechts-achter
	}
	for _, pos := range wheelPositions {
		wheel := graphic.NewMesh(wheelGeom, matWheel)
		wheel.SetPosition(pos[0], pos[1], pos[2])
		wheel.SetRotation(0, 0, math.Pi/2)
		z.scene.Add(wheel)
	}

	// --- B. Centrale Stam (Heftank / Hefkolom) ---
	z.centralArm = core.NewNode()
	centralGeom := geometry.NewBox(0.3, 1.2, 0.3)
	centralMesh := graphic.NewMesh(centralGeom, matArm)
	centralMesh.SetPosition(0, 0.6, 0)
	z.centralArm.Add(centralMesh)

	// Sensor Head op de top van de centrale stam
	sensorGeom := geometry.NewBox(0.25, 0.2, 0.25)
	sensorMesh := graphic.NewMesh(sensorGeom, matSensor)
	sensorMesh.SetPosition(0, 1.3, 0)
	z.centralArm.Add(sensorMesh)

	z.scene.Add(z.centralArm)

	// --- C. Twee Zijarmen (Montage aan de top van de centrale arm) ---
	z.sideArmL = core.NewNode()
	sideGeomL := geometry.NewBox(0.2, 1.0, 0.2)
	sideMeshL := graphic.NewMesh(sideGeomL, matArm)
	sideMeshL.SetPosition(-0.3, 0.5, 0)
	z.sideArmL.Add(sideMeshL)
	z.centralArm.Add(z.sideArmL)

	z.sideArmR = core.NewNode()
	sideGeomR := geometry.NewBox(0.2, 1.0, 0.2)
	sideMeshR := graphic.NewMesh(sideGeomR, matArm)
	sideMeshR.SetPosition(0.3, 0.5, 0)
	z.sideArmR.Add(sideMeshR)
	z.centralArm.Add(z.sideArmR)

	// Start in modus 0 (Ingeklapt)
	z.setMode(0)
}

func (z *ZBotApp) setMode(newMode int) {
	z.mode = newMode
	switch z.mode {
	case 0: // Ingeklapt (Nesting in bakje)
		z.centralArm.SetPosition(0, 0, 0)
		z.centralArm.SetRotation(0, 0, 0)
		z.sideArmL.SetPosition(0, 0, 0)
		z.sideArmR.SetPosition(0, 0, 0)
	case 1: // SUV / Werkplatform Mode
		z.centralArm.SetPosition(0, 0.3, -0.5)
		z.centralArm.SetRotation(0.3, 0, 0)
		z.sideArmL.SetPosition(-0.4, 0.8, 0)
		z.sideArmR.SetPosition(0.4, 0.8, 0)
	case 2: // Volledig Uitgeklapt Bimanual Mode
		z.centralArm.SetPosition(0, 0.5, 0)
		z.centralArm.SetRotation(0, 0, 0)
		z.sideArmL.SetPosition(-0.8, 1.2, 0.2)
		z.sideArmL.SetRotation(0, 0, -0.5)
		z.sideArmR.SetPosition(0.8, 1.2, 0.2)
		z.sideArmR.SetRotation(0, 0, 0.5)
	case 3: // Hand-locomotie Mode (op de handen lopen)
		z.centralArm.SetPosition(0, 1.0, 0)
		z.centralArm.SetRotation(-0.5, 0, 0)
		z.sideArmL.SetPosition(-0.6, 0, 1.5)
		z.sideArmR.SetPosition(0.6, 0, 1.5)
	}
}
