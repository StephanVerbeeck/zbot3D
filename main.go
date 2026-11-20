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

const (
	floorY        = -0.15 // Bottom of the wheels (radius 0.15 centered at Y=0)
	gridCellSize  = 1.0   // Size of each square on the floor
	gridHalfCount = 100   // Number of squares in each direction from center
)

type ZBotApp struct {
	app        *app.Application
	scene      *core.Node
	robot      *core.Node
	floor      *graphic.Lines
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
	scene.Add(cam)

	onResize := func(evname string, ev interface{}) {
		w, h := application.GetSize()
		application.Gls().Viewport(0, 0, int32(w), int32(h))
		cam.SetAspect(float32(w) / float32(h))
	}
	application.Subscribe(window.OnWindowSize, onResize)
	onResize("", nil)

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

	// 3. Bouw de vloer en de Z-Bot hiërarchie
	zApp.BuildFloor()
	zApp.BuildZBot()

	// 4. Muisbesturing voor de camera
	var (
		camYaw       float32 = 0.0
		camPitch     float32 = 0.45
		camDistance  float32 = 6.5
		cameraFrozen         = false
		hasCursorPos         = false
		lastCursorX  float32
		lastCursorY  float32
	)

	updateCamera := func() {
		robotPos := zApp.robot.Position()
		target := math32.Vector3{
			X: robotPos.X,
			Y: robotPos.Y + 0.6, // Centrum van de robot
			Z: robotPos.Z,
		}

		cosPitch := math32.Cos(camPitch)
		camX := target.X + camDistance*cosPitch*math32.Sin(camYaw)
		camY := target.Y + camDistance*math32.Sin(camPitch)
		camZ := target.Z + camDistance*cosPitch*math32.Cos(camYaw)

		cam.SetPosition(camX, camY, camZ)
		cam.LookAt(&target, &math32.Vector3{X: 0, Y: 1, Z: 0})
	}
	updateCamera()

	// Linkermuisknop schakelt camera-bevriezing aan/uit
	application.Subscribe(window.OnMouseDown, func(evname string, ev interface{}) {
		mev := ev.(*window.MouseEvent)
		if mev.Button == window.MouseButtonLeft {
			cameraFrozen = !cameraFrozen
			if !cameraFrozen {
				// Reset cursor-anker zodat de camera niet verspringt bij hervatten
				hasCursorPos = false
			}
		}
	})

	// Muis X/Y draait de camera horizontaal en verticaal rond het centrum van de robot
	application.Subscribe(window.OnCursor, func(evname string, ev interface{}) {
		cev := ev.(*window.CursorEvent)
		if cameraFrozen {
			return
		}
		if !hasCursorPos {
			lastCursorX = cev.Xpos
			lastCursorY = cev.Ypos
			hasCursorPos = true
			return
		}

		dx := cev.Xpos - lastCursorX
		dy := cev.Ypos - lastCursorY
		lastCursorX = cev.Xpos
		lastCursorY = cev.Ypos

		const sensitivity float32 = 0.005
		camYaw -= dx * sensitivity
		camPitch += dy * sensitivity

		// Beperk verticale hoek zodat de camera boven de vloer blijft en niet omslaat
		const minPitch float32 = 0.05
		const maxPitch float32 = math.Pi/2 - 0.05
		if camPitch < minPitch {
			camPitch = minPitch
		} else if camPitch > maxPitch {
			camPitch = maxPitch
		}

		updateCamera()
	})

	// Scrollwiel beweegt de camera dichterbij / verder weg
	application.Subscribe(window.OnScroll, func(evname string, ev interface{}) {
		if cameraFrozen {
			return
		}
		sev := ev.(*window.ScrollEvent)
		camDistance -= float32(sev.Yoffset) * 0.5
		if camDistance < 1.5 {
			camDistance = 1.5
		} else if camDistance > 60.0 {
			camDistance = 60.0
		}
		updateCamera()
	})

	// 5. Toetsenbord-interactie voor modi
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

	// 6. Render loop / update
	application.Run(func(rend *renderer.Renderer, deltaTime time.Duration) {
		// Houd de oneindige vloer gecentreerd onder de robot op hele rastervakken
		robotPos := zApp.robot.Position()
		snapX := float32(math.Floor(float64(robotPos.X/gridCellSize))) * gridCellSize
		snapZ := float32(math.Floor(float64(robotPos.Z/gridCellSize))) * gridCellSize
		zApp.floor.SetPosition(snapX, floorY, snapZ)

		if !cameraFrozen {
			updateCamera()
		}

		application.Gls().Clear(gls.DEPTH_BUFFER_BIT | gls.COLOR_BUFFER_BIT)
		rend.Render(scene, cam)
	})
}

func (z *ZBotApp) BuildFloor() {
	geom := geometry.NewGeometry()
	positions := math32.NewArrayF32(0, 0)

	extent := float32(gridHalfCount) * gridCellSize
	for i := -gridHalfCount; i <= gridHalfCount; i++ {
		offset := float32(i) * gridCellSize
		// Lijn parallel aan de Z-as
		positions.Append(offset, 0, -extent, offset, 0, extent)
		// Lijn parallel aan de X-as
		positions.Append(-extent, 0, offset, extent, 0, offset)
	}

	geom.AddVBO(gls.NewVBO(positions).AddAttrib(gls.VertexPosition))

	green := &math32.Color{R: 0.0, G: 0.85, B: 0.2}
	matFloor := material.NewStandard(green)
	matFloor.SetEmissiveColor(green)

	z.floor = graphic.NewLines(geom, matFloor)
	z.floor.SetPosition(0, floorY, 0)
	z.scene.Add(z.floor)
}

func (z *ZBotApp) BuildZBot() {
	z.robot = core.NewNode()
	z.scene.Add(z.robot)

	// Materialen
	matChassis := material.NewStandard(&math32.Color{R: 0.15, G: 0.15, B: 0.18}) // Donker carbon/metaal
	matArm := material.NewStandard(&math32.Color{R: 0.2, G: 0.6, B: 0.8})        // Industrieel blauw/teal
	matSensor := material.NewStandard(&math32.Color{R: 0.9, G: 0.5, B: 0.1})     // Accent oranje/geel

	// --- A. Basis Chassis (Skateboard Bakje) ---
	chassisGeom := geometry.NewBox(1.2, 0.3, 2.0)
	z.chassis = graphic.NewMesh(chassisGeom, matChassis)
	z.chassis.SetPosition(0, 0.15, 0)
	z.robot.Add(z.chassis)

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
		z.robot.Add(wheel)
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

	z.robot.Add(z.centralArm)

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
