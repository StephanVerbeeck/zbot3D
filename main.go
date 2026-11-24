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
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/renderer"
	"github.com/g3n/engine/window"
)

const (
	floorY        = -0.15 // Onderkant van de wielen (straal 0.15 gecentreerd op Y=0)
	gridCellSize  = 1.0   // Grootte van elk groen vierkant op de vloer
	gridHalfCount = 100   // Aantal vierkanten in elke richting vanaf het midden

	normalDriveMax float32 = 3.0  // Normale rijsnelheid (units/sec)
	fastDriveMax   float32 = 7.0  // Snelle rijsnelheid met Shift (units/sec)
	driveAccel     float32 = 4.5  // Versnelling (traagheid bij optrekken)
	driveDecel     float32 = 4.0  // Vertraging (traagheid bij uitrollen / Shift loslaten)
	maxSteerAngle  float32 = 0.45 // Max 4WS stuurhoek tijdens rijden (~26 graden)
	steerRate      float32 = 2.5  // Snelheid waarmee stuurhoek opbouwt (rad/sec)
	wheelPivotRate float32 = 7.0  // Draaisnelheid van de wielnaven (rad/sec)
	normalSpinMax  float32 = 1.4  // Normale draaisnelheid op de plaats (rad/sec)
	fastSpinMax    float32 = 2.8  // Snelle draaisnelheid op de plaats met Shift (rad/sec)
	spinAccel      float32 = 3.5  // Hoekversnelling
)

type ZBotApp struct {
	app   *app.Application
	scene *core.Node
	world *World
	robot *Robot
	floor *graphic.Lines
}

func moveToward(current, target, maxDelta float32) float32 {
	if target > current {
		if current+maxDelta > target {
			return target
		}
		return current + maxDelta
	}
	if current-maxDelta < target {
		return target
	}
	return current - maxDelta
}

func lerp(a, b, t float32) float32 {
	return a + (b-a)*t
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

	ambLight := light.NewAmbient(&math32.Color{R: 0.35, G: 0.35, B: 0.35}, 1.0)
	scene.Add(ambLight)
	dirLight := light.NewDirectional(&math32.Color{R: 0.85, G: 0.85, B: 0.85}, 1.0)
	SetNodePosition(dirLight, Point3D{X: 5, Y: 10, Z: 7})
	scene.Add(dirLight)

	zApp := &ZBotApp{
		app:   application,
		scene: scene,
		world: NewWorld(scene),
		robot: NewRobot(scene),
	}

	// 3. Bouw de vloer en de wereldobjecten
	zApp.BuildFloor()
	zApp.world.Build()

	// 4. Muisbesturing voor de camera
	var (
		camYaw       float32 = math.Pi - 0.35 // Start schuin achter de robot
		camPitch     float32 = 0.45
		camDistance  float32 = 7.5
		cameraFrozen         = false
		hasCursorPos         = false
		lastCursorX  float32
		lastCursorY  float32
	)

	upAxis := Point3D{X: 0, Y: 1, Z: 0}

	updateCamera := func() {
		robotCenter := GetNodePosition(zApp.robot).Add(Point3D{Y: 0.6})

		qYaw := NewQuatFromAxisAngle(Point3D{Y: 1}, camYaw)
		qPitch := NewQuatFromAxisAngle(Point3D{X: 1}, -camPitch)
		camOffset := qYaw.Mul(qPitch).Rotate(Point3D{Z: camDistance})
		camPos := robotCenter.Add(camOffset)

		SetNodePosition(cam, camPos)
		cam.LookAt(robotCenter.ToMath32(), upAxis.ToMath32())
	}
	updateCamera()

	// Linkermuisknop schakelt camera-bevriezing aan/uit
	application.Subscribe(window.OnMouseDown, func(evname string, ev interface{}) {
		mev := ev.(*window.MouseEvent)
		if mev.Button == window.MouseButtonLeft {
			cameraFrozen = !cameraFrozen
			if !cameraFrozen {
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

	// 5. Toetsenbord-interactie (modi + rijden/strafen/sturen + wireframe toggle)
	keysDown := make(map[window.Key]bool)
	shiftHeld := false

	application.Subscribe(window.OnKeyDown, func(evname string, ev interface{}) {
		kev := ev.(*window.KeyEvent)
		keysDown[kev.Key] = true
		shiftHeld = (kev.Mods&window.ModShift) != 0 || keysDown[window.KeyLeftShift] || keysDown[window.KeyRightShift]
	})

	application.Subscribe(window.OnKeyUp, func(evname string, ev interface{}) {
		kev := ev.(*window.KeyEvent)
		keysDown[kev.Key] = false
		shiftHeld = (kev.Mods&window.ModShift) != 0 || keysDown[window.KeyLeftShift] || keysDown[window.KeyRightShift]

		switch kev.Key {
		case window.Key0:
			zApp.robot.SetMode(0)
		case window.Key1:
			zApp.robot.SetMode(1)
		case window.Key2:
			zApp.robot.SetMode(2)
		case window.Key3:
			zApp.robot.SetMode(3)
		case window.Key4:
			zApp.robot.SetMode(4)
		case window.Key5:
			zApp.robot.SetMode(5)
		case window.Key6:
			zApp.robot.SetMode(6)
		case window.Key7:
			zApp.robot.SetMode(7)
		case window.Key8:
			zApp.robot.SetMode(8)
		case window.Key9:
			zApp.robot.SetMode(9)
		case window.KeyF:
			application.SetFullScreen(!application.FullScreen())
		case window.KeyM:
			zApp.world.ToggleWireframe()
			zApp.robot.SetWireframe(zApp.world.wireframe)
		}
	})

	// Robot bewegingsstatus (met traagheid in zowel vooruit/achteruit als links/rechts)
	var (
		robotPos     Point3D
		robotHeading float32
		velForward   float32 // Lokale snelheid langs de Z-as (W/S)
		velStrafe    float32 // Lokale snelheid langs de X-as (Q/E)
		currentSteer float32 // Huidige 4WS stuurhoek
		currentSpin  float32 // Huidige draaisnelheid
	)

	// 6. Render loop / update
	application.Run(func(rend *renderer.Renderer, deltaTime time.Duration) {
		dt := float32(deltaTime.Seconds())
		if dt > 0.05 {
			dt = 0.05
		}

		// Lees rij-invoer vooruit/achteruit (W/S) en links/rechts (Q/E)
		var forwardInput float32
		if keysDown[window.KeyW] {
			forwardInput += 1.0
		}
		if keysDown[window.KeyS] {
			forwardInput -= 1.0
		}

		var strafeInput float32
		if keysDown[window.KeyQ] {
			strafeInput += 1.0 // Links (+X in lokaal robotstelsel)
		}
		if keysDown[window.KeyE] {
			strafeInput -= 1.0 // Rechts (-X in lokaal robotstelsel)
		}

		var steerInput float32
		if keysDown[window.KeyA] {
			steerInput += 1.0 // Links draaien (+Y rotatie)
		}
		if keysDown[window.KeyD] {
			steerInput -= 1.0 // Rechts draaien (-Y rotatie)
		}

		// Normaliseer gecombineerde diagonale invoer (bijv. Q+W) zodat de totale rijsnelheid klopt
		inputMag := math32.Sqrt(forwardInput*forwardInput + strafeInput*strafeInput)
		normForward := forwardInput
		normStrafe := strafeInput
		if inputMag > 1.0 {
			normForward /= inputMag
			normStrafe /= inputMag
		}

		// Bepaal doelsnelheid (fast met Shift, normal zonder Shift, 0 bij loslaten)
		maxSpeed := normalDriveMax
		maxSpin := normalSpinMax
		if shiftHeld {
			maxSpeed = fastDriveMax
			maxSpin = fastSpinMax
		}

		targetForward := normForward * maxSpeed
		targetStrafe := normStrafe * maxSpeed

		// Pas 2D lineaire traagheid toe (fast <-> normal <-> zero)
		accelComponent := func(cur, target float32) float32 {
			r := driveDecel
			if math32.Abs(target) > math32.Abs(cur) && target*cur >= 0 {
				r = driveAccel
			} else if target*cur < 0 {
				r = driveAccel + driveDecel
			}
			return moveToward(cur, target, r*dt)
		}
		velForward = accelComponent(velForward, targetForward)
		velStrafe = accelComponent(velStrafe, targetStrafe)

		totalSpeed := math32.Sqrt(velForward*velForward + velStrafe*velStrafe)
		isDriving := totalSpeed >= 0.08 || inputMag > 0

		// Stuurgedrag: tijdens rijden vs. stilstaan (draaien rond het middelpunt)
		targetSteer := steerInput * maxSteerAngle
		if !isDriving {
			targetSteer = 0
		}
		currentSteer = moveToward(currentSteer, targetSteer, steerRate*dt)

		targetSpin := steerInput * maxSpin
		if steerInput == 0 {
			targetSpin = 0
		}
		currentSpin = moveToward(currentSpin, targetSpin, spinAccel*dt)

		var angularVel float32
		if !isDriving {
			// Speciale modus: niet rijden -> draai rond het middelpunt van de robot
			angularVel = currentSpin
		} else {
			// Tijdens rijden: 4WS bochtgedrag gecombineerd met draaien bij zijwaarts rijden
			driveTurn := (velForward / wheelBaseHalf) * math32.Tan(currentSteer)
			strafeTurn := currentSpin
			forwardRatio := float32(0)
			if totalSpeed > 0.001 {
				forwardRatio = math32.Abs(velForward) / totalSpeed
			}
			angularVel = lerp(strafeTurn, driveTurn, forwardRatio)
		}

		// Werk positie en oriëntatie van de robot bij via quaternionrotatie
		robotHeading += angularVel * dt
		robotQuat := NewQuatFromAxisAngle(Point3D{Y: 1}, robotHeading)
		localVel := Point3D{X: velStrafe, Y: 0, Z: velForward}
		worldVel := robotQuat.Rotate(localVel)
		robotPos = robotPos.Add(worldVel.Scale(dt))

		SetNodePosition(zApp.robot, robotPos)
		SetNodeRotation(zApp.robot, robotQuat)
		zApp.robot.UpdateWheels(velForward, velStrafe, angularVel, dt)

		// Houd de oneindige groene rastervloer gecentreerd onder de robot
		floorPos := Point3D{
			X: float32(math.Floor(float64(robotPos.X/gridCellSize))) * gridCellSize,
			Y: floorY,
			Z: float32(math.Floor(float64(robotPos.Z/gridCellSize))) * gridCellSize,
		}
		SetNodePosition(zApp.floor, floorPos)

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

	appendSegment := func(a, b Point3D) {
		positions.Append(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
	}

	extent := float32(gridHalfCount) * gridCellSize
	for i := -gridHalfCount; i <= gridHalfCount; i++ {
		offset := float32(i) * gridCellSize
		// Lijn parallel aan de Z-as
		appendSegment(Point3D{X: offset, Z: -extent}, Point3D{X: offset, Z: extent})
		// Lijn parallel aan de X-as
		appendSegment(Point3D{X: -extent, Z: offset}, Point3D{X: extent, Z: offset})
	}

	geom.AddVBO(gls.NewVBO(positions).AddAttrib(gls.VertexPosition))

	z.floor = graphic.NewLines(geom, MatFloorGrid())
	SetNodePosition(z.floor, Point3D{Y: floorY})
	z.scene.Add(z.floor)
}
