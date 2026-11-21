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
	floorY        = -0.15 // Onderkant van de wielen (straal 0.15 gecentreerd op Y=0)
	gridCellSize  = 1.0   // Grootte van elk groen vierkant op de vloer
	gridHalfCount = 100   // Aantal vierkanten in elke richting vanaf het midden

	wheelRadius    float32 = 0.15
	wheelBaseHalf  float32 = 0.80
	trackWidthHalf float32 = 0.65
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

var wheelPositions = [4]Point3D{
	{X: -trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},  // 0: Links-voor
	{X: trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},   // 1: Rechts-voor
	{X: -trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf}, // 2: Links-achter
	{X: trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf},  // 3: Rechts-achter
}

type ZBotApp struct {
	app          *app.Application
	scene        *core.Node
	world        *World
	robot        *core.Node
	floor        *graphic.Lines
	chassis      *graphic.Mesh
	centralArm   *core.Node
	sideArmL     *core.Node
	sideArmR     *core.Node
	wheelSteer   [4]*core.Node
	wheelRoll    [4]*core.Node
	wheelYaw     [4]float32
	wheelRollAng [4]float32
	mode         int // 0: Ingeklapt, 1: SUV/Werkplatform, 2: Volledig Uitgeklapt, 3: Hand-locomotie
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
		mode:  0,
	}

	// 3. Bouw de vloer, de Z-Bot hiërarchie en de wereldobjecten
	zApp.BuildFloor()
	zApp.BuildZBot()
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
		case window.Key1:
			zApp.setMode(0) // Ingeklapt (Nesting Cassette)
		case window.Key2:
			zApp.setMode(1) // SUV / Werkplatform Mode
		case window.Key3:
			zApp.setMode(2) // Volledig Uitgeklapt Bimanual Mode
		case window.Key4:
			zApp.setMode(3) // Hand-locomotie Mode
		case window.KeyM:
			zApp.world.ToggleWireframe()
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

		// Bereken voor elk wiel de exacte lokale snelheidsvector (translatie + rotatie)
		// zodat alle 4 wielen in de gereden richting sturen én rollen
		for i := 0; i < 4; i++ {
			wp := wheelPositions[i]

			// v_wiel = v_robot + omega x r_wiel
			wheelVel := Point3D{
				X: velStrafe + angularVel*wp.Z,
				Y: 0,
				Z: velForward - angularVel*wp.X,
			}
			wheelSpeed := wheelVel.Length()

			targetYaw := float32(0)
			if wheelSpeed > 0.02 {
				if wheelVel.Z >= 0 {
					targetYaw = math32.Atan2(wheelVel.X, wheelVel.Z)
				} else {
					targetYaw = math32.Atan2(-wheelVel.X, -wheelVel.Z)
				}
			}

			zApp.wheelYaw[i] = moveToward(zApp.wheelYaw[i], targetYaw, wheelPivotRate*dt)
			steerQuat := NewQuatFromAxisAngle(Point3D{Y: 1}, zApp.wheelYaw[i])
			SetNodeRotation(zApp.wheelSteer[i], steerQuat)

			// Projecteer de wielsnelheid op de huidige rolrichting van het wiel
			rollDir := steerQuat.Rotate(Point3D{Z: 1})
			rimSpeed := wheelVel.Dot(rollDir)

			zApp.wheelRollAng[i] += (rimSpeed / wheelRadius) * dt
			rollQuat := NewQuatFromAxisAngle(Point3D{X: 1}, zApp.wheelRollAng[i])
			SetNodeRotation(zApp.wheelRoll[i], rollQuat)
		}

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

	green := &math32.Color{R: 0.0, G: 0.85, B: 0.2}
	matFloor := material.NewStandard(green)
	matFloor.SetEmissiveColor(green)

	z.floor = graphic.NewLines(geom, matFloor)
	SetNodePosition(z.floor, Point3D{Y: floorY})
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
	SetNodePosition(z.chassis, Point3D{Y: 0.15})
	z.robot.Add(z.chassis)

	// Wielen (4 stuks met stuur- en rolhiërarchie + zichtbare spaak)
	wheelGeom := geometry.NewCylinder(float64(wheelRadius), 0.1, 16, 1, true, true)
	spokeGeom := geometry.NewBox(0.11, wheelRadius*1.8, 0.03)
	matWheel := material.NewStandard(&math32.Color{R: 0.1, G: 0.1, B: 0.1})
	matSpoke := material.NewStandard(&math32.Color{R: 0.85, G: 0.85, B: 0.85})
	wheelAlignQuat := NewQuatFromAxisAngle(Point3D{Z: 1}, math.Pi/2)

	for i, pos := range wheelPositions {
		steerNode := core.NewNode()
		SetNodePosition(steerNode, pos)

		rollNode := core.NewNode()
		steerNode.Add(rollNode)

		wheel := graphic.NewMesh(wheelGeom, matWheel)
		SetNodeRotation(wheel, wheelAlignQuat)
		rollNode.Add(wheel)

		spoke := graphic.NewMesh(spokeGeom, matSpoke)
		rollNode.Add(spoke)

		z.wheelSteer[i] = steerNode
		z.wheelRoll[i] = rollNode
		z.robot.Add(steerNode)
	}

	// --- B. Centrale Stam (Heftank / Hefkolom) ---
	z.centralArm = core.NewNode()
	centralGeom := geometry.NewBox(0.3, 1.2, 0.3)
	centralMesh := graphic.NewMesh(centralGeom, matArm)
	SetNodePosition(centralMesh, Point3D{Y: 0.6})
	z.centralArm.Add(centralMesh)

	// Sensor Head op de top van de centrale stam
	sensorGeom := geometry.NewBox(0.25, 0.2, 0.25)
	sensorMesh := graphic.NewMesh(sensorGeom, matSensor)
	SetNodePosition(sensorMesh, Point3D{Y: 1.3})
	z.centralArm.Add(sensorMesh)

	z.robot.Add(z.centralArm)

	// --- C. Twee Zijarmen (Montage aan de top van de centrale arm) ---
	z.sideArmL = core.NewNode()
	sideGeomL := geometry.NewBox(0.2, 1.0, 0.2)
	sideMeshL := graphic.NewMesh(sideGeomL, matArm)
	SetNodePosition(sideMeshL, Point3D{X: -0.3, Y: 0.5})
	z.sideArmL.Add(sideMeshL)
	z.centralArm.Add(z.sideArmL)

	z.sideArmR = core.NewNode()
	sideGeomR := geometry.NewBox(0.2, 1.0, 0.2)
	sideMeshR := graphic.NewMesh(sideGeomR, matArm)
	SetNodePosition(sideMeshR, Point3D{X: 0.3, Y: 0.5})
	z.sideArmR.Add(sideMeshR)
	z.centralArm.Add(z.sideArmR)

	// Start in modus 0 (Ingeklapt)
	z.setMode(0)
}

func (z *ZBotApp) setMode(newMode int) {
	z.mode = newMode
	switch z.mode {
	case 0: // Ingeklapt (Nesting in bakje)
		SetNodePosition(z.centralArm, Point3D{})
		SetNodeRotation(z.centralArm, IdentityQuat())
		SetNodePosition(z.sideArmL, Point3D{})
		SetNodeRotation(z.sideArmL, IdentityQuat())
		SetNodePosition(z.sideArmR, Point3D{})
		SetNodeRotation(z.sideArmR, IdentityQuat())
	case 1: // SUV / Werkplatform Mode
		SetNodePosition(z.centralArm, Point3D{Y: 0.3, Z: -0.5})
		SetNodeRotation(z.centralArm, NewQuatFromAxisAngle(Point3D{X: 1}, 0.3))
		SetNodePosition(z.sideArmL, Point3D{X: -0.4, Y: 0.8})
		SetNodeRotation(z.sideArmL, IdentityQuat())
		SetNodePosition(z.sideArmR, Point3D{X: 0.4, Y: 0.8})
		SetNodeRotation(z.sideArmR, IdentityQuat())
	case 2: // Volledig Uitgeklapt Bimanual Mode
		SetNodePosition(z.centralArm, Point3D{Y: 0.5})
		SetNodeRotation(z.centralArm, IdentityQuat())
		SetNodePosition(z.sideArmL, Point3D{X: -0.8, Y: 1.2, Z: 0.2})
		SetNodeRotation(z.sideArmL, NewQuatFromAxisAngle(Point3D{Z: 1}, -0.5))
		SetNodePosition(z.sideArmR, Point3D{X: 0.8, Y: 1.2, Z: 0.2})
		SetNodeRotation(z.sideArmR, NewQuatFromAxisAngle(Point3D{Z: 1}, 0.5))
	case 3: // Hand-locomotie Mode (op de handen lopen)
		SetNodePosition(z.centralArm, Point3D{Y: 1.0})
		SetNodeRotation(z.centralArm, NewQuatFromAxisAngle(Point3D{X: 1}, -0.5))
		SetNodePosition(z.sideArmL, Point3D{X: -0.6, Z: 1.5})
		SetNodeRotation(z.sideArmL, IdentityQuat())
		SetNodePosition(z.sideArmR, Point3D{X: 0.6, Z: 1.5})
		SetNodeRotation(z.sideArmR, IdentityQuat())
	}
}
