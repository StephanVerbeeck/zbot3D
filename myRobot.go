package main

import (
	"math"

	"github.com/g3n/engine/core"
	"github.com/g3n/engine/math32"
)

const (
	wheelRadius    float32 = 0.15
	wheelWidth     float32 = 0.10
	wheelBaseHalf  float32 = 0.80
	trackWidthHalf float32 = 0.65

	spokeWidth  float32 = 0.11
	spokeHeight float32 = wheelRadius * 1.8
	spokeDepth  float32 = 0.03
)

var wheelPositions = [4]Point3D{
	{X: -trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},  // 0: Links-voor
	{X: trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},   // 1: Rechts-voor
	{X: -trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf}, // 2: Links-achter
	{X: trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf},  // 3: Rechts-achter
}

// Robot represents the 3D Z-Bot node hierarchy and its ShapeInfo-constructed body parts.
type Robot struct {
	core.Node

	chassis      *Shape
	centralArm   *core.Node
	centralBeam  *Shape
	sensorHead   *Shape
	sideArmL     *core.Node
	sideArmR     *core.Node
	sideBeamL    *Shape
	sideBeamR    *Shape
	wheelSteer   [4]*core.Node
	wheelRoll    [4]*core.Node
	wheelShapes  [4]*Shape
	spokeShapes  [4]*Shape
	shapes       []*Shape
	wireframe    bool
	wheelYaw     [4]float32
	wheelRollAng [4]float32
	mode         int // 0: Ingeklapt, 1: SUV/Werkplatform, 2: Volledig Uitgeklapt, 3: Hand-locomotie
}

// NewRobot creates a new Z-Bot, constructs all body parts via ShapeInfo.New(), and attaches it to `scene`.
func NewRobot(scene *core.Node) *Robot {
	r := &Robot{}
	r.Init(r)
	if scene != nil {
		scene.Add(r)
	}
	r.Build()
	return r
}

func (r *Robot) addPart(parent *core.Node, info ShapeInfo, pos Point3D, rot Quat) *Shape {
	part := info.New()
	SetNodePosition(part, pos)
	SetNodeRotation(part, rot)
	parent.Add(part)
	r.shapes = append(r.shapes, part)
	return part
}

// Build constructs every body part of the Z-Bot using ShapeInfo.New().
func (r *Robot) Build() {
	// --- ShapeInfo definities voor alle robotonderdelen ---
	chassisInfo := ShapeInfo{
		Name:     "RobotChassis",
		Material: MatRobotChassis(),
		Steps:    1,
		Move:     []float32{2.0},
		Lines:    [][]Point2D{NewLineRect(1.2, 0.3)},
	}

	wheelInfo := ShapeInfo{
		Name:     "RobotWheel",
		Material: MatRobotWheel(),
		Steps:    20,
		Lines: [][]Point2D{{
			{X: 0.0, Y: 0.0},
			{X: wheelRadius, Y: 0.0},
			{X: wheelRadius, Y: wheelWidth},
			{X: 0.0, Y: wheelWidth},
		}},
	}

	spokeInfo := ShapeInfo{
		Name:     "RobotSpoke",
		Material: MatRobotSpoke(),
		Steps:    1,
		Move:     []float32{spokeDepth},
		Lines:    [][]Point2D{NewLineRect(spokeWidth, spokeHeight)},
	}

	centralArmInfo := ShapeInfo{
		Name:     "CentralArm",
		Material: MatRobotArm(),
		Steps:    1,
		Move:     []float32{0.3},
		Lines:    [][]Point2D{NewLineRect(0.3, 1.2)},
	}

	sensorInfo := ShapeInfo{
		Name:     "SensorHead",
		Material: MatRobotSensor(),
		Steps:    1,
		Move:     []float32{0.25},
		Lines:    [][]Point2D{NewLineRect(0.25, 0.2)},
	}

	sideArmInfo := ShapeInfo{
		Name:     "SideArm",
		Material: MatRobotArm(),
		Steps:    1,
		Move:     []float32{0.2},
		Lines:    [][]Point2D{NewLineRect(0.2, 1.0)},
	}

	// --- A. Basis Chassis (Skateboard Bakje: 1.2 x 0.3 x 2.0 gecentreerd op Y=0.15) ---
	r.chassis = r.addPart(&r.Node, chassisInfo, Point3D{X: -0.6, Y: 0.0, Z: -1.0}, IdentityQuat())

	// --- B. Wielen (4 stuks met stuur- en rolhiërarchie + zichtbare spaak) ---
	wheelAlignQuat := NewQuatFromAxisAngle(Point3D{Z: 1}, math.Pi/2)
	wheelCenterOffset := Point3D{X: wheelWidth * 0.5}
	spokeCenterOffset := Point3D{X: -spokeWidth * 0.5, Y: -spokeHeight * 0.5, Z: -spokeDepth * 0.5}

	for i, pos := range wheelPositions {
		steerNode := core.NewNode()
		SetNodePosition(steerNode, pos)

		rollNode := core.NewNode()
		steerNode.Add(rollNode)

		r.wheelShapes[i] = r.addPart(rollNode, wheelInfo, wheelCenterOffset, wheelAlignQuat)
		r.spokeShapes[i] = r.addPart(rollNode, spokeInfo, spokeCenterOffset, IdentityQuat())

		r.wheelSteer[i] = steerNode
		r.wheelRoll[i] = rollNode
		r.Add(steerNode)
	}

	// --- C. Centrale Stam (Hefkolom + Sensor Head) ---
	r.centralArm = core.NewNode()
	r.centralBeam = r.addPart(r.centralArm, centralArmInfo, Point3D{X: -0.15, Y: 0.0, Z: -0.15}, IdentityQuat())
	r.sensorHead = r.addPart(r.centralArm, sensorInfo, Point3D{X: -0.125, Y: 1.2, Z: -0.125}, IdentityQuat())
	r.Add(r.centralArm)

	// --- D. Twee Zijarmen (Montage aan de centrale arm) ---
	r.sideArmL = core.NewNode()
	r.sideBeamL = r.addPart(r.sideArmL, sideArmInfo, Point3D{X: -0.4, Y: 0.0, Z: -0.1}, IdentityQuat())
	r.centralArm.Add(r.sideArmL)

	r.sideArmR = core.NewNode()
	r.sideBeamR = r.addPart(r.sideArmR, sideArmInfo, Point3D{X: 0.2, Y: 0.0, Z: -0.1}, IdentityQuat())
	r.centralArm.Add(r.sideArmR)

	// Start in modus 0 (Ingeklapt)
	r.SetMode(0)
}

// SetWireframe switches all robot Shape parts between solid and wireframe rendering.
func (r *Robot) SetWireframe(wireframe bool) {
	r.wireframe = wireframe
	for _, s := range r.shapes {
		s.SetWireframe(wireframe)
	}
}

// UpdateWheels computes 4WS steering angles and rolling rotations from local robot velocities.
func (r *Robot) UpdateWheels(velForward, velStrafe, angularVel, dt float32) {
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

		r.wheelYaw[i] = moveToward(r.wheelYaw[i], targetYaw, wheelPivotRate*dt)
		steerQuat := NewQuatFromAxisAngle(Point3D{Y: 1}, r.wheelYaw[i])
		SetNodeRotation(r.wheelSteer[i], steerQuat)

		// Projecteer de wielsnelheid op de huidige rolrichting van het wiel
		rollDir := steerQuat.Rotate(Point3D{Z: 1})
		rimSpeed := wheelVel.Dot(rollDir)

		r.wheelRollAng[i] += (rimSpeed / wheelRadius) * dt
		rollQuat := NewQuatFromAxisAngle(Point3D{X: 1}, r.wheelRollAng[i])
		SetNodeRotation(r.wheelRoll[i], rollQuat)
	}
}

// SetMode sets the Z-Bot upper body to one of its standard postures.
func (r *Robot) SetMode(newMode int) {
	r.mode = newMode
	switch r.mode {
	case 0: // Ingeklapt (Nesting in bakje)
		SetNodePosition(r.centralArm, Point3D{})
		SetNodeRotation(r.centralArm, IdentityQuat())
		SetNodePosition(r.sideArmL, Point3D{})
		SetNodeRotation(r.sideArmL, IdentityQuat())
		SetNodePosition(r.sideArmR, Point3D{})
		SetNodeRotation(r.sideArmR, IdentityQuat())
	case 1: // SUV / Werkplatform Mode
		SetNodePosition(r.centralArm, Point3D{Y: 0.3, Z: -0.5})
		SetNodeRotation(r.centralArm, NewQuatFromAxisAngle(Point3D{X: 1}, 0.3))
		SetNodePosition(r.sideArmL, Point3D{X: -0.4, Y: 0.8})
		SetNodeRotation(r.sideArmL, IdentityQuat())
		SetNodePosition(r.sideArmR, Point3D{X: 0.4, Y: 0.8})
		SetNodeRotation(r.sideArmR, IdentityQuat())
	case 2: // Volledig Uitgeklapt Bimanual Mode
		SetNodePosition(r.centralArm, Point3D{Y: 0.5})
		SetNodeRotation(r.centralArm, IdentityQuat())
		SetNodePosition(r.sideArmL, Point3D{X: -0.8, Y: 1.2, Z: 0.2})
		SetNodeRotation(r.sideArmL, NewQuatFromAxisAngle(Point3D{Z: 1}, -0.5))
		SetNodePosition(r.sideArmR, Point3D{X: 0.8, Y: 1.2, Z: 0.2})
		SetNodeRotation(r.sideArmR, NewQuatFromAxisAngle(Point3D{Z: 1}, 0.5))
	case 3: // Hand-locomotie Mode (op de handen lopen)
		SetNodePosition(r.centralArm, Point3D{Y: 1.0})
		SetNodeRotation(r.centralArm, NewQuatFromAxisAngle(Point3D{X: 1}, -0.5))
		SetNodePosition(r.sideArmL, Point3D{X: -0.6, Z: 1.5})
		SetNodeRotation(r.sideArmL, IdentityQuat())
		SetNodePosition(r.sideArmR, Point3D{X: 0.6, Z: 1.5})
		SetNodeRotation(r.sideArmR, IdentityQuat())
	}
}
