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

	postureTransitionFrames = 100
	postureDOFCount         = 3
)

var wheelPositions = [4]Point3D{
	{X: -trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},  // 0: Links-voor
	{X: trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},   // 1: Rechts-voor
	{X: -trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf}, // 2: Links-achter
	{X: trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf},  // 3: Rechts-achter
}

// Robot represents the 3D Z-Bot node hierarchy, its ShapeInfo-constructed body parts,
// and the Hinge joints driving smooth upper-body posture transitions.
type Robot struct {
	core.Node

	chassis      *Shape
	centralBeam  *Shape
	sensorHead   *Shape
	sideBeamL    *Shape
	sideBeamR    *Shape
	centralHinge *Hinge
	sensorHinge  *Hinge
	sideHingeL   *Hinge
	sideHingeR   *Hinge
	wheelSteer   [4]*core.Node
	wheelRoll    [4]*core.Node
	wheelShapes  [4]*Shape
	spokeShapes  [4]*Shape
	shapes       []*Shape
	wireframe    bool
	wheelYaw     [4]float32
	wheelRollAng [4]float32

	mode               int // 0: Ingeklapt, 1: SUV/Werkplatform, 2: Volledig Uitgeklapt, 3: Hand-locomotie
	targetMode         int
	postureStep        int
	startCentralRatios [postureDOFCount]float32
	startSideLRatios   [postureDOFCount]float32
	startSideRRatios   [postureDOFCount]float32
	targetRatios       [postureDOFCount]float32
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

// Build constructs every body part of the Z-Bot using ShapeInfo.New() and wires the upper-body Hinges.
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
	chassisAt := AtMove(Point3D{X: -0.6, Y: 0.0, Z: -1.0})
	r.chassis = r.addPart(&r.Node, chassisInfo, chassisAt.Move, chassisAt.Turn)

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

	// --- C. Bovenlichaam onderdelen & Hinges ---
	r.centralBeam = r.addPart(&r.Node, centralArmInfo, Point3D{}, IdentityQuat())
	r.sensorHead = r.addPart(&r.Node, sensorInfo, Point3D{}, IdentityQuat())
	r.sideBeamL = r.addPart(&r.Node, sideArmInfo, Point3D{}, IdentityQuat())
	r.sideBeamR = r.addPart(&r.Node, sideArmInfo, Point3D{}, IdentityQuat())

	chassisInv := chassisAt.Inverse()
	beamOffset := AtMove(Point3D{X: -0.15, Y: 0.0, Z: -0.15})
	beamOffsetInv := beamOffset.Inverse()
	sensorOffset := AtMove(Point3D{X: -0.125, Y: 1.2, Z: -0.125})
	sideBeamLOffset := AtMove(Point3D{X: -0.4, Y: 0.0, Z: -0.1})
	sideBeamROffset := AtMove(Point3D{X: 0.2, Y: 0.0, Z: -0.1})

	centralPose := func(armAt At) At {
		return chassisInv.At(armAt.At(beamOffset))
	}
	sideLPose := func(armAt At) At {
		return beamOffsetInv.At(armAt.At(sideBeamLOffset))
	}
	sideRPose := func(armAt At) At {
		return beamOffsetInv.At(armAt.At(sideBeamROffset))
	}

	// 1. Centrale stam hinge (Parent: chassis -> Child: centralBeam)
	r.centralHinge = NewHinge(r.chassis, chassisAt, r.centralBeam, centralPose(IdentityAt()))
	r.centralHinge.AddDOF(0, centralPose(NewAtAxisAngle(Point3D{Y: 0.3, Z: -0.5}, Point3D{X: 1}, 0.3))) // Modus 1
	r.centralHinge.AddDOF(0, centralPose(AtMove(Point3D{Y: 0.5})))                                      // Modus 2
	r.centralHinge.AddDOF(0, centralPose(NewAtAxisAngle(Point3D{Y: 1.0}, Point3D{X: 1}, -0.5)))         // Modus 3

	// 2. Sensor head hinge (Parent: centralBeam -> Child: sensorHead)
	r.sensorHinge = NewHinge(r.centralBeam, r.centralHinge.Child.At, r.sensorHead, beamOffsetInv.At(sensorOffset))

	// 3. Linker zijarm hinge (Parent: centralBeam -> Child: sideBeamL)
	r.sideHingeL = NewHinge(r.centralBeam, r.centralHinge.Child.At, r.sideBeamL, sideLPose(IdentityAt()))
	r.sideHingeL.AddDOF(0, sideLPose(AtMove(Point3D{X: -0.4, Y: 0.8})))                                      // Modus 1
	r.sideHingeL.AddDOF(0, sideLPose(NewAtAxisAngle(Point3D{X: -0.8, Y: 1.2, Z: 0.2}, Point3D{Z: 1}, -0.5))) // Modus 2
	r.sideHingeL.AddDOF(0, sideLPose(AtMove(Point3D{X: -0.6, Z: 1.5})))                                      // Modus 3

	// 4. Rechter zijarm hinge (Parent: centralBeam -> Child: sideBeamR)
	r.sideHingeR = NewHinge(r.centralBeam, r.centralHinge.Child.At, r.sideBeamR, sideRPose(IdentityAt()))
	r.sideHingeR.AddDOF(0, sideRPose(AtMove(Point3D{X: 0.4, Y: 0.8})))                                     // Modus 1
	r.sideHingeR.AddDOF(0, sideRPose(NewAtAxisAngle(Point3D{X: 0.8, Y: 1.2, Z: 0.2}, Point3D{Z: 1}, 0.5))) // Modus 2
	r.sideHingeR.AddDOF(0, sideRPose(AtMove(Point3D{X: 0.6, Z: 1.5})))                                     // Modus 3

	// Start direct in modus 0 (Ingeklapt)
	r.mode = 0
	r.targetMode = 0
	r.postureStep = postureTransitionFrames
	r.updateUpperBodyHinges()
}

// SetWireframe switches all robot Shape parts between solid and wireframe rendering.
func (r *Robot) SetWireframe(wireframe bool) {
	r.wireframe = wireframe
	for _, s := range r.shapes {
		s.SetWireframe(wireframe)
	}
}

// UpdateWheels computes 4WS steering angles and rolling rotations from local robot velocities,
// and advances any active upper-body posture transition by one frame step.
func (r *Robot) UpdateWheels(velForward, velStrafe, angularVel, dt float32) {
	r.UpdatePosture()

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

func modeRatios(mode int) [postureDOFCount]float32 {
	var ratios [postureDOFCount]float32
	if mode >= 1 && mode <= postureDOFCount {
		ratios[mode-1] = 1.0
	}
	return ratios
}

func captureDOFRatios(h *Hinge) [postureDOFCount]float32 {
	var ratios [postureDOFCount]float32
	if h == nil {
		return ratios
	}
	for i := 0; i < postureDOFCount && i < len(h.DOF); i++ {
		ratios[i] = h.DOF[i].Ratio
	}
	return ratios
}

func applyLerpedDOFRatios(h *Hinge, start, target [postureDOFCount]float32, t float32) {
	if h == nil {
		return
	}
	for i := 0; i < postureDOFCount && i < len(h.DOF); i++ {
		h.DOF[i].Ratio = start[i] + (target[i]-start[i])*t
	}
}

func (r *Robot) updateUpperBodyHinges() {
	if r.centralHinge == nil {
		return
	}
	centralWorld := r.centralHinge.Update()

	if r.sensorHinge != nil {
		r.sensorHinge.Parent.At = centralWorld
		r.sensorHinge.Update()
	}
	if r.sideHingeL != nil {
		r.sideHingeL.Parent.At = centralWorld
		r.sideHingeL.Update()
	}
	if r.sideHingeR != nil {
		r.sideHingeR.Parent.At = centralWorld
		r.sideHingeR.Update()
	}
}

// UpdatePosture advances the upper-body hinge interpolation by 1 frame step (out of 100)
// whenever a posture change is in progress.
func (r *Robot) UpdatePosture() {
	if r.postureStep >= postureTransitionFrames {
		return
	}
	r.postureStep++
	t := float32(r.postureStep) / float32(postureTransitionFrames)

	applyLerpedDOFRatios(r.centralHinge, r.startCentralRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.sideHingeL, r.startSideLRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.sideHingeR, r.startSideRRatios, r.targetRatios, t)
	r.updateUpperBodyHinges()

	if r.postureStep == postureTransitionFrames {
		r.mode = r.targetMode
	}
}

// SetMode starts a 100-frame hinge transition of the Z-Bot upper body to `newMode` (0..3).
func (r *Robot) SetMode(newMode int) {
	if newMode < 0 || newMode > postureDOFCount {
		return
	}
	if newMode == r.targetMode && r.postureStep >= postureTransitionFrames {
		return
	}

	r.startCentralRatios = captureDOFRatios(r.centralHinge)
	r.startSideLRatios = captureDOFRatios(r.sideHingeL)
	r.startSideRRatios = captureDOFRatios(r.sideHingeR)
	r.targetRatios = modeRatios(newMode)
	r.targetMode = newMode
	r.postureStep = 0
}
