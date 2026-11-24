package main

import (
	"math"

	"github.com/g3n/engine/core"
	"github.com/g3n/engine/math32"
)

const (
	twoPi float32 = 2.0 * math.Pi

	wheelRadius    float32 = 0.15
	wheelWidth     float32 = 0.10
	wheelBaseHalf  float32 = 0.80
	trackWidthHalf float32 = 0.65

	spokeWidth  float32 = 0.11
	spokeHeight float32 = wheelRadius * 1.8
	spokeDepth  float32 = 0.03

	// Afmetingen van de uitstekende mechaniek-bodem + het open bakje erbovenop
	chassisWidth        float32 = 1.20  // X: -0.60 .. +0.60 (past exact tussen de wielen)
	chassisLength       float32 = 2.00  // Z: -1.00 .. +1.00
	chassisFloorBottomY float32 = -0.09 // Steekt naar beneden uit tussen de wielen (wielas = 0.0, grond = -0.15)
	chassisFloorHeight  float32 = 0.30  // Dikke bodem voor aandrijving/stuurmechaniek (Y: -0.09 .. +0.21)
	chassisSumpWidth    float32 = 0.92  // Onderste mechaniekbak die extra naar beneden uitsteekt
	chassisSumpHeight   float32 = 0.06  // Y: -0.12 .. -0.06 (3 cm bodemvrijheid boven de grond)
	chassisSumpLength   float32 = 1.84
	chassisRimThickness float32 = 0.08 // Dikte van elke zijrand
	chassisRimHeight    float32 = 0.50 // Hoogte van de randen boven de bakbodem (Y: +0.21 .. +0.71)

	// Gelijke afmetingen voor beide middenarm-delen en alle vier zijarm-delen
	armWidth  float32 = 0.24
	armLength float32 = 1.40
	armDepth  float32 = 0.24

	// Kleine camerakop op het uiteinde van de middenarm
	cameraWidth      float32 = 0.18
	cameraHeight     float32 = 0.14
	cameraDepth      float32 = 0.14
	cameraLensRadius float32 = 0.045
	cameraLensLength float32 = 0.05

	wristWidth  float32 = 0.22
	wristLength float32 = 0.14
	wristDepth  float32 = 0.16

	fingerWidth  float32 = 0.045
	fingerLength float32 = 0.24
	fingerDepth  float32 = 0.12

	postureCount            = 10 // Houdingen 0..9
	postureTransitionFrames = 100
)

// Fysieke bewegingsgrenzen in omwentelingen (turns: 1.0 = 1 volledige rotatie = 360°).
// Elk scharnier heeft uitsluitend 1 of 2 fysieke buigrichtingen (bijv. pitch en roll/yaw).
const (
	lowerCenterPitchMin float32 = -0.08
	lowerCenterPitchMax float32 = 0.26

	upperCenterPitchMin float32 = -0.498
	upperCenterPitchMax float32 = -0.015

	cameraPitchMin float32 = 0.04
	cameraPitchMax float32 = 0.498

	shoulderPitchMin float32 = 0.02
	shoulderPitchMax float32 = 0.498

	shoulderRollLMin float32 = -0.05
	shoulderRollLMax float32 = 0.27

	shoulderRollRMin float32 = -0.27
	shoulderRollRMax float32 = 0.05

	elbowPitchMin float32 = -0.498
	elbowPitchMax float32 = 0.00

	elbowRollMin float32 = -0.07
	elbowRollMax float32 = 0.07

	wristPitchMin float32 = -0.025
	wristPitchMax float32 = 0.465

	wristYawLMin float32 = 0.00
	wristYawLMax float32 = 0.26

	wristYawRMin float32 = -0.26
	wristYawRMax float32 = 0.00

	jawOpenTurns float32 = 0.083 // ~30° opening per vinger
)

// PostureTurns legt voor één houding (0..9) de gewenste hoeken in omwentelingen (turns) vast
// voor elke fysieke vrijheidsgraad van elk scharnier (1.0 = 1 volledige rotatie).
type PostureTurns struct {
	LowerCenterPitch float32    // 1 DOF: pitchX
	UpperCenterPitch float32    // 1 DOF: pitchX
	CameraPitch      float32    // 1 DOF: pitchX
	UpperSideL       [2]float32 // 2 DOFs: [0]=pitchX, [1]=rollZ
	LowerSideL       [2]float32 // 2 DOFs: [0]=pitchX, [1]=rollZ
	UpperSideR       [2]float32 // 2 DOFs: [0]=pitchX, [1]=rollZ
	LowerSideR       [2]float32 // 2 DOFs: [0]=pitchX, [1]=rollZ
	WristL           [2]float32 // 2 DOFs: [0]=pitchX, [1]=yawY
	WristR           [2]float32 // 2 DOFs: [0]=pitchX, [1]=yawY
	GripperOpen      float32    // 1 DOF per vinger: 0.0 (dicht) .. 1.0 (open)
}

var robotPostures = [postureCount]PostureTurns{
	// 0: Volledig opgevouwen in het chassis-bakje onder de rand (Y: +0.21 .. +0.69)
	{
		LowerCenterPitch: 0.250,
		UpperCenterPitch: -0.498,
		CameraPitch:      0.496,
		UpperSideL:       [2]float32{0.498, 0.000},
		LowerSideL:       [2]float32{-0.496, 0.000},
		UpperSideR:       [2]float32{0.498, 0.000},
		LowerSideR:       [2]float32{-0.496, 0.000},
		WristL:           [2]float32{0.462, 0.000},
		WristR:           [2]float32{0.462, 0.000},
		GripperOpen:      0.00,
	},
	// 1: Klassieke Z-werkhouding
	{
		LowerCenterPitch: 0.108,
		UpperCenterPitch: -0.226,
		CameraPitch:      0.342,
		UpperSideL:       [2]float32{0.279, 0.019},
		LowerSideL:       [2]float32{-0.151, 0.000},
		UpperSideR:       [2]float32{0.279, -0.019},
		LowerSideR:       [2]float32{-0.151, 0.000},
		WristL:           [2]float32{0.088, 0.000},
		WristR:           [2]float32{0.088, 0.000},
		GripperOpen:      0.45,
	},
	// 2: Zeer hoog reikend
	{
		LowerCenterPitch: 0.019,
		UpperCenterPitch: -0.038,
		CameraPitch:      0.151,
		UpperSideL:       [2]float32{0.024, 0.013},
		LowerSideL:       [2]float32{-0.016, -0.010},
		UpperSideR:       [2]float32{0.024, -0.013},
		LowerSideR:       [2]float32{-0.016, 0.010},
		WristL:           [2]float32{0.000, 0.000},
		WristR:           [2]float32{0.000, 0.000},
		GripperOpen:      1.00,
	},
	// 3: Zeer breed gespreide armen (T-pose)
	{
		LowerCenterPitch: 0.067,
		UpperCenterPitch: -0.134,
		CameraPitch:      0.322,
		UpperSideL:       [2]float32{0.072, 0.247},
		LowerSideL:       [2]float32{0.000, 0.006},
		UpperSideR:       [2]float32{0.072, -0.247},
		LowerSideR:       [2]float32{0.000, -0.006},
		WristL:           [2]float32{0.000, 0.064},
		WristR:           [2]float32{0.000, -0.064},
		GripperOpen:      0.85,
	},
	// 4: Zeer laag vooruit reikend
	{
		LowerCenterPitch: 0.151,
		UpperCenterPitch: -0.215,
		CameraPitch:      0.406,
		UpperSideL:       [2]float32{0.470, 0.032},
		LowerSideL:       [2]float32{-0.135, -0.016},
		UpperSideR:       [2]float32{0.470, -0.032},
		LowerSideR:       [2]float32{-0.135, 0.016},
		WristL:           [2]float32{-0.024, 0.000},
		WristR:           [2]float32{-0.024, 0.000},
		GripperOpen:      0.90,
	},
	// 5: Laag en breed zijwaarts reikend
	{
		LowerCenterPitch: 0.135,
		UpperCenterPitch: -0.223,
		CameraPitch:      0.406,
		UpperSideL:       [2]float32{0.470, 0.135},
		LowerSideL:       [2]float32{-0.119, 0.029},
		UpperSideR:       [2]float32{0.470, -0.135},
		LowerSideR:       [2]float32{-0.119, -0.029},
		WristL:           [2]float32{0.000, 0.056},
		WristR:           [2]float32{0.000, -0.056},
		GripperOpen:      0.75,
	},
	// 6: Ver vooruit gestrekt
	{
		LowerCenterPitch: 0.167,
		UpperCenterPitch: -0.175,
		CameraPitch:      0.271,
		UpperSideL:       [2]float32{0.266, 0.022},
		LowerSideL:       [2]float32{0.000, -0.016},
		UpperSideR:       [2]float32{0.266, -0.022},
		LowerSideR:       [2]float32{0.000, 0.016},
		WristL:           [2]float32{0.000, 0.032},
		WristR:           [2]float32{0.000, -0.032},
		GripperOpen:      0.80,
	},
	// 7: Achterwaartse kraanhouding
	{
		LowerCenterPitch: -0.072,
		UpperCenterPitch: -0.135,
		CameraPitch:      0.064,
		UpperSideL:       [2]float32{0.040, 0.056},
		LowerSideL:       [2]float32{-0.095, -0.032},
		UpperSideR:       [2]float32{0.040, -0.056},
		LowerSideR:       [2]float32{-0.095, 0.032},
		WristL:           [2]float32{-0.024, 0.000},
		WristR:           [2]float32{-0.024, 0.000},
		GripperOpen:      0.50,
	},
	// 8: Hand-lopen basishouding (beide handen links en rechts op de grond)
	{
		LowerCenterPitch: 0.150,
		UpperCenterPitch: -0.264,
		CameraPitch:      0.414,
		UpperSideL:       [2]float32{0.438, 0.099},
		LowerSideL:       [2]float32{-0.014, 0.000},
		UpperSideR:       [2]float32{0.438, -0.099},
		LowerSideR:       [2]float32{-0.014, 0.000},
		WristL:           [2]float32{0.000, 0.250},
		WristR:           [2]float32{0.000, -0.250},
		GripperOpen:      1.00,
	},
	// 9: Hand-lopen stap (linkerhand vooruit op de grond, rechterhand achteruit op de grond)
	{
		LowerCenterPitch: 0.150,
		UpperCenterPitch: -0.264,
		CameraPitch:      0.414,
		UpperSideL:       [2]float32{0.379, 0.092},
		LowerSideL:       [2]float32{0.000, 0.000},
		UpperSideR:       [2]float32{0.485, -0.103},
		LowerSideR:       [2]float32{-0.072, 0.000},
		WristL:           [2]float32{0.000, 0.250},
		WristR:           [2]float32{0.000, -0.250},
		GripperOpen:      1.00,
	},
}

var wheelPositions = [4]Point3D{
	{X: -trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},  // 0: Links-voor
	{X: trackWidthHalf, Y: 0.0, Z: wheelBaseHalf},   // 1: Rechts-voor
	{X: -trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf}, // 2: Links-achter
	{X: trackWidthHalf, Y: 0.0, Z: -wheelBaseHalf},  // 3: Rechts-achter
}

// Robot represents the 3D Z-Bot node hierarchy, its ShapeInfo-constructed body parts,
// and the full Hinge chain driving the 2-part Z-shaped center arm, camera head, 2-part side arms, wrists, and grippers.
type Robot struct {
	core.Node

	chassis        *Shape
	chassisSump    *Shape
	rimLeft        *Shape
	rimRight       *Shape
	rimFront       *Shape
	rimBack        *Shape
	lowerCenterArm *Shape
	upperCenterArm *Shape
	cameraHead     *Shape
	cameraLens     *Shape
	upperSideArmL  *Shape
	lowerSideArmL  *Shape
	upperSideArmR  *Shape
	lowerSideArmR  *Shape
	wristL         *Shape
	wristR         *Shape
	gripFingerL1   *Shape
	gripFingerL2   *Shape
	gripFingerR1   *Shape
	gripFingerR2   *Shape

	lowerCenterHinge *Hinge
	upperCenterHinge *Hinge
	cameraHinge      *Hinge
	cameraLensHinge  *Hinge
	upperSideHingeL  *Hinge
	lowerSideHingeL  *Hinge
	upperSideHingeR  *Hinge
	lowerSideHingeR  *Hinge
	wristHingeL      *Hinge
	wristHingeR      *Hinge
	gripHingeL1      *Hinge
	gripHingeL2      *Hinge
	gripHingeR1      *Hinge
	gripHingeR2      *Hinge

	wheelSteer   [4]*core.Node
	wheelRoll    [4]*core.Node
	wheelShapes  [4]*Shape
	spokeShapes  [4]*Shape
	shapes       []*Shape
	wireframe    bool
	wheelYaw     [4]float32
	wheelRollAng [4]float32

	mode        int // 0..9 postures
	targetMode  int
	postureStep int

	startLowerDOF       [1]float32
	targetLowerDOF      [1]float32
	startUpperDOF       [1]float32
	targetUpperDOF      [1]float32
	startCameraDOF      [1]float32
	targetCameraDOF     [1]float32
	startUpperSideLDOF  [2]float32
	targetUpperSideLDOF [2]float32
	startLowerSideLDOF  [2]float32
	targetLowerSideLDOF [2]float32
	startUpperSideRDOF  [2]float32
	targetUpperSideRDOF [2]float32
	startLowerSideRDOF  [2]float32
	targetLowerSideRDOF [2]float32
	startWristLDOF      [2]float32
	targetWristLDOF     [2]float32
	startWristRDOF      [2]float32
	targetWristRDOF     [2]float32

	startGripL  float32
	startGripR  float32
	targetGripL float32
	targetGripR float32
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

// hingePose computes a child Shape's At relative to its parent Shape's origin,
// placing `childPivot` (in child local coordinates) at `parentJoint` (in parent local coordinates)
// with orientation `rot`.
func hingePose(parentJoint Point3D, rot Quat, childPivot Point3D) At {
	return NewAt(parentJoint, rot).At(AtMove(childPivot.Scale(-1)))
}

func turnsToRad(turns float32) float32 {
	return turns * twoPi
}

func turnsToRatio(turns, minTurns, maxTurns float32) float32 {
	if maxTurns == minTurns {
		return 0
	}
	ratio := (turns - minTurns) / (maxTurns - minTurns)
	if ratio < 0 {
		return 0
	}
	if ratio > 1 {
		return 1
	}
	return ratio
}

// Build constructs every body part of the Z-Bot using ShapeInfo.New() and connects the entire
// upper-body kinematic tree via Hinges, each configured with 1 or 2 physical degrees of freedom.
func (r *Robot) Build() {
	// --- ShapeInfo definities voor alle robotonderdelen ---
	chassisFloorInfo := ShapeInfo{
		Name:     "RobotChassisFloor",
		Material: MatRobotChassis(),
		Steps:    1,
		Move:     []float32{chassisLength},
		Lines:    [][]Point2D{NewLineRect(chassisWidth, chassisFloorHeight)},
	}

	chassisSumpInfo := ShapeInfo{
		Name:     "RobotChassisMechSump",
		Material: MatRobotWheel(),
		Steps:    1,
		Move:     []float32{chassisSumpLength},
		Lines:    [][]Point2D{NewLineRect(chassisSumpWidth, chassisSumpHeight)},
	}

	sideRimInfo := ShapeInfo{
		Name:     "RobotChassisSideRim",
		Material: MatRobotChassis(),
		Steps:    1,
		Move:     []float32{chassisLength},
		Lines:    [][]Point2D{NewLineRect(chassisRimThickness, chassisRimHeight)},
	}

	endRimWidth := chassisWidth - 2*chassisRimThickness
	endRimInfo := ShapeInfo{
		Name:     "RobotChassisEndRim",
		Material: MatRobotChassis(),
		Steps:    1,
		Move:     []float32{chassisRimThickness},
		Lines:    [][]Point2D{NewLineRect(endRimWidth, chassisRimHeight)},
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

	armSegmentInfo := ShapeInfo{
		Name:     "ArmSegment",
		Material: MatRobotArm(),
		Steps:    1,
		Move:     []float32{armDepth},
		Lines:    [][]Point2D{NewLineRect(armWidth, armLength)},
	}

	cameraInfo := ShapeInfo{
		Name:     "CameraHead",
		Material: MatRobotSensor(),
		Steps:    1,
		Move:     []float32{cameraDepth},
		Lines:    [][]Point2D{NewLineRect(cameraWidth, cameraHeight)},
	}

	cameraLensInfo := ShapeInfo{
		Name:     "CameraLens",
		Material: MatRobotWheel(),
		Steps:    16,
		Lines: [][]Point2D{{
			{X: 0.0, Y: 0.0},
			{X: cameraLensRadius, Y: 0.0},
			{X: cameraLensRadius, Y: cameraLensLength},
			{X: 0.0, Y: cameraLensLength},
		}},
	}

	wristInfo := ShapeInfo{
		Name:     "GripperWrist",
		Material: MatRobotChassis(),
		Steps:    1,
		Move:     []float32{wristDepth},
		Lines:    [][]Point2D{NewLineRect(wristWidth, wristLength)},
	}

	fingerInfo := ShapeInfo{
		Name:     "GripperFinger",
		Material: MatRobotSensor(),
		Steps:    1,
		Move:     []float32{fingerDepth},
		Lines:    [][]Point2D{NewLineRect(fingerWidth, fingerLength)},
	}

	rotX := func(turns float32) Quat {
		return NewQuatFromAxisAngle(Point3D{X: 1}, turnsToRad(turns))
	}
	rotY := func(turns float32) Quat {
		return NewQuatFromAxisAngle(Point3D{Y: 1}, turnsToRad(turns))
	}
	rotZ := func(turns float32) Quat {
		return NewQuatFromAxisAngle(Point3D{Z: 1}, turnsToRad(turns))
	}

	// --- A. Basis Chassis: uitstekende mechaniek-bodem + open bakje erbovenop ---
	halfW := chassisWidth * 0.5
	halfL := chassisLength * 0.5
	trayFloorY := chassisFloorBottomY + chassisFloorHeight // Y = +0.21

	chassisAt := AtMove(Point3D{X: -halfW, Y: chassisFloorBottomY, Z: -halfL})
	r.chassis = r.addPart(&r.Node, chassisFloorInfo, chassisAt.Move, chassisAt.Turn)
	r.chassisSump = r.addPart(&r.Node, chassisSumpInfo, Point3D{
		X: -chassisSumpWidth * 0.5,
		Y: chassisFloorBottomY - chassisSumpHeight*0.5,
		Z: -chassisSumpLength * 0.5,
	}, IdentityQuat())

	r.rimLeft = r.addPart(&r.Node, sideRimInfo, Point3D{X: -halfW, Y: trayFloorY, Z: -halfL}, IdentityQuat())
	r.rimRight = r.addPart(&r.Node, sideRimInfo, Point3D{X: halfW - chassisRimThickness, Y: trayFloorY, Z: -halfL}, IdentityQuat())
	r.rimFront = r.addPart(&r.Node, endRimInfo, Point3D{X: -halfW + chassisRimThickness, Y: trayFloorY, Z: halfL - chassisRimThickness}, IdentityQuat())
	r.rimBack = r.addPart(&r.Node, endRimInfo, Point3D{X: -halfW + chassisRimThickness, Y: trayFloorY, Z: -halfL}, IdentityQuat())

	// --- B. Wielen (4 stuks met stuur- en rolhiërarchie + zichtbare spaak) ---
	wheelAlignQuat := rotZ(0.25) // Kwartslag om de wielcilinder op de X-as te leggen
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

	// --- C. Bovenlichaam Shapes ---
	r.lowerCenterArm = r.addPart(&r.Node, armSegmentInfo, Point3D{}, IdentityQuat())
	r.upperCenterArm = r.addPart(&r.Node, armSegmentInfo, Point3D{}, IdentityQuat())
	r.cameraHead = r.addPart(&r.Node, cameraInfo, Point3D{}, IdentityQuat())
	r.cameraLens = r.addPart(&r.Node, cameraLensInfo, Point3D{}, IdentityQuat())
	r.upperSideArmL = r.addPart(&r.Node, armSegmentInfo, Point3D{}, IdentityQuat())
	r.lowerSideArmL = r.addPart(&r.Node, armSegmentInfo, Point3D{}, IdentityQuat())
	r.upperSideArmR = r.addPart(&r.Node, armSegmentInfo, Point3D{}, IdentityQuat())
	r.lowerSideArmR = r.addPart(&r.Node, armSegmentInfo, Point3D{}, IdentityQuat())
	r.wristL = r.addPart(&r.Node, wristInfo, Point3D{}, IdentityQuat())
	r.wristR = r.addPart(&r.Node, wristInfo, Point3D{}, IdentityQuat())
	r.gripFingerL1 = r.addPart(&r.Node, fingerInfo, Point3D{}, IdentityQuat())
	r.gripFingerL2 = r.addPart(&r.Node, fingerInfo, Point3D{}, IdentityQuat())
	r.gripFingerR1 = r.addPart(&r.Node, fingerInfo, Point3D{}, IdentityQuat())
	r.gripFingerR2 = r.addPart(&r.Node, fingerInfo, Point3D{}, IdentityQuat())

	// --- D. Scharnierpunten (Joints & Pivots) ---
	chassisRearJoint := Point3D{
		X: halfW,
		Y: chassisFloorHeight + armDepth*0.5,
		Z: chassisRimThickness + 0.04,
	}

	armBasePivot := Point3D{X: armWidth * 0.5, Y: 0.0, Z: armDepth * 0.5}
	armBaseTopPivot := Point3D{X: armWidth * 0.5, Y: 0.0, Z: 0.0}
	armTipTopJoint := Point3D{X: armWidth * 0.5, Y: armLength, Z: 0.0}

	cameraFoldPivot := Point3D{X: cameraWidth * 0.5, Y: 0.0, Z: 0.0}
	cameraLensMount := Point3D{X: cameraWidth * 0.5, Y: cameraHeight * 0.5, Z: cameraDepth}

	upperCenterSideLeftJoint := Point3D{X: -armWidth*0.5 - 0.01, Y: armLength, Z: armDepth}
	upperCenterSideRightJoint := Point3D{X: armWidth + armWidth*0.5 + 0.01, Y: armLength, Z: armDepth}

	wristPivot := Point3D{X: wristWidth * 0.5, Y: 0.0, Z: wristDepth * 0.5}
	wristJawInnerJoint := Point3D{X: 0.035, Y: wristLength, Z: wristDepth * 0.5}
	wristJawOuterJoint := Point3D{X: wristWidth - 0.035, Y: wristLength, Z: wristDepth * 0.5}
	fingerPivot := Point3D{X: fingerWidth * 0.5, Y: 0.0, Z: fingerDepth * 0.5}

	// =========================================================================
	// 1. Onderste middenarm scharnier: 1 DOF (rotX: voor/achter kantelen)
	// =========================================================================
	r.lowerCenterHinge = NewHinge(
		r.chassis,
		chassisAt,
		r.lowerCenterArm,
		hingePose(chassisRearJoint, rotX(lowerCenterPitchMin), armBasePivot),
	)
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, rotX(lowerCenterPitchMax), armBasePivot))

	// =========================================================================
	// 2. Bovenste middenarm scharnier: 1 DOF (rotX: Z-knik buigen/strekken)
	// =========================================================================
	r.upperCenterHinge = NewHinge(
		r.lowerCenterArm,
		r.lowerCenterHinge.Child.At,
		r.upperCenterArm,
		hingePose(armTipTopJoint, rotX(upperCenterPitchMin), armBaseTopPivot),
	)
	r.upperCenterHinge.AddDOF(0, hingePose(armTipTopJoint, rotX(upperCenterPitchMax), armBaseTopPivot))

	// =========================================================================
	// 3. Camerakop scharnier: 1 DOF (rotX: omhoog/omlaag kijken) + vaste lens
	// =========================================================================
	r.cameraHinge = NewHinge(
		r.upperCenterArm,
		r.upperCenterHinge.Child.At,
		r.cameraHead,
		hingePose(armTipTopJoint, rotX(cameraPitchMin), cameraFoldPivot),
	)
	r.cameraHinge.AddDOF(0, hingePose(armTipTopJoint, rotX(cameraPitchMax), cameraFoldPivot))

	r.cameraLensHinge = NewHinge(
		r.cameraHead,
		r.cameraHinge.Child.At,
		r.cameraLens,
		hingePose(cameraLensMount, rotX(0.25), Point3D{}),
	)

	// =========================================================================
	// 4L. Linker schouder scharnier: 2 DOFs ([0]=rotX voor/achter, [1]=rotZ zijwaarts)
	// =========================================================================
	shoulderLBase := rotX(shoulderPitchMin).Mul(rotZ(shoulderRollLMin))
	r.upperSideHingeL = NewHinge(
		r.upperCenterArm,
		r.upperCenterHinge.Child.At,
		r.upperSideArmL,
		hingePose(upperCenterSideLeftJoint, shoulderLBase, armBasePivot),
	)
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterSideLeftJoint, rotX(shoulderPitchMax).Mul(rotZ(shoulderRollLMin)), armBasePivot))
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterSideLeftJoint, rotX(shoulderPitchMin).Mul(rotZ(shoulderRollLMax)), armBasePivot))

	// =========================================================================
	// 5L. Linker elleboog scharnier: 2 DOFs ([0]=rotX buigen/strekken, [1]=rotZ zijknik)
	// =========================================================================
	elbowBase := rotX(elbowPitchMin).Mul(rotZ(elbowRollMin))
	r.lowerSideHingeL = NewHinge(
		r.upperSideArmL,
		r.upperSideHingeL.Child.At,
		r.lowerSideArmL,
		hingePose(armTipTopJoint, elbowBase, armBaseTopPivot),
	)
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipTopJoint, rotX(elbowPitchMax).Mul(rotZ(elbowRollMin)), armBaseTopPivot))
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipTopJoint, rotX(elbowPitchMin).Mul(rotZ(elbowRollMax)), armBaseTopPivot))

	// =========================================================================
	// 4R. Rechter schouder scharnier: 2 DOFs ([0]=rotX voor/achter, [1]=rotZ zijwaarts)
	// =========================================================================
	shoulderRBase := rotX(shoulderPitchMin).Mul(rotZ(shoulderRollRMin))
	r.upperSideHingeR = NewHinge(
		r.upperCenterArm,
		r.upperCenterHinge.Child.At,
		r.upperSideArmR,
		hingePose(upperCenterSideRightJoint, shoulderRBase, armBasePivot),
	)
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterSideRightJoint, rotX(shoulderPitchMax).Mul(rotZ(shoulderRollRMin)), armBasePivot))
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterSideRightJoint, rotX(shoulderPitchMin).Mul(rotZ(shoulderRollRMax)), armBasePivot))

	// =========================================================================
	// 5R. Rechter elleboog scharnier: 2 DOFs ([0]=rotX buigen/strekken, [1]=rotZ zijknik)
	// =========================================================================
	r.lowerSideHingeR = NewHinge(
		r.upperSideArmR,
		r.upperSideHingeR.Child.At,
		r.lowerSideArmR,
		hingePose(armTipTopJoint, elbowBase, armBaseTopPivot),
	)
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipTopJoint, rotX(elbowPitchMax).Mul(rotZ(elbowRollMin)), armBaseTopPivot))
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipTopJoint, rotX(elbowPitchMin).Mul(rotZ(elbowRollMax)), armBaseTopPivot))

	// =========================================================================
	// 6L. Linker pols scharnier: 2 DOFs ([0]=rotX op/neer, [1]=rotY draaien)
	// =========================================================================
	wristLBase := rotX(wristPitchMin).Mul(rotY(wristYawLMin))
	r.wristHingeL = NewHinge(
		r.lowerSideArmL,
		r.lowerSideHingeL.Child.At,
		r.wristL,
		hingePose(armTipTopJoint, wristLBase, wristPivot),
	)
	r.wristHingeL.AddDOF(0, hingePose(armTipTopJoint, rotX(wristPitchMax).Mul(rotY(wristYawLMin)), wristPivot))
	r.wristHingeL.AddDOF(0, hingePose(armTipTopJoint, rotX(wristPitchMin).Mul(rotY(wristYawLMax)), wristPivot))

	// =========================================================================
	// 6R. Rechter pols scharnier: 2 DOFs ([0]=rotX op/neer, [1]=rotY draaien)
	// =========================================================================
	wristRBase := rotX(wristPitchMin).Mul(rotY(wristYawRMin))
	r.wristHingeR = NewHinge(
		r.lowerSideArmR,
		r.lowerSideHingeR.Child.At,
		r.wristR,
		hingePose(armTipTopJoint, wristRBase, wristPivot),
	)
	r.wristHingeR.AddDOF(0, hingePose(armTipTopJoint, rotX(wristPitchMax).Mul(rotY(wristYawRMin)), wristPivot))
	r.wristHingeR.AddDOF(0, hingePose(armTipTopJoint, rotX(wristPitchMin).Mul(rotY(wristYawRMax)), wristPivot))

	// =========================================================================
	// 7. Grijper-vinger scharnieren: 1 DOF per vinger (open/sluit)
	// =========================================================================
	r.gripHingeL1 = NewHinge(r.wristL, r.wristHingeL.Child.At, r.gripFingerL1, hingePose(wristJawInnerJoint, IdentityQuat(), fingerPivot))
	r.gripHingeL1.AddDOF(0, hingePose(wristJawInnerJoint, rotZ(jawOpenTurns), fingerPivot))

	r.gripHingeL2 = NewHinge(r.wristL, r.wristHingeL.Child.At, r.gripFingerL2, hingePose(wristJawOuterJoint, IdentityQuat(), fingerPivot))
	r.gripHingeL2.AddDOF(0, hingePose(wristJawOuterJoint, rotZ(-jawOpenTurns), fingerPivot))

	r.gripHingeR1 = NewHinge(r.wristR, r.wristHingeR.Child.At, r.gripFingerR1, hingePose(wristJawInnerJoint, IdentityQuat(), fingerPivot))
	r.gripHingeR1.AddDOF(0, hingePose(wristJawInnerJoint, rotZ(jawOpenTurns), fingerPivot))

	r.gripHingeR2 = NewHinge(r.wristR, r.wristHingeR.Child.At, r.gripFingerR2, hingePose(wristJawOuterJoint, IdentityQuat(), fingerPivot))
	r.gripHingeR2.AddDOF(0, hingePose(wristJawOuterJoint, rotZ(-jawOpenTurns), fingerPivot))

	// Initialiseer direct op houding 0 (Ingeklapt in het bakje)
	r.mode = 0
	r.targetMode = 0
	r.postureStep = postureTransitionFrames
	r.applyPostureImmediate(0)
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

func setHingeDOF1(h *Hinge, r0 float32) {
	if h != nil && len(h.DOF) >= 1 {
		h.DOF[0].Ratio = r0
	}
}

func setHingeDOF2(h *Hinge, r0, r1 float32) {
	if h != nil && len(h.DOF) >= 2 {
		h.DOF[0].Ratio = r0
		h.DOF[1].Ratio = r1
	}
}

func setGripperJawRatio(h1, h2 *Hinge, ratio float32) {
	setHingeDOF1(h1, ratio)
	setHingeDOF1(h2, ratio)
}

func lerp1(a, b [1]float32, t float32) float32 {
	return a[0] + (b[0]-a[0])*t
}

func lerp2(a, b [2]float32, t float32) (float32, float32) {
	return a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t
}

func (r *Robot) applyPostureImmediate(mode int) {
	p := robotPostures[mode]

	setHingeDOF1(r.lowerCenterHinge, turnsToRatio(p.LowerCenterPitch, lowerCenterPitchMin, lowerCenterPitchMax))
	setHingeDOF1(r.upperCenterHinge, turnsToRatio(p.UpperCenterPitch, upperCenterPitchMin, upperCenterPitchMax))
	setHingeDOF1(r.cameraHinge, turnsToRatio(p.CameraPitch, cameraPitchMin, cameraPitchMax))

	setHingeDOF2(r.upperSideHingeL,
		turnsToRatio(p.UpperSideL[0], shoulderPitchMin, shoulderPitchMax),
		turnsToRatio(p.UpperSideL[1], shoulderRollLMin, shoulderRollLMax),
	)
	setHingeDOF2(r.lowerSideHingeL,
		turnsToRatio(p.LowerSideL[0], elbowPitchMin, elbowPitchMax),
		turnsToRatio(p.LowerSideL[1], elbowRollMin, elbowRollMax),
	)
	setHingeDOF2(r.upperSideHingeR,
		turnsToRatio(p.UpperSideR[0], shoulderPitchMin, shoulderPitchMax),
		turnsToRatio(p.UpperSideR[1], shoulderRollRMin, shoulderRollRMax),
	)
	setHingeDOF2(r.lowerSideHingeR,
		turnsToRatio(p.LowerSideR[0], elbowPitchMin, elbowPitchMax),
		turnsToRatio(p.LowerSideR[1], elbowRollMin, elbowRollMax),
	)
	setHingeDOF2(r.wristHingeL,
		turnsToRatio(p.WristL[0], wristPitchMin, wristPitchMax),
		turnsToRatio(p.WristL[1], wristYawLMin, wristYawLMax),
	)
	setHingeDOF2(r.wristHingeR,
		turnsToRatio(p.WristR[0], wristPitchMin, wristPitchMax),
		turnsToRatio(p.WristR[1], wristYawRMin, wristYawRMax),
	)

	r.startGripL = p.GripperOpen
	r.targetGripL = p.GripperOpen
	r.startGripR = p.GripperOpen
	r.targetGripR = p.GripperOpen
	setGripperJawRatio(r.gripHingeL1, r.gripHingeL2, p.GripperOpen)
	setGripperJawRatio(r.gripHingeR1, r.gripHingeR2, p.GripperOpen)

	r.updateUpperBodyHinges()
}

func (r *Robot) updateUpperBodyHinges() {
	if r.lowerCenterHinge == nil || r.upperCenterHinge == nil {
		return
	}

	// 1. Centrale 2-delige Z-arm keten
	UpdateHingeChain(r.lowerCenterHinge, r.upperCenterHinge)
	upperWorld := r.upperCenterHinge.Child.At

	// 2. Camerakop + lens aan het uiteinde van de middenarm
	if r.cameraHinge != nil && r.cameraLensHinge != nil {
		r.cameraHinge.Parent.At = upperWorld
		UpdateHingeChain(r.cameraHinge, r.cameraLensHinge)
	}

	// 3. Linker 2-delige zijarm -> linker pols -> linker grijper vingers
	if r.upperSideHingeL != nil && r.lowerSideHingeL != nil && r.wristHingeL != nil {
		r.upperSideHingeL.Parent.At = upperWorld
		UpdateHingeChain(r.upperSideHingeL, r.lowerSideHingeL, r.wristHingeL)
		wristLWorld := r.wristHingeL.Child.At
		if r.gripHingeL1 != nil {
			r.gripHingeL1.Parent.At = wristLWorld
			r.gripHingeL1.Update()
		}
		if r.gripHingeL2 != nil {
			r.gripHingeL2.Parent.At = wristLWorld
			r.gripHingeL2.Update()
		}
	}

	// 4. Rechter 2-delige zijarm -> rechter pols -> rechter grijper vingers
	if r.upperSideHingeR != nil && r.lowerSideHingeR != nil && r.wristHingeR != nil {
		r.upperSideHingeR.Parent.At = upperWorld
		UpdateHingeChain(r.upperSideHingeR, r.lowerSideHingeR, r.wristHingeR)
		wristRWorld := r.wristHingeR.Child.At
		if r.gripHingeR1 != nil {
			r.gripHingeR1.Parent.At = wristRWorld
			r.gripHingeR1.Update()
		}
		if r.gripHingeR2 != nil {
			r.gripHingeR2.Parent.At = wristRWorld
			r.gripHingeR2.Update()
		}
	}
}

// SetGripper sets the left and right gripper open/close DOF ratios (0.0 = closed, 1.0 = open) immediately.
func (r *Robot) SetGripper(leftRatio, rightRatio float32) {
	r.startGripL = leftRatio
	r.targetGripL = leftRatio
	r.startGripR = rightRatio
	r.targetGripR = rightRatio
	setGripperJawRatio(r.gripHingeL1, r.gripHingeL2, leftRatio)
	setGripperJawRatio(r.gripHingeR1, r.gripHingeR2, rightRatio)
	r.updateUpperBodyHinges()
}

// UpdatePosture advances the upper-body hinge interpolation by 1 frame step (out of 100)
// whenever a posture change is in progress.
func (r *Robot) UpdatePosture() {
	if r.postureStep >= postureTransitionFrames {
		return
	}
	r.postureStep++
	t := float32(r.postureStep) / float32(postureTransitionFrames)

	setHingeDOF1(r.lowerCenterHinge, lerp1(r.startLowerDOF, r.targetLowerDOF, t))
	setHingeDOF1(r.upperCenterHinge, lerp1(r.startUpperDOF, r.targetUpperDOF, t))
	setHingeDOF1(r.cameraHinge, lerp1(r.startCameraDOF, r.targetCameraDOF, t))

	uL0, uL1 := lerp2(r.startUpperSideLDOF, r.targetUpperSideLDOF, t)
	setHingeDOF2(r.upperSideHingeL, uL0, uL1)

	lL0, lL1 := lerp2(r.startLowerSideLDOF, r.targetLowerSideLDOF, t)
	setHingeDOF2(r.lowerSideHingeL, lL0, lL1)

	uR0, uR1 := lerp2(r.startUpperSideRDOF, r.targetUpperSideRDOF, t)
	setHingeDOF2(r.upperSideHingeR, uR0, uR1)

	lR0, lR1 := lerp2(r.startLowerSideRDOF, r.targetLowerSideRDOF, t)
	setHingeDOF2(r.lowerSideHingeR, lR0, lR1)

	wL0, wL1 := lerp2(r.startWristLDOF, r.targetWristLDOF, t)
	setHingeDOF2(r.wristHingeL, wL0, wL1)

	wR0, wR1 := lerp2(r.startWristRDOF, r.targetWristRDOF, t)
	setHingeDOF2(r.wristHingeR, wR0, wR1)

	gripL := r.startGripL + (r.targetGripL-r.startGripL)*t
	gripR := r.startGripR + (r.targetGripR-r.startGripR)*t
	setGripperJawRatio(r.gripHingeL1, r.gripHingeL2, gripL)
	setGripperJawRatio(r.gripHingeR1, r.gripHingeR2, gripR)

	r.updateUpperBodyHinges()

	if r.postureStep == postureTransitionFrames {
		r.mode = r.targetMode
	}
}

// SetMode starts a 100-frame hinge transition of the Z-Bot upper body to `newMode` (0..9).
func (r *Robot) SetMode(newMode int) {
	if newMode < 0 || newMode >= postureCount {
		return
	}
	if newMode == r.targetMode && r.postureStep >= postureTransitionFrames {
		return
	}

	r.startLowerDOF[0] = r.lowerCenterHinge.DOF[0].Ratio
	r.startUpperDOF[0] = r.upperCenterHinge.DOF[0].Ratio
	r.startCameraDOF[0] = r.cameraHinge.DOF[0].Ratio

	r.startUpperSideLDOF = [2]float32{r.upperSideHingeL.DOF[0].Ratio, r.upperSideHingeL.DOF[1].Ratio}
	r.startLowerSideLDOF = [2]float32{r.lowerSideHingeL.DOF[0].Ratio, r.lowerSideHingeL.DOF[1].Ratio}
	r.startUpperSideRDOF = [2]float32{r.upperSideHingeR.DOF[0].Ratio, r.upperSideHingeR.DOF[1].Ratio}
	r.startLowerSideRDOF = [2]float32{r.lowerSideHingeR.DOF[0].Ratio, r.lowerSideHingeR.DOF[1].Ratio}
	r.startWristLDOF = [2]float32{r.wristHingeL.DOF[0].Ratio, r.wristHingeL.DOF[1].Ratio}
	r.startWristRDOF = [2]float32{r.wristHingeR.DOF[0].Ratio, r.wristHingeR.DOF[1].Ratio}

	r.startGripL = r.gripHingeL1.DOF[0].Ratio
	r.startGripR = r.gripHingeR1.DOF[0].Ratio

	p := robotPostures[newMode]
	r.targetLowerDOF = [1]float32{turnsToRatio(p.LowerCenterPitch, lowerCenterPitchMin, lowerCenterPitchMax)}
	r.targetUpperDOF = [1]float32{turnsToRatio(p.UpperCenterPitch, upperCenterPitchMin, upperCenterPitchMax)}
	r.targetCameraDOF = [1]float32{turnsToRatio(p.CameraPitch, cameraPitchMin, cameraPitchMax)}

	r.targetUpperSideLDOF = [2]float32{
		turnsToRatio(p.UpperSideL[0], shoulderPitchMin, shoulderPitchMax),
		turnsToRatio(p.UpperSideL[1], shoulderRollLMin, shoulderRollLMax),
	}
	r.targetLowerSideLDOF = [2]float32{
		turnsToRatio(p.LowerSideL[0], elbowPitchMin, elbowPitchMax),
		turnsToRatio(p.LowerSideL[1], elbowRollMin, elbowRollMax),
	}
	r.targetUpperSideRDOF = [2]float32{
		turnsToRatio(p.UpperSideR[0], shoulderPitchMin, shoulderPitchMax),
		turnsToRatio(p.UpperSideR[1], shoulderRollRMin, shoulderRollRMax),
	}
	r.targetLowerSideRDOF = [2]float32{
		turnsToRatio(p.LowerSideR[0], elbowPitchMin, elbowPitchMax),
		turnsToRatio(p.LowerSideR[1], elbowRollMin, elbowRollMax),
	}
	r.targetWristLDOF = [2]float32{
		turnsToRatio(p.WristL[0], wristPitchMin, wristPitchMax),
		turnsToRatio(p.WristL[1], wristYawLMin, wristYawLMax),
	}
	r.targetWristRDOF = [2]float32{
		turnsToRatio(p.WristR[0], wristPitchMin, wristPitchMax),
		turnsToRatio(p.WristR[1], wristYawRMin, wristYawRMax),
	}

	r.targetGripL = p.GripperOpen
	r.targetGripR = p.GripperOpen
	r.targetMode = newMode
	r.postureStep = 0
}
