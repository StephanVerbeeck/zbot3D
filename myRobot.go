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

	// Equal dimensions shared by both center arm segments and all four side arm segments.
	armWidth  float32 = 0.24
	armLength float32 = 1.40
	armDepth  float32 = 0.24

	// Small camera head mounted at the top center end of the upper center arm.
	cameraWidth      float32 = 0.18
	cameraHeight     float32 = 0.14
	cameraDepth      float32 = 0.16
	cameraLensRadius float32 = 0.045
	cameraLensLength float32 = 0.06

	wristWidth  float32 = 0.22
	wristLength float32 = 0.14
	wristDepth  float32 = 0.16

	fingerWidth  float32 = 0.045
	fingerLength float32 = 0.24
	fingerDepth  float32 = 0.12

	postureTransitionFrames = 100
	postureDOFCount         = 9 // Modes 1..9 (Mode 0 is FromAt)
)

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

	mode                  int // 0..9 postures
	targetMode            int
	postureStep           int
	startLowerRatios      [postureDOFCount]float32
	startUpperRatios      [postureDOFCount]float32
	startCameraRatios     [postureDOFCount]float32
	startUpperSideLRatios [postureDOFCount]float32
	startLowerSideLRatios [postureDOFCount]float32
	startUpperSideRRatios [postureDOFCount]float32
	startLowerSideRRatios [postureDOFCount]float32
	startWristLRatios     [postureDOFCount]float32
	startWristRRatios     [postureDOFCount]float32
	targetRatios          [postureDOFCount]float32
	startGripL            float32
	startGripR            float32
	targetGripL           float32
	targetGripR           float32
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

// Build constructs every body part of the Z-Bot using ShapeInfo.New() and connects the entire
// upper-body kinematic tree (2-part center arm -> camera head + 2-part left/right arms -> wrists -> grippers) via Hinges.
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

	// --- C. Bovenlichaam Shapes (2-delige Z-middenarm + camerakop + 2x 2-delige zijarmen + polsen + grijpers) ---
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
	// Achterkant van het chassis (Z = -0.82 in robotruimte -> Z = 0.18 in chassisruimte)
	chassisRearJoint := Point3D{X: 0.60, Y: 0.30, Z: 0.18}

	// Alle 6 armsegmenten delen exact dezelfde dimensies (armWidth x armLength x armDepth)
	armBasePivot := Point3D{X: armWidth * 0.5, Y: 0.0, Z: armDepth * 0.5}
	armTipJoint := Point3D{X: armWidth * 0.5, Y: armLength, Z: armDepth * 0.5}

	// Camerakop bovenop het midden van het uiteinde van de bovenste middenarm
	cameraPivot := Point3D{X: cameraWidth * 0.5, Y: 0.0, Z: cameraDepth * 0.5}
	cameraLensMount := Point3D{X: cameraWidth * 0.5, Y: cameraHeight * 0.5, Z: cameraDepth}

	// Links en rechts bovenop het uiteinde van de bovenste middenarm
	upperCenterTopLeftJoint := Point3D{X: -armWidth*0.5 - 0.01, Y: armLength, Z: armDepth * 0.5}
	upperCenterTopRightJoint := Point3D{X: armWidth + armWidth*0.5 + 0.01, Y: armLength, Z: armDepth * 0.5}

	wristPivot := Point3D{X: wristWidth * 0.5, Y: 0.0, Z: wristDepth * 0.5}
	wristJawInnerJoint := Point3D{X: 0.035, Y: wristLength, Z: wristDepth * 0.5}
	wristJawOuterJoint := Point3D{X: wristWidth - 0.035, Y: wristLength, Z: wristDepth * 0.5}
	fingerPivot := Point3D{X: fingerWidth * 0.5, Y: 0.0, Z: fingerDepth * 0.5}

	pitchX := func(rad float32) Quat {
		return NewQuatFromAxisAngle(Point3D{X: 1}, rad)
	}
	yawY := func(rad float32) Quat {
		return NewQuatFromAxisAngle(Point3D{Y: 1}, rad)
	}
	rollZ := func(rad float32) Quat {
		return NewQuatFromAxisAngle(Point3D{Z: 1}, rad)
	}

	// =========================================================================
	// 1. Onderste middenarm hinge (verbonden aan de achterkant van het chassis)
	// =========================================================================
	r.lowerCenterHinge = NewHinge(
		r.chassis,
		chassisAt,
		r.lowerCenterArm,
		hingePose(chassisRearJoint, pitchX(1.52), armBasePivot), // [0] Ingeklapt naar voren
	)
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(0.75), armBasePivot))  // [1] Klassieke Z-vorm
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(0.12), armBasePivot))  // [2] Zeer hoog reikend
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(0.45), armBasePivot))  // [3] Zeer breed (T-pose)
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(1.15), armBasePivot))  // [4] Zeer laag vooruit
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(0.95), armBasePivot))  // [5] Laag & breed zijwaarts
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(1.25), armBasePivot))  // [6] Ver vooruit reikend
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(-0.35), armBasePivot)) // [7] Achterwaarts kraan
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(0.95), armBasePivot))  // [8] Hand-lopen basis (links/rechts aan grond)
	r.lowerCenterHinge.AddDOF(0, hingePose(chassisRearJoint, pitchX(0.92), armBasePivot))  // [9] Hand-lopen stap (asymmetrisch aan grond)

	// =========================================================================
	// 2. Bovenste middenarm hinge (vormt samen met onderste arm en wielbasis de Z-vorm)
	// =========================================================================
	r.upperCenterHinge = NewHinge(
		r.lowerCenterArm,
		r.lowerCenterHinge.Child.At,
		r.upperCenterArm,
		hingePose(armTipJoint, pitchX(-3.04), armBasePivot), // [0] Teruggevouwen naar achteren
	)
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-1.50), armBasePivot)) // [1] Klassieke Z-knik
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-0.24), armBasePivot)) // [2] Zeer hoog gestrekt
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-0.90), armBasePivot)) // [3] Zeer breed (T-pose)
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-1.65), armBasePivot)) // [4] Zeer laag vooruit
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-1.55), armBasePivot)) // [5] Laag & breed zijwaarts
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-1.35), armBasePivot)) // [6] Ver vooruit reikend
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(0.65), armBasePivot))  // [7] Achterwaarts kraan
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-1.70), armBasePivot)) // [8] Hand-lopen basis
	r.upperCenterHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-1.68), armBasePivot)) // [9] Hand-lopen stap

	// =========================================================================
	// 2b. Camerakop hinge + lens (gemonteerd aan het uiteinde van de middenarm)
	// =========================================================================
	r.cameraHinge = NewHinge(
		r.upperCenterArm,
		r.upperCenterHinge.Child.At,
		r.cameraHead,
		hingePose(armTipJoint, IdentityQuat(), cameraPivot), // [0] Ingeklapt in lijn met middenarm
	)
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(2.15), cameraPivot))  // [1] Gericht op Z-werkgebied
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(0.95), cameraPivot))  // [2] Omhoog kijkend naar hoge grijpers
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(2.02), cameraPivot))  // [3] Horizontaal vooruit (T-pose)
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(2.55), cameraPivot))  // [4] Omlaag kijkend naar lage grijpers
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(2.55), cameraPivot))  // [5] Omlaag & vooruit
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(1.70), cameraPivot))  // [6] Recht vooruit langs lange reikwijdte
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(-2.10), cameraPivot)) // [7] Achteruit kijkend (kraan)
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(2.60), cameraPivot))  // [8] Vloerzicht vooruit (hand-lopen)
	r.cameraHinge.AddDOF(0, hingePose(armTipJoint, pitchX(2.60), cameraPivot))  // [9] Vloerzicht vooruit (hand-lopen stap)

	r.cameraLensHinge = NewHinge(
		r.cameraHead,
		r.cameraHinge.Child.At,
		r.cameraLens,
		hingePose(cameraLensMount, pitchX(math.Pi/2), Point3D{}),
	)

	// =========================================================================
	// 3L. Linker bovenarm hinge (gemonteerd links bovenop het uiteinde van de middenarm)
	// =========================================================================
	r.upperSideHingeL = NewHinge(
		r.upperCenterArm,
		r.upperCenterHinge.Child.At,
		r.upperSideArmL,
		hingePose(upperCenterTopLeftJoint, pitchX(3.04), armBasePivot), // [0] Ingeklapt
	)
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(1.75).Mul(rollZ(0.12)), armBasePivot))  // [1] Werkhouding
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(0.15).Mul(rollZ(0.08)), armBasePivot))  // [2] Zeer hoog reikend
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(0.45).Mul(rollZ(1.55)), armBasePivot))  // [3] Zeer breed horizontaal links
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(2.45).Mul(rollZ(0.22)), armBasePivot))  // [4] Zeer laag vooruit
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(2.65).Mul(rollZ(0.85)), armBasePivot))  // [5] Laag & breed zijwaarts
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(1.67).Mul(rollZ(0.14)), armBasePivot))  // [6] Ver vooruit reikend
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(-1.35).Mul(rollZ(0.35)), armBasePivot)) // [7] Achterwaarts kraan
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(2.82).Mul(rollZ(0.62)), armBasePivot))  // [8] Hand-lopen basis (links aan grond)
	r.upperSideHingeL.AddDOF(0, hingePose(upperCenterTopLeftJoint, pitchX(2.52).Mul(rollZ(0.58)), armBasePivot))  // [9] Hand-lopen stap (links vooruit aan grond)

	// =========================================================================
	// 4L. Linker onderarm hinge (tweede deel van de linker zijarm)
	// =========================================================================
	r.lowerSideHingeL = NewHinge(
		r.upperSideArmL,
		r.upperSideHingeL.Child.At,
		r.lowerSideArmL,
		hingePose(armTipJoint, pitchX(-3.04), armBasePivot), // [0] Ingeklapt
	)
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(-0.95), armBasePivot))                   // [1] Werkhouding
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(-0.10).Mul(rollZ(-0.06)), armBasePivot)) // [2] Zeer hoog gestrekt
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, rollZ(0.04), armBasePivot))                     // [3] Zeer breed gestrekt
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(0.75).Mul(rollZ(-0.12)), armBasePivot))  // [4] Zeer laag naar vloer voor
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(0.55).Mul(rollZ(0.18)), armBasePivot))   // [5] Laag & breed naar vloer zij
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, rollZ(-0.10), armBasePivot))                    // [6] Ver vooruit gestrekt
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(-0.82).Mul(rollZ(-0.20)), armBasePivot)) // [7] Achterwaarts kraan
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, rollZ(-0.62).Mul(pitchX(1.07)), armBasePivot))  // [8] Hand-lopen basis (verticaal naar grond)
	r.lowerSideHingeL.AddDOF(0, hingePose(armTipJoint, rollZ(-0.58).Mul(pitchX(1.38)), armBasePivot))  // [9] Hand-lopen stap (links aan grond)

	// =========================================================================
	// 3R. Rechter bovenarm hinge (gemonteerd rechts bovenop het uiteinde van de middenarm)
	// =========================================================================
	r.upperSideHingeR = NewHinge(
		r.upperCenterArm,
		r.upperCenterHinge.Child.At,
		r.upperSideArmR,
		hingePose(upperCenterTopRightJoint, pitchX(3.04), armBasePivot), // [0] Ingeklapt
	)
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(1.75).Mul(rollZ(-0.12)), armBasePivot))  // [1] Werkhouding
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(0.15).Mul(rollZ(-0.08)), armBasePivot))  // [2] Zeer hoog reikend
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(0.45).Mul(rollZ(-1.55)), armBasePivot))  // [3] Zeer breed horizontaal rechts
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(2.45).Mul(rollZ(-0.22)), armBasePivot))  // [4] Zeer laag vooruit
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(2.65).Mul(rollZ(-0.85)), armBasePivot))  // [5] Laag & breed zijwaarts
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(1.67).Mul(rollZ(-0.14)), armBasePivot))  // [6] Ver vooruit reikend
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(-1.35).Mul(rollZ(-0.35)), armBasePivot)) // [7] Achterwaarts kraan
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(2.82).Mul(rollZ(-0.62)), armBasePivot))  // [8] Hand-lopen basis (rechts aan grond)
	r.upperSideHingeR.AddDOF(0, hingePose(upperCenterTopRightJoint, pitchX(3.15).Mul(rollZ(-0.65)), armBasePivot))  // [9] Hand-lopen stap (rechts achter aan grond)

	// =========================================================================
	// 4R. Rechter onderarm hinge (tweede deel van de rechter zijarm)
	// =========================================================================
	r.lowerSideHingeR = NewHinge(
		r.upperSideArmR,
		r.upperSideHingeR.Child.At,
		r.lowerSideArmR,
		hingePose(armTipJoint, pitchX(-3.04), armBasePivot), // [0] Ingeklapt
	)
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(-0.95), armBasePivot))                  // [1] Werkhouding
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(-0.10).Mul(rollZ(0.06)), armBasePivot)) // [2] Zeer hoog gestrekt
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, rollZ(-0.04), armBasePivot))                   // [3] Zeer breed gestrekt
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(0.75).Mul(rollZ(0.12)), armBasePivot))  // [4] Zeer laag naar vloer voor
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(0.55).Mul(rollZ(-0.18)), armBasePivot)) // [5] Laag & breed naar vloer zij
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, rollZ(0.10), armBasePivot))                    // [6] Ver vooruit gestrekt
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(-0.82).Mul(rollZ(0.20)), armBasePivot)) // [7] Achterwaarts kraan
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, rollZ(0.62).Mul(pitchX(1.07)), armBasePivot))  // [8] Hand-lopen basis (verticaal naar grond)
	r.lowerSideHingeR.AddDOF(0, hingePose(armTipJoint, rollZ(0.65).Mul(pitchX(0.75)), armBasePivot))  // [9] Hand-lopen stap (rechts aan grond)

	// =========================================================================
	// 5L. Linker pols hinge (verbonden tussen linker onderarm en linker grijper)
	// =========================================================================
	r.wristHingeL = NewHinge(
		r.lowerSideArmL,
		r.lowerSideHingeL.Child.At,
		r.wristL,
		hingePose(armTipJoint, IdentityQuat(), wristPivot), // [0] Recht ingeklapt
	)
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(0.55), wristPivot))      // [1] Werkhouding
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, IdentityQuat(), wristPivot))    // [2] Hoog reikend
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, yawY(0.40), wristPivot))        // [3] Breed uitgespreid
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(-0.85), wristPivot))     // [4] Laag horizontaal vooruit
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, rollZ(-0.35), wristPivot))      // [5] Laag & breed zijwaarts
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, yawY(0.20), wristPivot))        // [6] Ver vooruit reikend
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, pitchX(-0.35), wristPivot))     // [7] Achterwaarts kraan
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, yawY(math.Pi*0.5), wristPivot)) // [8] Hand-lopen steunvoet op grond
	r.wristHingeL.AddDOF(0, hingePose(armTipJoint, yawY(math.Pi*0.5), wristPivot)) // [9] Hand-lopen stap op grond

	// =========================================================================
	// 5R. Rechter pols hinge (verbonden tussen rechter onderarm en rechter grijper)
	// =========================================================================
	r.wristHingeR = NewHinge(
		r.lowerSideArmR,
		r.lowerSideHingeR.Child.At,
		r.wristR,
		hingePose(armTipJoint, IdentityQuat(), wristPivot), // [0] Recht ingeklapt
	)
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(0.55), wristPivot))       // [1] Werkhouding
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, IdentityQuat(), wristPivot))     // [2] Hoog reikend
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, yawY(-0.40), wristPivot))        // [3] Breed uitgespreid
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(-0.85), wristPivot))      // [4] Laag horizontaal vooruit
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, rollZ(0.35), wristPivot))        // [5] Laag & breed zijwaarts
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, yawY(-0.20), wristPivot))        // [6] Ver vooruit reikend
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, pitchX(-0.35), wristPivot))      // [7] Achterwaarts kraan
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, yawY(-math.Pi*0.5), wristPivot)) // [8] Hand-lopen steunvoet op grond
	r.wristHingeR.AddDOF(0, hingePose(armTipJoint, yawY(-math.Pi*0.5), wristPivot)) // [9] Hand-lopen stap op grond

	// =========================================================================
	// 6. Grijper-vinger hinges met eigen open/sluit DOF (0.0 = dicht, 1.0 = open)
	// =========================================================================
	const jawOpenAngle float32 = 0.52

	r.gripHingeL1 = NewHinge(r.wristL, r.wristHingeL.Child.At, r.gripFingerL1, hingePose(wristJawInnerJoint, IdentityQuat(), fingerPivot))
	r.gripHingeL1.AddDOF(0, hingePose(wristJawInnerJoint, rollZ(jawOpenAngle), fingerPivot))

	r.gripHingeL2 = NewHinge(r.wristL, r.wristHingeL.Child.At, r.gripFingerL2, hingePose(wristJawOuterJoint, IdentityQuat(), fingerPivot))
	r.gripHingeL2.AddDOF(0, hingePose(wristJawOuterJoint, rollZ(-jawOpenAngle), fingerPivot))

	r.gripHingeR1 = NewHinge(r.wristR, r.wristHingeR.Child.At, r.gripFingerR1, hingePose(wristJawInnerJoint, IdentityQuat(), fingerPivot))
	r.gripHingeR1.AddDOF(0, hingePose(wristJawInnerJoint, rollZ(jawOpenAngle), fingerPivot))

	r.gripHingeR2 = NewHinge(r.wristR, r.wristHingeR.Child.At, r.gripFingerR2, hingePose(wristJawOuterJoint, IdentityQuat(), fingerPivot))
	r.gripHingeR2.AddDOF(0, hingePose(wristJawOuterJoint, rollZ(-jawOpenAngle), fingerPivot))

	// Start in modus 0 (Ingeklapt)
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

func modeGripperOpen(mode int) float32 {
	switch mode {
	case 1: // Z-Werkhouding
		return 0.45
	case 2: // Zeer hoog reikend
		return 1.0
	case 3: // Zeer breed uitgespreid
		return 0.85
	case 4: // Zeer laag vooruit
		return 0.90
	case 5: // Laag & breed zijwaarts
		return 0.75
	case 6: // Ver vooruit reikend
		return 0.80
	case 7: // Achterwaarts kraan
		return 0.50
	case 8, 9: // Hand-locomotie (wijde steunvoeten op de grond)
		return 1.0
	default: // 0: Ingeklapt
		return 0.0
	}
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

func setGripperJawRatio(h1, h2 *Hinge, ratio float32) {
	if h1 != nil && len(h1.DOF) > 0 {
		h1.DOF[0].Ratio = ratio
	}
	if h2 != nil && len(h2.DOF) > 0 {
		h2.DOF[0].Ratio = ratio
	}
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

	applyLerpedDOFRatios(r.lowerCenterHinge, r.startLowerRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.upperCenterHinge, r.startUpperRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.cameraHinge, r.startCameraRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.upperSideHingeL, r.startUpperSideLRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.lowerSideHingeL, r.startLowerSideLRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.upperSideHingeR, r.startUpperSideRRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.lowerSideHingeR, r.startLowerSideRRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.wristHingeL, r.startWristLRatios, r.targetRatios, t)
	applyLerpedDOFRatios(r.wristHingeR, r.startWristRRatios, r.targetRatios, t)

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
	if newMode < 0 || newMode > postureDOFCount {
		return
	}
	if newMode == r.targetMode && r.postureStep >= postureTransitionFrames {
		return
	}

	r.startLowerRatios = captureDOFRatios(r.lowerCenterHinge)
	r.startUpperRatios = captureDOFRatios(r.upperCenterHinge)
	r.startCameraRatios = captureDOFRatios(r.cameraHinge)
	r.startUpperSideLRatios = captureDOFRatios(r.upperSideHingeL)
	r.startLowerSideLRatios = captureDOFRatios(r.lowerSideHingeL)
	r.startUpperSideRRatios = captureDOFRatios(r.upperSideHingeR)
	r.startLowerSideRRatios = captureDOFRatios(r.lowerSideHingeR)
	r.startWristLRatios = captureDOFRatios(r.wristHingeL)
	r.startWristRRatios = captureDOFRatios(r.wristHingeR)
	if r.gripHingeL1 != nil && len(r.gripHingeL1.DOF) > 0 {
		r.startGripL = r.gripHingeL1.DOF[0].Ratio
	}
	if r.gripHingeR1 != nil && len(r.gripHingeR1.DOF) > 0 {
		r.startGripR = r.gripHingeR1.DOF[0].Ratio
	}

	r.targetRatios = modeRatios(newMode)
	gripTarget := modeGripperOpen(newMode)
	r.targetGripL = gripTarget
	r.targetGripR = gripTarget
	r.targetMode = newMode
	r.postureStep = 0
}
