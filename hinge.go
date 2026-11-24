package main

// Hinge connects a Parent and Child (*Shape + At) through one or more degrees of freedom (DOF).
// FromAt defines the child's base pose (at Ratio = 0), and each DOF[i] defines a target pose
// interpolated via LerpAt(h.FromAt, h.DOF[i].FromAt, h.DOF[i].Ratio).
type Hinge struct {
	Parent, Child struct {
		*Shape
		At
	}
	FromAt At
	DOF    []struct {
		Ratio float32
		At
	}
}

// NewHinge creates a Hinge connecting `parent` (at `parentAt`) to `child` (with base pose `fromAt`),
// and immediately positions `child` at its initial pose.
func NewHinge(parent *Shape, parentAt At, child *Shape, fromAt At) *Hinge {
	h := &Hinge{
		FromAt: fromAt,
	}
	h.Parent.Shape = parent
	h.Parent.At = parentAt
	h.Child.Shape = child
	h.Child.At = fromAt
	h.Update()
	return h
}

// AddDOF appends a new degree of freedom with an initial `ratio` and target `fromAt`,
// updates the child transform, and returns the new DOF index.
func (h *Hinge) AddDOF(ratio float32, fromAt At) int {
	h.DOF = append(h.DOF, struct {
		Ratio float32
		At
	}{
		Ratio: ratio,
		At:    fromAt,
	})
	h.Update()
	return len(h.DOF) - 1
}

// SetRatio sets the interpolation ratio for DOF `index` and updates the child pose.
func (h *Hinge) SetRatio(index int, ratio float32) {
	if index < 0 || index >= len(h.DOF) {
		return
	}
	h.DOF[index].Ratio = ratio
	h.Update()
}

// SetRatios sets the interpolation ratios for multiple DOFs in order and updates the child pose.
func (h *Hinge) SetRatios(ratios ...float32) {
	n := len(ratios)
	if len(h.DOF) < n {
		n = len(h.DOF)
	}
	for i := 0; i < n; i++ {
		h.DOF[i].Ratio = ratios[i]
	}
	h.Update()
}

// At computes the child's At transform relative to Parent.At by starting at FromAt
// and composing the lerp between Hinge.FromAt and Hinge.DOF[i].FromAt for every DOF.
func (h *Hinge) At() At {
	if len(h.DOF) == 0 {
		return h.FromAt.Normalized()
	}

	// First DOF: direct lerp between Hinge.FromAt and Hinge.DOF[0].FromAt
	acc := LerpAt(h.FromAt, h.DOF[0].At, h.DOF[0].Ratio)

	// Each extra DOF[i] introduces a lerp between Hinge.FromAt and Hinge.DOF[i].FromAt,
	// applied as a relative delta transform on top of the accumulated pose.
	baseInv := h.FromAt.Inverse()
	for i := 1; i < len(h.DOF); i++ {
		dofPose := LerpAt(h.FromAt, h.DOF[i].At, h.DOF[i].Ratio)
		delta := baseInv.At(dofPose)
		acc = acc.At(delta)
	}
	return acc
}

// WorldAt returns the child's resulting world-space At transform (`h.Parent.At.At(h.LocalAt())`).
func (h *Hinge) WorldAt() At {
	return h.Parent.At.At(h.At())
}

// Update evaluates the hinge's current WorldAt, stores it in `h.Child.At`,
// and applies it to `h.Child.Shape` if non-nil.
func (h *Hinge) Update() At {
	world := h.WorldAt()
	h.Child.At = world
	if h.Child.Shape != nil {
		SetShapeAt(h.Child.Shape, world)
	}
	return world
}

// UpdateHingeChain updates a parent-to-child sequence of hinges in order,
// forwarding each hinge's resulting Child.At into the next hinge's Parent.At.
func UpdateHingeChain(hinges ...*Hinge) {
	for i, h := range hinges {
		if h == nil {
			continue
		}
		if i > 0 && hinges[i-1] != nil {
			h.Parent.At = hinges[i-1].Child.At
		}
		h.Update()
	}
}
