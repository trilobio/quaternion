package quaternion

import (
	"math"
	"testing"

	"gonum.org/v1/gonum/mat"
)

func TestVecSum(t *testing.T) {
	v1 := Vec3{1, 2, 3}
	v2 := Vec3{4, 5, 6}

	d := mat.Dot(v1, v2)
	if (math.Abs(d) - 32) > 1e-9 {
		t.Errorf("Dot product of %v and %v is %v; want %v", v1, v2, d, 32)
	}
}

func TestSumScal(t *testing.T) {
	v1 := Vec3{1, 2, 3}
	scalar := 3.0
	v2 := v1.SumScal(scalar)
	vTarg := Vec3{4, 5, 6}

	if !v2.ApproxEquals(vTarg, 1e-7) {
		t.Errorf("Adding vector %v to scalar %v gives %v; want %v", v1, scalar,
			v2, vTarg)
	}
}

func TestVecNormalize(t *testing.T) {
	v1 := Vec3{1, 2, 3}
	v1.Normalize()
	norm := v1.Norm()
	if norm-1 > 1e-7 {
		t.Errorf("Norm of %v expected to be 1, got %v", v1, norm)
	}
}

func TestVecNorm(t *testing.T) {
	v1 := Vec3{1, 2, 3}
	norm := v1.Norm()
	normTarg := math.Sqrt(v1.X*v1.X + v1.Y*v1.Y + v1.Z*v1.Z)
	if math.Abs(norm-normTarg) > 1e-7 {
		t.Errorf("Norm of vector %v calculated to be %v; want %v", v1,
			norm, normTarg)
	}
}

func TestVectorMethods(t *testing.T) {
	v1 := Vec3{1, 2, 3}
	idx := 2
	v1AtIdx := v1.At(2, 0)
	v1AtTarg := v1.Z
	if math.Abs(v1AtIdx-v1AtTarg) > 1e-7 {
		t.Errorf("Element at index %v of vector %v is %v; want %v", idx, v1,
			v1AtIdx, v1AtTarg)
	}

	causePanic := func() {
		v1.At(3, 0)
	}

	assertPanic(t, causePanic)

	causePanic = func() {
		v1.At(0, 1)
	}
	assertPanic(t, causePanic)

	// Check transpose
	v1T := v1.T()
	idx = 2
	v1TAtIdx := v1T.At(0, 2)
	if math.Abs(v1AtIdx-v1AtTarg) > 1e-7 {
		t.Errorf("Element at index %v of vector %v is %v; want %v", idx, v1T,
			v1TAtIdx, v1AtTarg)
	}

	r, c := v1.Dims()
	if r != 3 || c != 1 {
		t.Errorf("Dimensions of Vec3 are (%v, %v); want (3, 1)", r, c)
	}

	v1AtVecIdx := v1.AtVec(idx)
	if math.Abs(v1AtVecIdx-v1AtTarg) > 1e-7 {
		t.Errorf("Element at index %v of vector %v is %v; want %v", idx, v1T,
			v1TAtIdx, v1AtTarg)
	}

	causePanic = func() {
		v1.AtVec(3)
	}
	assertPanic(t, causePanic)

	causePanic = func() {
		v1.AtVec(-1)
	}
	assertPanic(t, causePanic)

}

func TestVecApproxEquals(t *testing.T) {
	var v1 = Vec3{1, 2, 3}
	var v2 = Vec3{1 + 1e-5, 2, 3 - 1e-5}
	var epsilon = 1e-4
	if !(v1.ApproxEquals(v2, epsilon)) {
		t.Errorf("Approximate Equality expected for vecs %v and %v at"+
			" epsilon %v", v1, v2, epsilon)
	}

	epsilon = 1e-6
	if v1.ApproxEquals(v2, epsilon) {
		t.Errorf("Approximate equality not expected for quats %v and %v at"+
			" epsilon %v", v1, v2, epsilon)
	}
}

func assertPanic(t *testing.T, f func()) {
	defer func() {
		if r := recover(); r == nil {
			t.Errorf("The code did not panic")
		}
	}()
	f()
}
