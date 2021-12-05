package quaternion

import (
	"fmt"
	"math"
	"testing"

	"gonum.org/v1/gonum/mat"
)

func TestRotVec(t *testing.T) {
	var q = Quat{0, 1, 0, 0}
	var v = Vec3{1, 2, 3}
	var vTarg = Vec3{1, -2, -3}

	var rotV = q.RotVec(v)
	if !(rotV.ApproxEquals(vTarg, 1e-7)) {
		t.Errorf("Rotated Vector = %v; want %v", rotV, vTarg)
	}

	q = Quat{0.5, 0.5, 0.5, 0.5}
	vTarg = Vec3{3, 1, 2}
	rotV = q.RotVec(v)
	if !(rotV.ApproxEquals(vTarg, 1e-7)) {
		t.Errorf("Rotated Vector = %v; want %v", rotV, vTarg)
	}
}

func TestMulQuat(t *testing.T) {
	var q1 = Quat{0.653281482438188, 0.270598050073098, -0.653281482438188,
		0.270598050073099}
	var q2 = Quat{-0.168953174898454, 0.490392640201615, 0.0975451610080642,
		0.849384968487042}
	var qTarg = Quat{-0.40919074161503, -0.306636970450269, 0.0769558039230822,
		0.855929033023284}
	var mulRes = q1.MulQuat(q2)
	if !(mulRes.ApproxEquals(qTarg, 1e-7)) {
		t.Errorf("Multiplied Quat = %v; want %v", mulRes, qTarg)
	}
}

func TestNormalize(t *testing.T) {
	var q1 = Quat{1, 2, 3, 4}
	q1.Normalize()
	if !(math.Abs(q1.Norm()-1) < 1e-5) {
		t.Errorf("Normalizing Quat has norm = %v; want 1", q1.Norm())
	}
}

func TestIdentity(t *testing.T) {
	var q1 Quat
	var qTarg = Quat{1, 0, 0, 0}
	q1.Identity()
	if !(q1.ApproxEquals(qTarg, 1e-7)) {
		t.Errorf("Identity Quat is %v; want %v", q1, qTarg)
	}
}

func TestApproxEquals(t *testing.T) {
	var q1 = Quat{1, 2, 3, 4}
	var q2 = Quat{1 + 1e-5, 2, 3 - 1e-5, 4}
	var epsilon = 1e-4
	if !(q1.ApproxEquals(q2, epsilon)) {
		t.Errorf("Approximate Equality expected for quats %v and %v at"+
			" epsilon %v", q1, q2, epsilon)
	}

	epsilon = 1e-6
	if q1.ApproxEquals(q2, epsilon) {
		t.Errorf("Approximate equality not expected for quats %v and %v at"+
			" epsilon %v", q1, q2, epsilon)
	}
}

func TestConj(t *testing.T) {
	q1 := Quat{1, 2, 3, 4}
	q1.Normalize()
	q2 := q1.Conjugate()
	v := Vec3{5, 3, 1}
	vAug := v.PureQuat()
	vRot := q1.MulQuat(vAug).MulQuat(q2).ExtractVec()
	vRotTarg := q1.RotVec(v)

	if !vRotTarg.ApproxEquals(vRot, 1e-7) {
		t.Errorf("q1 * v * q1' is %v; want %v", vRot, vRotTarg)
	}
}

func TestArgMax(t *testing.T) {
	var arr = []float64{}
	idx := argMax(arr)
	if idx != -1 {
		t.Errorf("Index of -1 expected for argMax of empty array, got %v", idx)
	}
	arr = []float64{1, 3, 2}
	idx = argMax(arr)
	if idx != 1 {
		t.Errorf("Index of 1 expected for argMax of %v, got %v", arr, idx)
	}

	arr = []float64{1, 2, 3}
	idx = argMax(arr)
	if idx != 2 {
		t.Errorf("Index of 2 expected for argMax of %v, got %v", arr, idx)
	}

	arr = []float64{1, 2, 2}
	idx = argMax(arr)
	if idx != 1 {
		t.Errorf("Index of 1 expected for argMax of %v, got %v", arr, idx)
	}
}

func TestToRotMat(t *testing.T) {
	var q1 = Quat{1, 0, 0, 0}

	R := q1.ToRotMat()
	RTarg := eye(3)
	if !mat.EqualApprox(R, RTarg, 1e-9) {
		t.Errorf("Rotation matrix from identity quaternion is = %v; want %v",
			matFmt(R), matFmt(RTarg))
	}

	q1 = Quat{1, 2, 3, 4}
	q1.Normalize()
	R = q1.ToRotMat()
	RTargData := []float64{
		-0.66666666666667, 0.13333333333333, 0.73333333333333,
		0.66666666666667, -0.33333333333333, 0.66666666666667,
		0.33333333333333, 0.93333333333333, 0.13333333333333,
	}
	RTarg = mat.NewDense(3, 3, RTargData)
	if !mat.EqualApprox(R, RTarg, 1e-6) {
		t.Errorf("Rotation matrix from quaternion is = %v; want %v",
			matFmt(R), matFmt(RTarg))
	}

	q1 = Quat{-3, 0, 1, 2}
	q1.Normalize()
	R = q1.ToRotMat()
	RTargData = []float64{
		0.28571429, 0.85714286, -0.42857143,
		-0.85714286, 0.42857143, 0.28571429,
		0.42857143, 0.28571429, 0.85714286,
	}
	RTarg = mat.NewDense(3, 3, RTargData)
	if !mat.EqualApprox(R, RTarg, 1e-6) {
		t.Errorf("Rotation matrix from quaternion is = %v; want %v",
			matFmt(R), matFmt(RTarg))
	}
}

func TestQuatFromRotMat(t *testing.T) {
	var epsilon = 1e-6
	var QComp Quat

	// Test the first branch path of QuatFromRotMat
	RMatDat := []float64{
		0.5461756, 0.81975061, 0.1723402,
		0.63368737, -0.53888998, 0.55501163,
		0.54784353, -0.193924, -0.81379417,
	}
	RMat := mat.NewDense(3, 3, RMatDat)
	QTarg := Quat{0.219938314333263, -0.851301907985121, -0.426828005273729, -0.211494806705}
	QComp = QuatFromRotMat(RMat)
	if !(QComp.ApproxEquals(QTarg, epsilon) ||
		QComp.ApproxEquals(QTarg.MulScal(-1), epsilon)) {
		t.Errorf("Quat from rotation matrix %v is expected to be %v, not %v",
			matFmt(RMat), QTarg, QComp)
	}

	// Test the second branch path of QuatFromRotMat
	RMatDat = []float64{
		-0.78381123, 0.61936038, 0.04508521,
		0.58609292, 0.76179912, -0.27596594,
		-0.20526825, -0.18988108, -0.96010943,
	}
	RMat = mat.NewDense(3, 3, RMatDat)
	QTarg = Quat{0.0668551704949506, 0.321908018212949, 0.936178371957253, -0.124401245443004}
	QComp = QuatFromRotMat(RMat)
	if !(QComp.ApproxEquals(QTarg, epsilon) ||
		QComp.ApproxEquals(QTarg.MulScal(-1), epsilon)) {
		t.Errorf("Quat from rotation matrix %v is expected to be %v, not %v",
			matFmt(RMat), QTarg, QComp)
	}

	// Test the third branch path of QuatFromRotMat
	RMatDat = []float64{
		-0.83800261, -0.16077859, 0.52144211,
		0.52903087, -0.00522788, 0.84858648,
		-0.1337085, 0.98697665, 0.08943782,
	}
	RMat = mat.NewDense(3, 3, RMatDat)
	QTarg = Quat{0.24809641384963, 0.13945201421527, 0.660177420921586, 0.695102206924695}
	QComp = QuatFromRotMat(RMat)
	if !(QComp.ApproxEquals(QTarg, epsilon) ||
		QComp.ApproxEquals(QTarg.MulScal(-1), epsilon)) {
		t.Errorf("Quat from rotation matrix %v is expected to be %v, not %v",
			matFmt(RMat), QTarg, QComp)
	}

	// Test the fourth branch path of QuatFromRotMat
	RMatDat = []float64{
		0.45957249, -0.01894392, 0.88793821,
		0.69007053, 0.63699394, -0.34357151,
		-0.55910267, 0.770636, 0.30581753,
	}
	RMat = mat.NewDense(3, 3, RMatDat)
	QTarg = Quat{0.774981282496085, 0.359430458639042, 0.466798652234362, 0.228719862398165}
	QComp = QuatFromRotMat(RMat)
	if !(QComp.ApproxEquals(QTarg, epsilon) ||
		QComp.ApproxEquals(QTarg.MulScal(-1), epsilon)) {
		t.Errorf("Quat from rotation matrix %v is expected to be %v, not %v",
			matFmt(RMat), QTarg, QComp)
	}
}

func matFmt(X mat.Matrix) fmt.Formatter {
	return mat.Formatted(X, mat.Prefix(""), mat.Squeeze())
}

func eye(n int) mat.Matrix {
	d := make([]float64, n*n)
	for i := 0; i < n*n; i += n + 1 {
		d[i] = 1
	}
	return mat.NewDense(n, n, d)
}
