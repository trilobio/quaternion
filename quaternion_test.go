package quaternion

import (
	"testing"
)

func TestRotVec(t *testing.T) {
	var q = Quat{0, 1, 0, 0}
	var v = Vec3{1, 2, 3}
	var vTarg = Vec3{1, -2, -3}

	var rotV = q.RotVec(v)
	if !(rotV.ApproxEquals(vTarg)) {
		t.Errorf("Rotated Vector = %v; want %v", rotV, vTarg)
	}

	q = Quat{0.5, 0.5, 0.5, 0.5}
	vTarg = Vec3{3, 1, 2}
	rotV = q.RotVec(v)
	if !(rotV.ApproxEquals(vTarg)) {
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
	if !(mulRes.ApproxEquals(qTarg)) {
		t.Errorf("Multiplied Quat = %v; want %v", mulRes, qTarg)
	}

}
