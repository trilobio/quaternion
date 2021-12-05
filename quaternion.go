package quaternion

import (
	"math"

	"gonum.org/v1/gonum/mat"
)

type Quat struct {
	W float64
	X float64
	Y float64
	Z float64
}

func (q *Quat) Identity() {
	q.W = 1
	q.X = 0
	q.Y = 0
	q.Z = 0
}

func (q1 Quat) MulQuat(q2 Quat) Quat {
	var result = Quat{
		q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z,
		q1.W*q2.X + q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y,
		q1.W*q2.Y - q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X,
		q1.W*q2.Z + q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W,
	}
	return result
}

func (q Quat) MulScal(s float64) Quat {
	var result = Quat{
		q.W * s, q.X * s, q.Y * s, q.Z * s,
	}
	return result
}

func (q Quat) RotVec(v Vec3) Vec3 {
	var r = Vec3{q.X, q.Y, q.Z}
	var m = q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z
	return v.SumVec(r.Cross(v.MulScal(q.W).SumVec(r.Cross(v))).MulScal(2 / m))
}

// ExtractVec extracts the complex component of the quaternion into a Vec3.
func (q Quat) ExtractVec() Vec3 {
	return Vec3{q.X, q.Y, q.Z}
}

func (q1 Quat) ApproxEquals(q2 Quat, epsilon float64) bool {
	if math.Abs(q1.W-q2.W) < epsilon &&
		math.Abs(q1.X-q2.X) < epsilon &&
		math.Abs(q1.Y-q2.Y) < epsilon &&
		math.Abs(q1.Z-q2.Z) < epsilon {
		return true
	}
	return false
}

func (q Quat) Conjugate() Quat {
	return Quat{q.W, -q.X, -q.Y, -q.Z}
}

func (q Quat) Norm() float64 {
	return math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
}

func (q *Quat) Normalize() {
	*q = q.MulScal(1 / q.Norm())
}

func (q *Quat) ToRotMat() mat.Matrix {
	rData := []float64{
		1 - 2*(q.Y*q.Y+q.Z*q.Z), 2 * (q.X*q.Y - q.Z*q.W), 2 * (q.X*q.Z + q.Y*q.W),
		2 * (q.X*q.Y + q.Z*q.W), 1 - 2*(q.X*q.X+q.Z*q.Z), 2 * (q.Y*q.Z - q.X*q.W),
		2 * (q.X*q.Z - q.Y*q.W), 2 * (q.Y*q.Z + q.X*q.W), 1 - 2*(q.X*q.X+q.Y*q.Y),
	}
	return mat.NewDense(3, 3, rData)
}

func QuatFromRotMat(m mat.Matrix) Quat {
	var Q Quat
	var diag = []float64{m.At(0, 0), m.At(1, 1), m.At(2, 2), mat.Trace(m)}
	idx := argMax(diag)

	if idx == 0 {
		Q.W = m.At(2, 1) - m.At(1, 2)
		Q.X = 1 + m.At(0, 0) - m.At(1, 1) - m.At(2, 2)
		Q.Y = m.At(0, 1) + m.At(1, 0)
		Q.Z = m.At(0, 2) + m.At(2, 0)
	} else if idx == 1 {
		Q.W = m.At(0, 2) - m.At(2, 0)
		Q.X = m.At(1, 0) + m.At(0, 1)
		Q.Y = 1 - m.At(0, 0) + m.At(1, 1) - m.At(2, 2)
		Q.Z = m.At(1, 2) + m.At(2, 1)
	} else if idx == 2 {
		Q.W = m.At(1, 0) - m.At(0, 1)
		Q.X = m.At(2, 0) + m.At(0, 2)
		Q.Y = m.At(2, 1) + m.At(1, 2)
		Q.Z = 1 - m.At(0, 0) - m.At(1, 1) + m.At(2, 2)
	} else if idx == 3 {
		Q.W = 1 + m.At(0, 0) + m.At(1, 1) + m.At(2, 2)
		Q.X = m.At(2, 1) - m.At(1, 2)
		Q.Y = m.At(0, 2) - m.At(2, 0)
		Q.Z = m.At(1, 0) - m.At(0, 1)
	}
	Q.Normalize()
	return Q
}

func argMax(arr []float64) int {
	if len(arr) == 0 {
		return -1
	}

	var amax int = 0
	var maxVal float64 = arr[0]
	for i := 1; i < len(arr); i++ {
		if arr[i] > maxVal {
			amax = i
			maxVal = arr[i]
		}
	}
	return amax
}
