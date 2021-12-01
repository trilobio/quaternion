package quaternion

import "math"

type Quat struct {
	W float64
	X float64
	Y float64
	Z float64
}

func (q *Quat) Unit() {
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
	var s = q.W
	var m = q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z
	return v.SumVec(r.Cross(v.MulScal(s).SumVec(r.Cross(v))).MulScal(2 / m))
}

func (q1 Quat) ApproxEquals(q2 Quat) bool {
	if math.Abs(q1.W-q2.W) < Tol &&
		math.Abs(q1.X-q2.X) < Tol &&
		math.Abs(q1.Y-q2.Y) < Tol &&
		math.Abs(q1.Z-q2.Z) < Tol {
		return true
	}
	return false
}
