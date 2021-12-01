package quaternion

import "math"

var Tol float64 = 1e-7

type Vec3 struct {
	X float64
	Y float64
	Z float64
}

func (v1 Vec3) Cross(v2 Vec3) Vec3 {
	var result = Vec3{
		v1.Y*v2.Z - v1.Z*v2.Y,
		v1.Z*v2.X - v1.X*v2.Z,
		v1.X*v2.Y - v1.Y*v2.X,
	}
	return result
}

func (v1 Vec3) SumVec(v2 Vec3) Vec3 {
	var result = Vec3{
		v1.X + v2.X,
		v1.Y + v2.Y,
		v1.Z + v2.Z,
	}
	return result
}

func (v Vec3) SumScal(s float64) Vec3 {
	var result = Vec3{
		v.X + s, v.Y + s, v.Z + s,
	}
	return result
}

func (v Vec3) MulScal(s float64) Vec3 {
	var result = Vec3{
		v.X * s, v.Y * s, v.Z * s,
	}
	return result
}

func (v1 Vec3) ApproxEquals(v2 Vec3) bool {
	if math.Abs(v1.X-v2.X) < Tol &&
		math.Abs(v1.Y-v2.Y) < Tol &&
		math.Abs(v1.Z-v2.Z) < Tol {
		return true
	}
	return false
}

func (v Vec3) Norm() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v *Vec3) Normalize() {
	var vNormed = v.MulScal(v.Norm())
	v.X = vNormed.X
	v.Y = vNormed.Y
	v.Z = vNormed.Z
}
