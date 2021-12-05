package quaternion

import (
	"math"

	"gonum.org/v1/gonum/mat"
)

type Vec3 struct {
	X float64
	Y float64
	Z float64
}

// Dims, At and T minimally satisfy the mat.Matrix interface.
func (v Vec3) Dims() (r, c int) { return 3, 1 }
func (v Vec3) At(i, j int) float64 {
	if j != 0 {
		panic(mat.ErrColAccess)
	}

	if i == 0 {
		return v.X
	} else if i == 1 {
		return v.Y
	} else if i == 2 {
		return v.Z
	}
	panic(mat.ErrRowAccess)
}

func (v Vec3) AtVec(i int) float64 {
	if uint(i) >= uint(3) {
		panic(mat.ErrRowAccess)
	}
	return v.At(i, 0)
}

func (v Vec3) T() mat.Matrix {
	return mat.Transpose{Matrix: v}
}

func (v Vec3) Len() int { return 3 }

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

func (v1 Vec3) ApproxEquals(v2 Vec3, epsilon float64) bool {
	if math.Abs(v1.X-v2.X) < epsilon &&
		math.Abs(v1.Y-v2.Y) < epsilon &&
		math.Abs(v1.Z-v2.Z) < epsilon {
		return true
	}
	return false
}

func (v Vec3) Norm() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v Vec3) PureQuat() Quat {
	return Quat{0, v.X, v.Y, v.Z}
}

func (v *Vec3) Normalize() {
	var vNormed = v.MulScal(1 / v.Norm())
	v.X = vNormed.X
	v.Y = vNormed.Y
	v.Z = vNormed.Z
}
