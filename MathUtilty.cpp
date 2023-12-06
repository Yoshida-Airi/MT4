#include "MathUtilty.h"


bool IsCollision(const Sphere& s1, const Sphere& s2) {
	// 2つの弾の中心点間の距離を求める
	float distance = Length(Subtract(s1.center, s2.center));
	// 半径の合計よりも短ければ衝突
	if (distance <= s1.radius + s2.radius) {
		return true;
	}

	return false;
}

bool IsCollision(const Sphere& sphere, const Plane& plane) {
	float distance = fabs(Dot(plane.normal, sphere.center) - plane.distance);
	if (distance <= sphere.radius) {
		return true;
	}

	return false;
}

bool IsCollision(const Segment& segment, const Plane& plane) {

	// まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(plane.normal, segment.diff);

	// 垂直=平行であるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}

	// tを求める
	float t = (plane.distance - Dot(segment.origin, plane.normal)) / dot;

	// tの値と線の種類によって衝突しているかを判断する
	if (t <= 1.0f && t >= 0.0f) {
		return true;
	}
	return false;
}

bool IsCollision(const Triangle& triangle, const Segment& segment) {
	Vector3 v1 = Subtract(triangle.vertices[1], triangle.vertices[0]);
	Vector3 v2 = Subtract(triangle.vertices[2], triangle.vertices[1]);
	Vector3 cross = Cross(v1, v2);
	Vector3 normal = Normalize(cross);

	// 法線と線の内積を求める
	float dot = Dot(normal, segment.diff);

	// 垂直=平行であるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}
	float d = Dot(triangle.vertices[0], normal);
	float t = (d - Dot(segment.origin, normal) / dot);

	// tの値と線の種類によって衝突しているかを判断する
	if (t <= 1.0f && t >= 0.0f) {
		Vector3 p = {
		    segment.origin.x + t * segment.diff.x, segment.origin.y + t * segment.diff.y,
		    segment.origin.z + t * segment.diff.z};
		Vector3 v01 = Subtract(triangle.vertices[0], triangle.vertices[1]);
		Vector3 v12 = Subtract(triangle.vertices[1], triangle.vertices[2]);
		Vector3 v20 = Subtract(triangle.vertices[2], triangle.vertices[0]);
		Vector3 v0p = Subtract(triangle.vertices[0], p);
		Vector3 v1p = Subtract(triangle.vertices[1], p);
		Vector3 v2p = Subtract(triangle.vertices[2], p);
		// 各辺を結んだベクトルと、頂点と衝突点pを結んだベクトルのクロス積を取る
		Vector3 cross01 = Cross(v01, v0p);
		Vector3 cross12 = Cross(v12, v1p);
		Vector3 cross20 = Cross(v20, v2p);
		// すべての小三角形のクロス積と法線が同じ方向を向いていたら衝突
		if (Dot(cross01, normal) >= 0.0f && Dot(cross12, normal) >= 0.0f &&
		    Dot(cross20, normal) >= 0.0f) {
			return true;
		}
	}
	return false;
}

bool IsCollision(const AABB& aabb1, const AABB& aabb2) {
	if ((aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x) &&
	    (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y) &&
	    (aabb1.min.z <= aabb2.max.z && aabb1.max.z >= aabb2.min.z)) {
		return true;
	}
	return false;
}

bool IsCollision(const AABB& aabb, const Sphere& sphere) {
	// 最近接点を求める
	Vector3 clossestPoint{
	    std::clamp(sphere.center.x, aabb.min.x, aabb.max.x),
	    std::clamp(sphere.center.y, aabb.min.y, aabb.max.y),
	    std::clamp(sphere.center.z, aabb.min.z, aabb.max.z)};
	// 最近接点と球の中心との距離を求める
	float distance = Length(Subtract(clossestPoint, sphere.center));

	// 距離が半径よりも小さければ衝突
	if (distance <= sphere.radius) {
		return true;
	}
	return false;
}

// 加算
Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 result;
	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;

	return result;
}

// 減算
Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 result;
	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;
	return result;
}

// スカラー倍
Vector3 Multiply(float scalar, const Vector3& v) {
	Vector3 result;
	result.x = scalar * v.x;
	result.y = scalar * v.y;
	result.z = scalar * v.z;
	return result;
}

// 内積
float Dot(const Vector3& v1, const Vector3& v2) {
	float result;
	result = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	return result;
}

// 長さ(ノルム)
float Length(const Vector3& v) {
	float result;
	result = powf(v.x, 2.0) + powf(v.y, 2.0) + powf(v.z, 2.0);

	return sqrtf(result);
};

// 正規化
Vector3 Normalize(const Vector3& v) {
	Vector3 result;
	float x;
	x = Length(v);
	result.x = v.x / x;
	result.y = v.y / x;
	result.z = v.z / x;
	return result;
}

// 1.行列の加法
Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 resultAdd;
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			resultAdd.m[row][column] = m1.m[row][column] + m2.m[row][column];
		}
	}
	return resultAdd;
}

// 2.行列の減法
Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 resultSubtract;
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			resultSubtract.m[row][column] = m1.m[row][column] - m2.m[row][column];
		}
	}
	return resultSubtract;
}

// 行列の掛け算の関数
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 resultMultiply;
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			resultMultiply.m[row][column] =
			    m1.m[row][0] * m2.m[0][column] + m1.m[row][1] * m2.m[1][column] +
			    m1.m[row][2] * m2.m[2][column] + m1.m[row][3] * m2.m[3][column];
		}
	}
	return resultMultiply;
}

// 4.逆行列
Matrix4x4 Inverse(const Matrix4x4& m) {
	Matrix4x4 result;
	float formula = m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3] +
	                m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1] +
	                m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2] -

	                m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1] -
	                m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3] -
	                m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2] -

	                m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3] -
	                m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1] -
	                m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2] +

	                m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1] +
	                m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3] +
	                m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2] +

	                m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3] +
	                m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1] +
	                m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2] -

	                m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1] -
	                m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3] -
	                m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2] -

	                m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0] -
	                m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0] -
	                m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0] +

	                m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0] +
	                m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0] +
	                m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0];

	assert(formula != 0.0f);
	float formulaRec = 1.0f / formula;

	result.m[0][0] = (m.m[1][1] * m.m[2][2] * m.m[3][3] + m.m[1][2] * m.m[2][3] * m.m[3][1] +
	                  m.m[1][3] * m.m[2][1] * m.m[3][2] - m.m[1][3] * m.m[2][2] * m.m[3][1] -
	                  m.m[1][2] * m.m[2][1] * m.m[3][3] - m.m[1][1] * m.m[2][3] * m.m[3][2]) *
	                 formulaRec;

	result.m[0][1] = (-m.m[0][1] * m.m[2][2] * m.m[3][3] - m.m[0][2] * m.m[2][3] * m.m[3][1] -
	                  m.m[0][3] * m.m[2][1] * m.m[3][2] + m.m[0][3] * m.m[2][2] * m.m[3][1] +
	                  m.m[0][2] * m.m[2][1] * m.m[3][3] + m.m[0][1] * m.m[2][3] * m.m[3][2]) *
	                 formulaRec;

	result.m[0][2] = (m.m[0][1] * m.m[1][2] * m.m[3][3] + m.m[0][2] * m.m[1][3] * m.m[3][1] +
	                  m.m[0][3] * m.m[1][1] * m.m[3][2] - m.m[0][3] * m.m[1][2] * m.m[3][1] -
	                  m.m[0][2] * m.m[1][1] * m.m[3][3] - m.m[0][1] * m.m[1][3] * m.m[3][2]) *
	                 formulaRec;

	result.m[0][3] = (-m.m[0][1] * m.m[1][2] * m.m[2][3] - m.m[0][2] * m.m[1][3] * m.m[2][1] -
	                  m.m[0][3] * m.m[1][1] * m.m[2][2] + m.m[0][3] * m.m[1][2] * m.m[2][1] +
	                  m.m[0][2] * m.m[1][1] * m.m[2][3] + m.m[0][1] * m.m[1][3] * m.m[2][2]) *
	                 formulaRec;

	result.m[1][0] = (-m.m[1][0] * m.m[2][2] * m.m[3][3] - m.m[1][2] * m.m[2][3] * m.m[3][0] -
	                  m.m[1][3] * m.m[2][0] * m.m[3][2] + m.m[1][3] * m.m[2][2] * m.m[3][0] +
	                  m.m[1][2] * m.m[2][0] * m.m[3][3] + m.m[1][0] * m.m[2][3] * m.m[3][2]) *
	                 formulaRec;

	result.m[1][1] = (m.m[0][0] * m.m[2][2] * m.m[3][3] + m.m[0][2] * m.m[2][3] * m.m[3][0] +
	                  m.m[0][3] * m.m[2][0] * m.m[3][2] - m.m[0][3] * m.m[2][2] * m.m[3][0] -
	                  m.m[0][2] * m.m[2][0] * m.m[3][3] - m.m[0][0] * m.m[2][3] * m.m[3][2]) *
	                 formulaRec;

	result.m[1][2] = (-m.m[0][0] * m.m[1][2] * m.m[3][3] - m.m[0][2] * m.m[1][3] * m.m[3][0] -
	                  m.m[0][3] * m.m[1][0] * m.m[3][2] + m.m[0][3] * m.m[1][2] * m.m[3][0] +
	                  m.m[0][2] * m.m[1][0] * m.m[3][3] + m.m[0][0] * m.m[1][3] * m.m[3][2]) *
	                 formulaRec;

	result.m[1][3] = (m.m[0][0] * m.m[1][2] * m.m[2][3] + m.m[0][2] * m.m[1][3] * m.m[2][0] +
	                  m.m[0][3] * m.m[1][0] * m.m[2][2] - m.m[0][3] * m.m[1][2] * m.m[2][0] -
	                  m.m[0][2] * m.m[1][0] * m.m[2][3] - m.m[0][0] * m.m[1][3] * m.m[2][2]) *
	                 formulaRec;

	result.m[2][0] = result.m[1][1] =
	    (m.m[1][0] * m.m[2][1] * m.m[3][3] + m.m[1][1] * m.m[2][3] * m.m[3][0] +
	     m.m[1][3] * m.m[2][0] * m.m[3][1] - m.m[1][3] * m.m[2][1] * m.m[3][0] -
	     m.m[1][1] * m.m[2][0] * m.m[3][3] - m.m[1][0] * m.m[2][3] * m.m[3][1]) *
	    formulaRec;

	result.m[2][1] = result.m[1][1] =
	    (-m.m[0][0] * m.m[2][1] * m.m[3][3] - m.m[0][1] * m.m[2][3] * m.m[3][0] -
	     m.m[0][3] * m.m[2][0] * m.m[3][1] + m.m[0][3] * m.m[2][1] * m.m[3][0] +
	     m.m[0][1] * m.m[2][0] * m.m[3][3] + m.m[0][0] * m.m[2][3] * m.m[3][1]) *
	    formulaRec;

	result.m[2][2] = result.m[1][1] =
	    (m.m[0][0] * m.m[1][1] * m.m[3][3] + m.m[0][1] * m.m[1][3] * m.m[3][0] +
	     m.m[0][3] * m.m[1][0] * m.m[3][1] - m.m[0][3] * m.m[1][1] * m.m[3][0] -
	     m.m[0][1] * m.m[1][0] * m.m[3][3] - m.m[0][0] * m.m[1][3] * m.m[3][1]) *
	    formulaRec;

	result.m[2][3] = (m.m[0][0] * m.m[1][1] * m.m[2][3] + m.m[0][1] * m.m[1][3] * m.m[2][0] +
	                  m.m[0][3] * m.m[1][0] * m.m[2][1] - m.m[0][3] * m.m[1][1] * m.m[2][0] -
	                  m.m[0][1] * m.m[1][0] * m.m[2][3] - m.m[0][0] * m.m[1][3] * m.m[2][1]) *
	                 formulaRec;

	result.m[3][0] = (-m.m[1][0] * m.m[2][1] * m.m[3][2] - m.m[1][1] * m.m[2][2] * m.m[3][0] -
	                  m.m[1][2] * m.m[2][0] * m.m[3][1] + m.m[1][2] * m.m[2][1] * m.m[3][0] +
	                  m.m[1][1] * m.m[2][0] * m.m[3][2] + m.m[1][0] * m.m[2][2] * m.m[3][1]) *
	                 formulaRec;

	result.m[3][1] = (m.m[0][0] * m.m[2][1] * m.m[3][2] + m.m[0][1] * m.m[2][2] * m.m[3][0] +
	                  m.m[0][2] * m.m[2][0] * m.m[3][1] - m.m[0][2] * m.m[2][1] * m.m[3][0] -
	                  m.m[0][1] * m.m[2][0] * m.m[3][2] - m.m[0][0] * m.m[2][2] * m.m[3][1]) *
	                 formulaRec;

	result.m[3][2] = (-m.m[0][0] * m.m[1][1] * m.m[3][2] - m.m[0][1] * m.m[1][2] * m.m[3][0] -
	                  m.m[0][2] * m.m[1][0] * m.m[3][1] + m.m[0][2] * m.m[1][1] * m.m[3][0] +
	                  m.m[0][1] * m.m[1][0] * m.m[3][2] + m.m[0][0] * m.m[1][2] * m.m[3][1]) *
	                 formulaRec;

	result.m[3][3] = (m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[0][1] * m.m[1][2] * m.m[2][0] +
	                  m.m[0][2] * m.m[1][0] * m.m[2][1] - m.m[0][2] * m.m[1][1] * m.m[2][0] -
	                  m.m[0][1] * m.m[1][0] * m.m[2][2] - m.m[0][0] * m.m[1][2] * m.m[2][1]) *
	                 formulaRec;

	return result;
}

////5.転置行列
Matrix4x4 Transpose(const Matrix4x4& m) {
	Matrix4x4 result;
	result.m[0][0] = m.m[0][0];
	result.m[0][1] = m.m[1][0];
	result.m[0][2] = m.m[2][0];
	result.m[0][3] = m.m[3][0];

	result.m[1][0] = m.m[0][1];
	result.m[1][1] = m.m[1][1];
	result.m[1][2] = m.m[2][1];
	result.m[1][3] = m.m[3][1];

	result.m[2][0] = m.m[0][2];
	result.m[2][1] = m.m[1][2];
	result.m[2][2] = m.m[2][2];
	result.m[2][3] = m.m[3][2];

	result.m[3][0] = m.m[0][3];
	result.m[3][1] = m.m[1][3];
	result.m[3][2] = m.m[2][3];
	result.m[3][3] = m.m[3][3];

	return result;
}
////6.単位行列の作成
Matrix4x4 MakeIdentity4x4() {
	Matrix4x4 result;
	result.m[0][0] = 1.0f;
	result.m[0][1] = 0.0f;
	result.m[0][2] = 0.0f;
	result.m[0][3] = 0.0f;

	result.m[1][0] = 0.0f;
	result.m[1][1] = 1.0f;
	result.m[1][2] = 0.0f;
	result.m[1][3] = 0.0f;

	result.m[2][0] = 0.0f;
	result.m[2][1] = 0.0f;
	result.m[2][2] = 1.0f;
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;
}

// 平行移動(translate)
// 平行移動行列の関数
Matrix4x4 MakeTranselateMatrix(const Vector3& transelate) {
	Matrix4x4 result;
	result.m[0][0] = 1.0f;
	result.m[0][1] = 0.0f;
	result.m[0][2] = 0.0f;
	result.m[0][3] = 0.0f;

	result.m[1][0] = 0.0f;
	result.m[1][1] = 1.0f;
	result.m[1][2] = 0.0f;
	result.m[1][3] = 0.0f;

	result.m[2][0] = 0.0f;
	result.m[2][1] = 0.0f;
	result.m[2][2] = 1.0f;
	result.m[2][3] = 0.0f;

	result.m[3][0] = transelate.x;
	result.m[3][1] = transelate.y;
	result.m[3][2] = transelate.z;
	result.m[3][3] = 1.0f;

	return result;
}

// 拡大縮小(scale)
// 拡大縮小行列の関数
Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 result;
	result.m[0][0] = scale.x;
	result.m[0][1] = 0.0f;
	result.m[0][2] = 0.0f;
	result.m[0][3] = 0.0f;

	result.m[1][0] = 0.0f;
	result.m[1][1] = scale.y;
	result.m[1][2] = 0.0f;
	result.m[1][3] = 0.0f;

	result.m[2][0] = 0.0f;
	result.m[2][1] = 0.0f;
	result.m[2][2] = scale.z;
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;
}
// 3.座標変換
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 result;
	result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] +
	           1.0f * matrix.m[3][0];
	result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] +
	           1.0f * matrix.m[3][1];
	result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] +
	           1.0f * matrix.m[3][2];
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] +
	          1.0f * matrix.m[3][3];

	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;
}

// 回転(rotate)

// x軸回転行列の関数
Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 resultMakeRotatedMatrix;
	resultMakeRotatedMatrix.m[0][0] = 1;
	resultMakeRotatedMatrix.m[0][1] = 0;
	resultMakeRotatedMatrix.m[0][2] = 0;
	resultMakeRotatedMatrix.m[0][3] = 0;

	resultMakeRotatedMatrix.m[1][0] = 0;
	resultMakeRotatedMatrix.m[1][1] = std::cos(radian);
	resultMakeRotatedMatrix.m[1][2] = std::sin(radian);
	resultMakeRotatedMatrix.m[1][3] = 0;

	resultMakeRotatedMatrix.m[2][0] = 0;
	resultMakeRotatedMatrix.m[2][1] = -std::sin(radian);
	resultMakeRotatedMatrix.m[2][2] = std::cos(radian);
	resultMakeRotatedMatrix.m[2][3] = 0;

	resultMakeRotatedMatrix.m[3][0] = 0;
	resultMakeRotatedMatrix.m[3][1] = 0;
	resultMakeRotatedMatrix.m[3][2] = 0;
	resultMakeRotatedMatrix.m[3][3] = 1;

	return resultMakeRotatedMatrix;
}

// y軸回転行列の関数
Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 resultMakeRotatedMatrix;
	resultMakeRotatedMatrix.m[0][0] = std::cos(radian);
	resultMakeRotatedMatrix.m[0][1] = 0;
	resultMakeRotatedMatrix.m[0][2] = -std::sin(radian);
	resultMakeRotatedMatrix.m[0][3] = 0;

	resultMakeRotatedMatrix.m[1][0] = 0;
	resultMakeRotatedMatrix.m[1][1] = 1;
	resultMakeRotatedMatrix.m[1][2] = 0;
	resultMakeRotatedMatrix.m[1][3] = 0;

	resultMakeRotatedMatrix.m[2][0] = std::sin(radian);
	resultMakeRotatedMatrix.m[2][1] = 0;
	resultMakeRotatedMatrix.m[2][2] = std::cos(radian);
	resultMakeRotatedMatrix.m[2][3] = 0;

	resultMakeRotatedMatrix.m[3][0] = 0;
	resultMakeRotatedMatrix.m[3][1] = 0;
	resultMakeRotatedMatrix.m[3][2] = 0;
	resultMakeRotatedMatrix.m[3][3] = 1;

	return resultMakeRotatedMatrix;
}

// z軸回転行列の関数
Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 resultMakeRotatedMatrix;
	resultMakeRotatedMatrix.m[0][0] = std::cos(radian);
	resultMakeRotatedMatrix.m[0][1] = std::sin(radian);
	resultMakeRotatedMatrix.m[0][2] = 0;
	resultMakeRotatedMatrix.m[0][3] = 0;

	resultMakeRotatedMatrix.m[1][0] = -std::sin(radian);
	resultMakeRotatedMatrix.m[1][1] = std::cos(radian);
	resultMakeRotatedMatrix.m[1][2] = 0;
	resultMakeRotatedMatrix.m[1][3] = 0;

	resultMakeRotatedMatrix.m[2][0] = 0;
	resultMakeRotatedMatrix.m[2][1] = 0;
	resultMakeRotatedMatrix.m[2][2] = 1;
	resultMakeRotatedMatrix.m[2][3] = 0;

	resultMakeRotatedMatrix.m[3][0] = 0;
	resultMakeRotatedMatrix.m[3][1] = 0;
	resultMakeRotatedMatrix.m[3][2] = 0;
	resultMakeRotatedMatrix.m[3][3] = 1;

	return resultMakeRotatedMatrix;
}

// 3次元アフィン変換行列の関数
Matrix4x4 MakeAffinMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 resultMakeAffinMatrix;
	Matrix4x4 resultMakeScaleMatrix = MakeScaleMatrix(scale);
	Matrix4x4 resultMakeTranselateMatrix = MakeTranselateMatrix(translate);
	Matrix4x4 resultMakeRotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 resultMakeRotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 resultMakeRotateZMatrix = MakeRotateZMatrix(rotate.z);

	Matrix4x4 rotateXYZMatrix = Multiply(
	    resultMakeRotateXMatrix, Multiply(resultMakeRotateYMatrix, resultMakeRotateZMatrix));

	resultMakeAffinMatrix =
	    Multiply(resultMakeScaleMatrix, Multiply(rotateXYZMatrix, resultMakeTranselateMatrix));

	return resultMakeAffinMatrix;
}

Vector3 TransformNormal(const Vector3& v, const Matrix4x4& m) {
	Vector3 result = {
	    v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0],
	    v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1],
	    v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2],
	};

	return result;
}

Vector3 SumVector3(Vector3& num1, Vector3& num2) {
	Vector3 result{};
	result.x = num1.x += num2.x;
	result.y = num1.y += num2.y;
	result.z = num1.z += num2.z;
	return result;
}

// 1.透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float forY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result;

	result.m[0][0] = 1 / aspectRatio * (1 / std::tan(forY / 2));
	result.m[0][1] = 0;
	result.m[0][2] = 0;
	result.m[0][3] = 0;

	result.m[1][0] = 0;
	result.m[1][1] = 1 / std::tan(forY / 2);
	result.m[1][2] = 0;
	result.m[1][3] = 0;

	result.m[2][0] = 0;
	result.m[2][1] = 0;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1;

	result.m[3][0] = 0;
	result.m[3][1] = 0;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	result.m[3][3] = 0;

	return result;
}

// 2.正射影行列
Matrix4x4 MakeOrthographicmatrix(
    float left, float top, float right, float bottom, float nearClip, float farClip) {
	Matrix4x4 result;

	result.m[0][0] = 2 / (right - left);
	result.m[0][1] = 0;
	result.m[0][2] = 0;
	result.m[0][3] = 0;

	result.m[1][0] = 0;
	result.m[1][1] = 2 / (top - bottom);
	result.m[1][2] = 0;
	result.m[1][3] = 0;

	result.m[2][0] = 0;
	result.m[2][1] = 0;
	result.m[2][2] = 1 / (farClip - nearClip);
	result.m[2][3] = 0;

	result.m[3][0] = (left + right) / (left - right);
	result.m[3][1] = (top + bottom) / (bottom - top);
	result.m[3][2] = nearClip / (nearClip - farClip);
	result.m[3][3] = 1;

	return result;
}

// 3.ビューポート変換行列
Matrix4x4 MakeViewportMatrix(
    float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result;

	result.m[0][0] = width / 2;
	result.m[0][1] = 0;
	result.m[0][2] = 0;
	result.m[0][3] = 0;

	result.m[1][0] = 0;
	result.m[1][1] = -height / 2;
	result.m[1][2] = 0;
	result.m[1][3] = 0;

	result.m[2][0] = 0;
	result.m[2][1] = 0;
	result.m[2][2] = maxDepth - minDepth;
	result.m[2][3] = 0;

	result.m[3][0] = left + (width / 2);
	result.m[3][1] = top + (height / 2);
	result.m[3][2] = minDepth;
	result.m[3][3] = 1;

	return result;
}

// クロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 result;
	result = {v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x};
	return result;
}

void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);
}

void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, label);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(
			    x + column * kColumnWidth, y + 20 + row * kRowHeight, "%6.03f",
			    matrix.m[row][column]);
		}
	}
}

void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label)
{
	Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", quaternion.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", quaternion.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", quaternion.w);
	Novice::ScreenPrintf(x + kColumnWidth * 5, y, ":%s", label);
}

// グリッド線
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;                                      // Gridの半分の幅
	const uint32_t kSubdivision = 10;                                       // 分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision); // 1つ分の長さ

	Vector3 worldVertices[2];
	Vector3 screenVertices[2];
	Vector3 ndcVertex;

	// 奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {

		// ワールド座標系上の始点と終点を求める
		worldVertices[0] = {xIndex * kGridEvery - kGridHalfWidth, 0.0f, kGridHalfWidth};
		worldVertices[1] = {xIndex * kGridEvery - kGridHalfWidth, 0.0f, -kGridHalfWidth};

		// スクリーン座標系まで変換をかける
		for (uint32_t i = 0; i < 2; ++i) {
			ndcVertex = Transform(worldVertices[i], viewProjectionMatrix);
			screenVertices[i] = Transform(ndcVertex, viewportMatrix);
		}

		// 表示
		if (xIndex * kGridEvery - kGridHalfWidth == 0.0f) {
			Novice::DrawLine(
			    int(screenVertices[0].x), int(screenVertices[0].y), int(screenVertices[1].x),
			    int(screenVertices[1].y), 0x000000FF);
		} else {
			Novice::DrawLine(
			    int(screenVertices[0].x), int(screenVertices[0].y), int(screenVertices[1].x),
			    int(screenVertices[1].y), 0xAAAAAAFF);
		}
	}

	// 左から右も同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		// ワールド座標系上の始点と終点を求める
		worldVertices[0] = {kGridHalfWidth, 0.0f, zIndex * kGridEvery - kGridHalfWidth};
		worldVertices[1] = {-kGridHalfWidth, 0.0f, zIndex * kGridEvery - kGridHalfWidth};

		// スクリーン座標系まで変換をかける
		for (uint32_t i = 0; i < 2; ++i) {
			ndcVertex = Transform(worldVertices[i], viewProjectionMatrix);
			screenVertices[i] = Transform(ndcVertex, viewportMatrix);
		}

		// 表示
		if (zIndex * kGridEvery - kGridHalfWidth == 0.0f) {
			Novice::DrawLine(
			    int(screenVertices[0].x), int(screenVertices[0].y), int(screenVertices[1].x),
			    int(screenVertices[1].y), 0x000000FF);
		} else {
			Novice::DrawLine(
			    int(screenVertices[0].x), int(screenVertices[0].y), int(screenVertices[1].x),
			    int(screenVertices[1].y), 0xAAAAAAFF);
		}
	}
}

// 球
void DrawSphere(
    const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix,
    uint32_t color) {
	const uint32_t kSubdivision = 16; // 分割数
	const float kLonEvery =
	    (2.0f * static_cast<float>(std::numbers::pi)) / kSubdivision; // 経度分割1つ分の角度
	const float kLatEvery =
	    static_cast<float>(std::numbers::pi) / kSubdivision; // 緯度分割1つ分の角度

	// 緯度の方向に分割
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat =
		    static_cast<float>(-std::numbers::pi) / 2.0f + kLatEvery * latIndex; // 現在の緯度
		// 経度の方向に分割
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery; // 現在の経度
			// world座標系でのa,b,cを求める
			Vector3 a, b, c;

			a = {
			    sphere.radius * std::cosf(lat) * std::cosf(lon), sphere.radius * std::sinf(lat),
			    sphere.radius * std::cosf(lat) * std::sinf(lon)};
			a = Add(a, sphere.center);
			b = {
			    sphere.radius * std::cosf(lat + kLatEvery) * std::cosf(lon),
			    sphere.radius * std::sinf(lat + kLatEvery),
			    sphere.radius * std::cosf(lat + kLatEvery) * std::sinf(lon)};
			b = Add(b, sphere.center);
			c = {
			    sphere.radius * std::cosf(lat) * std::cosf(lon + kLonEvery),
			    sphere.radius * std::sinf(lat),
			    sphere.radius * std::cosf(lat) * std::sinf(lon + kLonEvery)};
			c = Add(c, sphere.center);

			// a,b,cをScreen座標系まで変換
			Vector3 ndcA = Transform(a, viewProjectionMatrix);
			Vector3 ndcB = Transform(b, viewProjectionMatrix);
			Vector3 ndcC = Transform(c, viewProjectionMatrix);

			Vector3 screenA = Transform(ndcA, viewportMatrix);
			Vector3 screenB = Transform(ndcB, viewportMatrix);
			Vector3 screenC = Transform(ndcC, viewportMatrix);

			// ab,bcで線を引く
			Novice::DrawLine(
			    static_cast<int>(screenA.x), static_cast<int>(screenA.y),
			    static_cast<int>(screenB.x), static_cast<int>(screenB.y), color);

			Novice::DrawLine(
			    static_cast<int>(screenA.x), static_cast<int>(screenA.y),
			    static_cast<int>(screenC.x), static_cast<int>(screenC.y), color);
		}
	}
}


// 正射影ベクトル
Vector3 Project(const Vector3& v1, const Vector3& v2) {
	Vector3 result;

	Vector3 nor = Normalize(v2);
	float dot = Dot(v1, nor);

	result.x = dot * nor.x;
	result.y = dot * nor.y;
	result.z = dot * nor.z;

	return result;
}

// 最近接点
Vector3 ClosestPoint(const Vector3& point, const Segment& segment) {
	Vector3 result;
	Vector3 project = Project(Subtract(point, segment.origin), segment.diff);

	result.x = segment.origin.x + project.x;
	result.y = segment.origin.y + project.y;
	result.z = segment.origin.z + project.z;

	return result;
}

Vector3 Perpendicular(const Vector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return {-vector.y, vector.x, 0.0f};
	}

	return {0.0f, -vector.z, vector.y};
}

// 平面
void DrawPlane(
    const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	Vector3 center = Multiply(plane.distance, plane.normal);

	Vector3 perpendiclars[4];
	perpendiclars[0] = Normalize(Perpendicular(plane.normal));
	perpendiclars[1] = {-perpendiclars[0].x, -perpendiclars[0].y, -perpendiclars[0].z};
	perpendiclars[2] = Cross(plane.normal, perpendiclars[0]);
	perpendiclars[3] = {-perpendiclars[2].x, -perpendiclars[2].y, -perpendiclars[2].z};

	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = Multiply(2.0f, perpendiclars[index]);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawLine(
	    int(points[0].x), int(points[0].y), int(points[3].x), int(points[3].y), plane.color);
	Novice::DrawLine(
	    int(points[1].x), int(points[1].y), int(points[2].x), int(points[2].y), plane.color);
	Novice::DrawLine(
	    int(points[2].x), int(points[2].y), int(points[0].x), int(points[0].y), plane.color);
	Novice::DrawLine(
	    int(points[3].x), int(points[3].y), int(points[1].x), int(points[1].y), plane.color);
}

void DrawTriangle(
    const Triangle& triangle, const Matrix4x4& viewProjectionMatrix,
    const Matrix4x4& viewportMatrix, uint32_t color) {
	// 頂点座標
	Vector3 scVertices[3];
	for (int i = 0; i < 3; ++i) {
		Vector3 ndcVertex = Transform(triangle.vertices[i], viewProjectionMatrix);
		scVertices[i] = Transform(ndcVertex, viewportMatrix);
	}

	// 描画
	Novice::DrawTriangle(
	    int(scVertices[0].x), int(scVertices[0].y), int(scVertices[1].x), int(scVertices[1].y),
	    int(scVertices[2].x), int(scVertices[2].y), color, kFillModeWireFrame);
}

// AABB
void DrawAABB(
    const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix,
    uint32_t color) {
	Vector3 vertex[8]{};
	vertex[0] = {aabb.min.x, aabb.min.y, aabb.min.z};
	vertex[1] = {aabb.min.x, aabb.min.y, aabb.max.z};
	vertex[2] = {aabb.min.x, aabb.max.y, aabb.min.z};
	vertex[3] = {aabb.max.x, aabb.min.y, aabb.min.z};
	vertex[4] = {aabb.max.x, aabb.max.y, aabb.min.z};
	vertex[5] = {aabb.min.x, aabb.max.y, aabb.max.z};
	vertex[6] = {aabb.max.x, aabb.min.y, aabb.max.z};
	vertex[7] = {aabb.max.x, aabb.max.y, aabb.max.z};

	Vector3 screenVertex[8]{};
	for (int i = 0; i < 8; i++) {
		vertex[i] = Transform(vertex[i], viewProjectionMatrix);
		screenVertex[i] = Transform(vertex[i], viewportMatrix);
	}

	Novice::DrawLine(
	    int(screenVertex[0].x), int(screenVertex[0].y), int(screenVertex[1].x),
	    int(screenVertex[1].y), color);
	Novice::DrawLine(
	    int(screenVertex[0].x), int(screenVertex[0].y), int(screenVertex[2].x),
	    int(screenVertex[2].y), color);
	Novice::DrawLine(
	    int(screenVertex[0].x), int(screenVertex[0].y), int(screenVertex[3].x),
	    int(screenVertex[3].y), color);

	Novice::DrawLine(
	    int(screenVertex[1].x), int(screenVertex[1].y), int(screenVertex[5].x),
	    int(screenVertex[5].y), color);
	Novice::DrawLine(
	    int(screenVertex[1].x), int(screenVertex[1].y), int(screenVertex[6].x),
	    int(screenVertex[6].y), color);

	Novice::DrawLine(
	    int(screenVertex[2].x), int(screenVertex[2].y), int(screenVertex[4].x),
	    int(screenVertex[4].y), color);
	Novice::DrawLine(
	    int(screenVertex[2].x), int(screenVertex[2].y), int(screenVertex[5].x),
	    int(screenVertex[5].y), color);

	Novice::DrawLine(
	    int(screenVertex[3].x), int(screenVertex[3].y), int(screenVertex[4].x),
	    int(screenVertex[4].y), color);
	Novice::DrawLine(
	    int(screenVertex[3].x), int(screenVertex[3].y), int(screenVertex[6].x),
	    int(screenVertex[6].y), color);

	Novice::DrawLine(
	    int(screenVertex[4].x), int(screenVertex[4].y), int(screenVertex[7].x),
	    int(screenVertex[7].y), color);
	Novice::DrawLine(
	    int(screenVertex[5].x), int(screenVertex[5].y), int(screenVertex[7].x),
	    int(screenVertex[7].y), color);
	Novice::DrawLine(
	    int(screenVertex[6].x), int(screenVertex[6].y), int(screenVertex[7].x),
	    int(screenVertex[7].y), color);
}

Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float angle)
{
	Matrix4x4 result;

	double radian = angle;

	float sinTheta = static_cast<float>(std::sin(radian));
	float cosTheta = static_cast<float>(std::cos(radian));

	Vector3 nAxsis = axis;


	result.m[0][0] = nAxsis.x * nAxsis.x * (1 - cosTheta) + cosTheta;
	result.m[0][1] = nAxsis.x * nAxsis.y * (1 - cosTheta) + nAxsis.z * sinTheta;
	result.m[0][2] = nAxsis.x * nAxsis.z * (1 - cosTheta) - nAxsis.y * sinTheta;
	result.m[0][3] = 0.0f;

	result.m[1][0] = nAxsis.x * nAxsis.y * (1 - cosTheta) - nAxsis.z * sinTheta;
	result.m[1][1] = nAxsis.y * nAxsis.y * (1 - cosTheta) + cosTheta;
	result.m[1][2] = nAxsis.y * nAxsis.z * (1 - cosTheta) + nAxsis.x * sinTheta;
	result.m[1][3] = 0.0f;

	result.m[2][0] = nAxsis.x * nAxsis.z * (1 - cosTheta) + nAxsis.y * sinTheta;
	result.m[2][1] = nAxsis.y * nAxsis.z * (1 - cosTheta) - nAxsis.x * sinTheta;
	result.m[2][2] = nAxsis.z * nAxsis.z * (1 - cosTheta) + cosTheta;
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;

}

Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float sinTheta,float cosTheta) {
	Matrix4x4 result;

	Vector3 nAxsis = axis;

	result.m[0][0] = nAxsis.x * nAxsis.x * (1 - cosTheta) + cosTheta;
	result.m[0][1] = nAxsis.x * nAxsis.y * (1 - cosTheta) + nAxsis.z * sinTheta;
	result.m[0][2] = nAxsis.x * nAxsis.z * (1 - cosTheta) - nAxsis.y * sinTheta;
	result.m[0][3] = 0.0f;

	result.m[1][0] = nAxsis.x * nAxsis.y * (1 - cosTheta) - nAxsis.z * sinTheta;
	result.m[1][1] = nAxsis.y * nAxsis.y * (1 - cosTheta) + cosTheta;
	result.m[1][2] = nAxsis.y * nAxsis.z * (1 - cosTheta) + nAxsis.x * sinTheta;
	result.m[1][3] = 0.0f;

	result.m[2][0] = nAxsis.x * nAxsis.z * (1 - cosTheta) + nAxsis.y * sinTheta;
	result.m[2][1] = nAxsis.y * nAxsis.z * (1 - cosTheta) - nAxsis.x * sinTheta;
	result.m[2][2] = nAxsis.z * nAxsis.z * (1 - cosTheta) + cosTheta;
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;
}


Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs)
{
	Quaternion result;

	result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
	result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
	result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
	result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;

	return result;
}

Quaternion IdentityQuaternion()
{
	Quaternion result;
	result.x = 0.0f;
	result.y = 0.0f;
	result.z = 0.0f;
	result.w = 1.0f;
	return result;
}

Quaternion Conjugate(const Quaternion& quaternion) 
{ 
	Quaternion result;
	result.x = -quaternion.x;
	result.y = -quaternion.y;
	result.z = -quaternion.z;
	result.w = quaternion.w;
	return result;
}

float Norm(const Quaternion& quaternion)
{ 
	float result;
	result = sqrtf(
	    (quaternion.w * quaternion.w) + (quaternion.x * quaternion.x) +
	    (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z));

	return result;
}

Quaternion Normalize(const Quaternion& quaternion) {
	Quaternion result;
	float norm = Norm(quaternion);
	float invNorm = 1.0f / norm;

	result.x = quaternion.x * invNorm;
	result.y = quaternion.y * invNorm;
	result.z = quaternion.z * invNorm;
	result.w = quaternion.w * invNorm;

	return result;
}

Quaternion Inverse(const Quaternion& quaternion) {
	
	Quaternion result;

	Quaternion result2 = Conjugate(quaternion);

	result.x = result2.x / (Norm(quaternion) * Norm(quaternion));
	result.y = result2.y / (Norm(quaternion) * Norm(quaternion));
	result.z = result2.z / (Norm(quaternion) * Norm(quaternion));
	result.w = result2.w / (Norm(quaternion) * Norm(quaternion));

	return result;
}


// 線形補間
Vector3 Lerp(const Vector3& v1, const Vector3& v2, float t) {
	Vector3 P;
	P.x = v1.x + t * (v2.x - v1.x);
	P.y = v1.y + t * (v2.y - v1.y);
	P.z = v1.z + t * (v2.z - v1.z);
	return P;
}

// 球面線形補間
Vector3 Slerp(const Vector3& v1, const Vector3& v2, float t) {

	float angle = std::acos(Dot(v1, v2));

	float sinTheta = std::sin(angle);

	float PositonStart = std::sin(angle * (1 - t));
	float PositonEnd = std::sin(angle * t);

	Vector3 result;
	result.x = (PositonStart * v1.x + PositonEnd * v2.x) / sinTheta;
	result.y = (PositonStart * v1.y + PositonEnd * v2.y) / sinTheta;
	result.z = (PositonStart * v1.z + PositonEnd * v2.z) / sinTheta;

	return result;
}