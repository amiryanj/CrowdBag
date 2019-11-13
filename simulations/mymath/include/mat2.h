#ifndef MAT3_H
#define MAT3_H

#include "vec2.h"

class Mat2
{
	public:
        float m[2][2];

		Mat2() {};
		~Mat2() {};

        inline Vec2 operator* (const Vec2& rkVec) const
        {
            return Vec2 (rkVec[0]*m[0][0] + rkVec[1]*m[0][1],
                         rkVec[0]*m[1][0] + rkVec[1]*m[1][1]);
        };

        void setIdentity(void);
        void setRot(float angle);

	protected:

	private:

};

#endif

//0003 This source file is part of OGRE
//00004     (Object-oriented Graphics Rendering Engine)
//00005 For the latest info, see http://www.ogre3d.org/
//00006
//00007 Copyright (c) 2000-2006 Torus Knot Software Ltd
//00008 Also see acknowledgements in Readme.html
//00009
//00010 This program is free software; you can redistribute it and/or modify it under
//00011 the terms of the GNU Lesser General Public License as published by the Free Software
//00012 Foundation; either version 2 of the License, or (at your option) any later
//00013 version.
//00014
//00015 This program is distributed in the hope that it will be useful, but WITHOUT
//00016 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
//00017 FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
//00018
//00019 You should have received a copy of the GNU Lesser General Public License along with
//00020 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
//00021 Place - Suite 330, Boston, MA 02111-1307, USA, or go to
//00022 http://www.gnu.org/copyleft/lesser.txt.
//00023
//00024 You may alternatively use this source under the terms of a specific version of
//00025 the OGRE Unrestricted License provided you have obtained such a license from
//00026 Torus Knot Software Ltd.
//00027 -----------------------------------------------------------------------------
//00028 */
//00029 #ifndef __Matrix3_H__
//00030 #define __Matrix3_H__
//00031
//00032 #include "OgrePrerequisites.h"
//00033
//00034 #include "OgreVector3.h"
//00035
//00036 // NB All code adapted from Wild Magic 0.2 Matrix math (free source code)
//00037 // http://www.geometrictools.com/
//00038
//00039 // NOTE.  The (x,y,z) coordinate system is assumed to be right-handed.
//00040 // Coordinate axis rotation matrices are of the form
//00041 //   RX =    1       0       0
//00042 //           0     cos(t) -sin(t)
//00043 //           0     sin(t)  cos(t)
//00044 // where t > 0 indicates a counterclockwise rotation in the yz-plane
//00045 //   RY =  cos(t)    0     sin(t)
//00046 //           0       1       0
//00047 //        -sin(t)    0     cos(t)
//00048 // where t > 0 indicates a counterclockwise rotation in the zx-plane
//00049 //   RZ =  cos(t) -sin(t)    0
//00050 //         sin(t)  cos(t)    0
//00051 //           0       0       1
//00052 // where t > 0 indicates a counterclockwise rotation in the xy-plane.
//00053
//00054 namespace Ogre
//00055 {
//00063     class _OgreExport Matrix3
//00064     {
//00065     public:
//00070         inline Matrix3 () {};
//00071         inline explicit Matrix3 (const Real arr[3][3])
//00072         {
//00073             memcpy(m,arr,9*sizeof(Real));
//00074         }
//00075         inline Matrix3 (const Matrix3& rkMatrix)
//00076         {
//00077             memcpy(m,rkMatrix.m,9*sizeof(Real));
//00078         }
//00079         Matrix3 (Real fEntry00, Real fEntry01, Real fEntry02,
//00080                     Real fEntry10, Real fEntry11, Real fEntry12,
//00081                     Real fEntry20, Real fEntry21, Real fEntry22)
//00082         {
//00083             m[0][0] = fEntry00;
//00084             m[0][1] = fEntry01;
//00085             m[0][2] = fEntry02;
//00086             m[1][0] = fEntry10;
//00087             m[1][1] = fEntry11;
//00088             m[1][2] = fEntry12;
//00089             m[2][0] = fEntry20;
//00090             m[2][1] = fEntry21;
//00091             m[2][2] = fEntry22;
//00092         }
//00093
//00094         // member access, allows use of construct mat[r][c]
//00095         inline Real* operator[] (size_t iRow) const
//00096         {
//00097             return (Real*)m[iRow];
//00098         }
//00099         /*inline operator Real* ()
//00100         {
//00101             return (Real*)m[0];
//00102         }*/
//00103         Vector3 GetColumn (size_t iCol) const;
//00104         void SetColumn(size_t iCol, const Vector3& vec);
//00105         void FromAxes(const Vector3& xAxis, const Vector3& yAxis, const Vector3& zAxis);
//00106
//00107         // assignment and comparison
//00108         inline Matrix3& operator= (const Matrix3& rkMatrix)
//00109         {
//00110             memcpy(m,rkMatrix.m,9*sizeof(Real));
//00111             return *this;
//00112         }
//00113         bool operator== (const Matrix3& rkMatrix) const;
//00114         inline bool operator!= (const Matrix3& rkMatrix) const
//00115         {
//00116             return !operator==(rkMatrix);
//00117         }
//00118
//00119         // arithmetic operations
//00120         Matrix3 operator+ (const Matrix3& rkMatrix) const;
//00121         Matrix3 operator- (const Matrix3& rkMatrix) const;
//00122         Matrix3 operator* (const Matrix3& rkMatrix) const;
//00123         Matrix3 operator- () const;
//00124
//00125         // matrix * vector [3x3 * 3x1 = 3x1]
//00126         Vector3 operator* (const Vector3& rkVector) const;
//00127
//00128         // vector * matrix [1x3 * 3x3 = 1x3]
//00129         _OgreExport friend Vector3 operator* (const Vector3& rkVector,
//00130             const Matrix3& rkMatrix);
//00131
//00132         // matrix * scalar
//00133         Matrix3 operator* (Real fScalar) const;
//00134
//00135         // scalar * matrix
//00136         _OgreExport friend Matrix3 operator* (Real fScalar, const Matrix3& rkMatrix);
//00137
//00138         // utilities
//00139         Matrix3 Transpose () const;
//00140         bool Inverse (Matrix3& rkInverse, Real fTolerance = 1e-06) const;
//00141         Matrix3 Inverse (Real fTolerance = 1e-06) const;
//00142         Real Determinant () const;
//00143
//00144         // singular value decomposition
//00145         void SingularValueDecomposition (Matrix3& rkL, Vector3& rkS,
//00146             Matrix3& rkR) const;
//00147         void SingularValueComposition (const Matrix3& rkL,
//00148             const Vector3& rkS, const Matrix3& rkR);
//00149
//00150         // Gram-Schmidt orthonormalization (applied to columns of rotation matrix)
//00151         void Orthonormalize ();
//00152
//00153         // orthogonal Q, diagonal D, upper triangular U stored as (u01,u02,u12)
//00154         void QDUDecomposition (Matrix3& rkQ, Vector3& rkD,
//00155             Vector3& rkU) const;
//00156
//00157         Real SpectralNorm () const;
//00158
//00159         // matrix must be orthonormal
//00160         void ToAxisAngle (Vector3& rkAxis, Radian& rfAngle) const;
//00161         inline void ToAxisAngle (Vector3& rkAxis, Degree& rfAngle) const {
//00162             Radian r;
//00163             ToAxisAngle ( rkAxis, r );
//00164             rfAngle = r;
//00165         }
//00166         void FromAxisAngle (const Vector3& rkAxis, const Radian& fRadians);
//00167 #ifndef OGRE_FORCE_ANGLE_TYPES
//00168         inline void ToAxisAngle (Vector3& rkAxis, Real& rfRadians) const {
//00169             Radian r;
//00170             ToAxisAngle ( rkAxis, r );
//00171             rfRadians = r.valueRadians();
//00172         }
//00173         inline void FromAxisAngle (const Vector3& rkAxis, Real fRadians) {
//00174             FromAxisAngle ( rkAxis, Radian(fRadians) );
//00175         }
//00176 #endif//OGRE_FORCE_ANGLE_TYPES
//00177
//00178         // The matrix must be orthonormal.  The decomposition is yaw*pitch*roll
//00179         // where yaw is rotation about the Up vector, pitch is rotation about the
//00180         // Right axis, and roll is rotation about the Direction axis.
//00181         bool ToEulerAnglesXYZ (Radian& rfYAngle, Radian& rfPAngle,
//00182             Radian& rfRAngle) const;
//00183         bool ToEulerAnglesXZY (Radian& rfYAngle, Radian& rfPAngle,
//00184             Radian& rfRAngle) const;
//00185         bool ToEulerAnglesYXZ (Radian& rfYAngle, Radian& rfPAngle,
//00186             Radian& rfRAngle) const;
//00187         bool ToEulerAnglesYZX (Radian& rfYAngle, Radian& rfPAngle,
//00188             Radian& rfRAngle) const;
//00189         bool ToEulerAnglesZXY (Radian& rfYAngle, Radian& rfPAngle,
//00190             Radian& rfRAngle) const;
//00191         bool ToEulerAnglesZYX (Radian& rfYAngle, Radian& rfPAngle,
//00192             Radian& rfRAngle) const;
//00193         void FromEulerAnglesXYZ (const Radian& fYAngle, const Radian& fPAngle, const Radian& fRAngle);
//00194         void FromEulerAnglesXZY (const Radian& fYAngle, const Radian& fPAngle, const Radian& fRAngle);
//00195         void FromEulerAnglesYXZ (const Radian& fYAngle, const Radian& fPAngle, const Radian& fRAngle);
//00196         void FromEulerAnglesYZX (const Radian& fYAngle, const Radian& fPAngle, const Radian& fRAngle);
//00197         void FromEulerAnglesZXY (const Radian& fYAngle, const Radian& fPAngle, const Radian& fRAngle);
//00198         void FromEulerAnglesZYX (const Radian& fYAngle, const Radian& fPAngle, const Radian& fRAngle);
//00199 #ifndef OGRE_FORCE_ANGLE_TYPES
//00200         inline bool ToEulerAnglesXYZ (float& rfYAngle, float& rfPAngle,
//00201             float& rfRAngle) const {
//00202             Radian y, p, r;
//00203             bool b = ToEulerAnglesXYZ(y,p,r);
//00204             rfYAngle = y.valueRadians();
//00205             rfPAngle = p.valueRadians();
//00206             rfRAngle = r.valueRadians();
//00207             return b;
//00208         }
//00209         inline bool ToEulerAnglesXZY (float& rfYAngle, float& rfPAngle,
//00210             float& rfRAngle) const {
//00211             Radian y, p, r;
//00212             bool b = ToEulerAnglesXZY(y,p,r);
//00213             rfYAngle = y.valueRadians();
//00214             rfPAngle = p.valueRadians();
//00215             rfRAngle = r.valueRadians();
//00216             return b;
//00217         }
//00218         inline bool ToEulerAnglesYXZ (float& rfYAngle, float& rfPAngle,
//00219             float& rfRAngle) const {
//00220             Radian y, p, r;
//00221             bool b = ToEulerAnglesYXZ(y,p,r);
//00222             rfYAngle = y.valueRadians();
//00223             rfPAngle = p.valueRadians();
//00224             rfRAngle = r.valueRadians();
//00225             return b;
//00226         }
//00227         inline bool ToEulerAnglesYZX (float& rfYAngle, float& rfPAngle,
//00228             float& rfRAngle) const {
//00229             Radian y, p, r;
//00230             bool b = ToEulerAnglesYZX(y,p,r);
//00231             rfYAngle = y.valueRadians();
//00232             rfPAngle = p.valueRadians();
//00233             rfRAngle = r.valueRadians();
//00234             return b;
//00235         }
//00236         inline bool ToEulerAnglesZXY (float& rfYAngle, float& rfPAngle,
//00237             float& rfRAngle) const {
//00238             Radian y, p, r;
//00239             bool b = ToEulerAnglesZXY(y,p,r);
//00240             rfYAngle = y.valueRadians();
//00241             rfPAngle = p.valueRadians();
//00242             rfRAngle = r.valueRadians();
//00243             return b;
//00244         }
//00245         inline bool ToEulerAnglesZYX (float& rfYAngle, float& rfPAngle,
//00246             float& rfRAngle) const {
//00247             Radian y, p, r;
//00248             bool b = ToEulerAnglesZYX(y,p,r);
//00249             rfYAngle = y.valueRadians();
//00250             rfPAngle = p.valueRadians();
//00251             rfRAngle = r.valueRadians();
//00252             return b;
//00253         }
//00254         inline void FromEulerAnglesXYZ (float fYAngle, float fPAngle, float fRAngle) {
//00255             FromEulerAnglesXYZ ( Radian(fYAngle), Radian(fPAngle), Radian(fRAngle) );
//00256         }
//00257         inline void FromEulerAnglesXZY (float fYAngle, float fPAngle, float fRAngle) {
//00258             FromEulerAnglesXZY ( Radian(fYAngle), Radian(fPAngle), Radian(fRAngle) );
//00259         }
//00260         inline void FromEulerAnglesYXZ (float fYAngle, float fPAngle, float fRAngle) {
//00261             FromEulerAnglesYXZ ( Radian(fYAngle), Radian(fPAngle), Radian(fRAngle) );
//00262         }
//00263         inline void FromEulerAnglesYZX (float fYAngle, float fPAngle, float fRAngle) {
//00264             FromEulerAnglesYZX ( Radian(fYAngle), Radian(fPAngle), Radian(fRAngle) );
//00265         }
//00266         inline void FromEulerAnglesZXY (float fYAngle, float fPAngle, float fRAngle) {
//00267             FromEulerAnglesZXY ( Radian(fYAngle), Radian(fPAngle), Radian(fRAngle) );
//00268         }
//00269         inline void FromEulerAnglesZYX (float fYAngle, float fPAngle, float fRAngle) {
//00270             FromEulerAnglesZYX ( Radian(fYAngle), Radian(fPAngle), Radian(fRAngle) );
//00271         }
//00272 #endif//OGRE_FORCE_ANGLE_TYPES
//00273         // eigensolver, matrix must be symmetric
//00274         void EigenSolveSymmetric (Real afEigenvalue[3],
//00275             Vector3 akEigenvector[3]) const;
//00276
//00277         static void TensorProduct (const Vector3& rkU, const Vector3& rkV,
//00278             Matrix3& rkProduct);
//00279
//00280         static const Real EPSILON;
//00281         static const Matrix3 ZERO;
//00282         static const Matrix3 IDENTITY;
//00283
//00284     protected:
//00285         // support for eigensolver
//00286         void Tridiagonal (Real afDiag[3], Real afSubDiag[3]);
//00287         bool QLAlgorithm (Real afDiag[3], Real afSubDiag[3]);
//00288
//00289         // support for singular value decomposition
//00290         static const Real ms_fSvdEpsilon;
//00291         static const unsigned int ms_iSvdMaxIterations;
//00292         static void Bidiagonalize (Matrix3& kA, Matrix3& kL,
//00293             Matrix3& kR);
//00294         static void GolubKahanStep (Matrix3& kA, Matrix3& kL,
//00295             Matrix3& kR);
//00296
//00297         // support for spectral norm
//00298         static Real MaxCubicRoot (Real afCoeff[3]);
//00299
//00300         Real m[3][3];
//00301
//00302         // for faster access
//00303         friend class Matrix4;
//00304     };
//00305 }
//00306 #endif

