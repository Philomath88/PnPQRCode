//
//  PnPSolver.swift
//  PnPQRCode
//

import simd
import Accelerate

/// Pure-math PnP solver for planar targets (QR codes).
/// No UIKit / SceneKit / ARKit dependencies.
nonisolated class PnPSolver {

    /// Rolling window of quaternions for temporal smoothing.
    private var quatHistory: [simd_quatf] = []

    /// Solve PnP for a planar target (z=0) via homography decomposition.
    /// - Parameters:
    ///   - imagePoints: 4 corners in buffer pixel coordinates
    ///   - objectPoints: 4 matching corners in the object's XY plane (meters)
    ///   - intrinsics: camera intrinsic matrix
    /// - Returns: (R, t) in camera intrinsic frame (+x right, +y down, +z forward)
    func solve(imagePoints: [simd_float2],
               objectPoints: [simd_float2],
               intrinsics: simd_float3x3) -> (rotation: simd_float3x3, translation: simd_float3)? {
        guard imagePoints.count == 4, objectPoints.count == 4 else { return nil }

        // 1. Normalize image points by intrinsics: p_norm = K^-1 * p_pixel
        let fx = intrinsics[0][0]
        let fy = intrinsics[1][1]
        let cx = intrinsics[2][0]
        let cy = intrinsics[2][1]

        let normPts = imagePoints.map { p in
            simd_float2((p.x - cx) / fx, (p.y - cy) / fy)
        }

        // 2. Build DLT matrix A (8×9) for homography: A·h = 0
        var A = [Float](repeating: 0, count: 8 * 9) // column-major for LAPACK

        for i in 0..<4 {
            let X = objectPoints[i].x
            let Y = objectPoints[i].y
            let u = normPts[i].x
            let v = normPts[i].y
            let r1 = i * 2
            let r2 = i * 2 + 1

            // Row r1: [X, Y, 1, 0, 0, 0, -uX, -uY, -u]
            A[r1 + 0 * 8] =  X;  A[r1 + 1 * 8] =  Y;  A[r1 + 2 * 8] =  1
            A[r1 + 3 * 8] =  0;  A[r1 + 4 * 8] =  0;  A[r1 + 5 * 8] =  0
            A[r1 + 6 * 8] = -u*X; A[r1 + 7 * 8] = -u*Y; A[r1 + 8 * 8] = -u

            // Row r2: [0, 0, 0, X, Y, 1, -vX, -vY, -v]
            A[r2 + 0 * 8] =  0;  A[r2 + 1 * 8] =  0;  A[r2 + 2 * 8] =  0
            A[r2 + 3 * 8] =  X;  A[r2 + 4 * 8] =  Y;  A[r2 + 5 * 8] =  1
            A[r2 + 6 * 8] = -v*X; A[r2 + 7 * 8] = -v*Y; A[r2 + 8 * 8] = -v
        }

        // 3. SVD of A (8×9) → null space is last row of VT
        var m: __CLPK_integer = 8
        var n: __CLPK_integer = 9
        var lda: __CLPK_integer = 8
        var s = [Float](repeating: 0, count: 8)
        var dummyU = [Float](repeating: 0, count: 1)
        var ldu: __CLPK_integer = 1
        var vt = [Float](repeating: 0, count: 9 * 9)
        var ldvt: __CLPK_integer = 9
        var work = [Float](repeating: 0, count: 1)
        var lwork: __CLPK_integer = -1
        var info: __CLPK_integer = 0
        var jobu = Int8(UInt8(ascii: "N"))
        var jobvt = Int8(UInt8(ascii: "A"))

        // Query optimal workspace
        sgesvd_(&jobu, &jobvt, &m, &n, &A, &lda, &s, &dummyU, &ldu, &vt, &ldvt, &work, &lwork, &info)
        lwork = __CLPK_integer(work[0])
        work = [Float](repeating: 0, count: Int(lwork))

        // Compute SVD
        sgesvd_(&jobu, &jobvt, &m, &n, &A, &lda, &s, &dummyU, &ldu, &vt, &ldvt, &work, &lwork, &info)
        guard info == 0 else {
            print("⚠️ PnP: SVD failed (info=\(info))")
            return nil
        }

        // Last row of VT (row index 8) = null space of A
        var h = [Float](repeating: 0, count: 9)
        for j in 0..<9 { h[j] = vt[8 + j * 9] }

        // 4. Reshape h into 3×3 homography H (row-major in h)
        let col0 = simd_float3(h[0], h[3], h[6])
        let col1 = simd_float3(h[1], h[4], h[7])
        let col2 = simd_float3(h[2], h[5], h[8])

        // 5. Recover scale
        let n0 = simd_length(col0)
        let n1 = simd_length(col1)
        guard n0 > 1e-8, n1 > 1e-8 else { return nil }
        var lambda = (n0 + n1) / 2.0

        // Ensure marker is in front of camera (t.z > 0)
        let tTest = col2 / lambda
        if tTest.z < 0 { lambda = -lambda }

        let r1 = col0 / lambda
        let r2 = col1 / lambda
        let t  = col2 / lambda

        // 6. r3 = r1 × r2
        let r3 = simd_cross(r1, r2)

        // 7. Orthogonalize R via SVD polar decomposition
        var Rflat: [Float] = [
            r1.x, r1.y, r1.z,
            r2.x, r2.y, r2.z,
            r3.x, r3.y, r3.z
        ]
        var m3: __CLPK_integer = 3
        var n3: __CLPK_integer = 3
        var lda3: __CLPK_integer = 3
        var s3 = [Float](repeating: 0, count: 3)
        var u3 = [Float](repeating: 0, count: 9)
        var ldu3: __CLPK_integer = 3
        var vt3 = [Float](repeating: 0, count: 9)
        var ldvt3: __CLPK_integer = 3
        var work3 = [Float](repeating: 0, count: 1)
        var lwork3: __CLPK_integer = -1
        var info3: __CLPK_integer = 0
        var jobu3 = Int8(UInt8(ascii: "A"))
        var jobvt3 = Int8(UInt8(ascii: "A"))

        sgesvd_(&jobu3, &jobvt3, &m3, &n3, &Rflat, &lda3, &s3, &u3, &ldu3, &vt3, &ldvt3, &work3, &lwork3, &info3)
        lwork3 = __CLPK_integer(work3[0])
        work3 = [Float](repeating: 0, count: Int(lwork3))
        sgesvd_(&jobu3, &jobvt3, &m3, &n3, &Rflat, &lda3, &s3, &u3, &ldu3, &vt3, &ldvt3, &work3, &lwork3, &info3)
        guard info3 == 0 else {
            print("⚠️ PnP: rotation SVD failed")
            return nil
        }

        let U3 = simd_float3x3(columns: (
            simd_float3(u3[0], u3[1], u3[2]),
            simd_float3(u3[3], u3[4], u3[5]),
            simd_float3(u3[6], u3[7], u3[8])
        ))
        let VT3 = simd_float3x3(columns: (
            simd_float3(vt3[0], vt3[1], vt3[2]),
            simd_float3(vt3[3], vt3[4], vt3[5]),
            simd_float3(vt3[6], vt3[7], vt3[8])
        ))
        var R = U3 * VT3

        // 8. Ensure det(R) = +1 (proper rotation)
        let det = R.columns.0.x * (R.columns.1.y * R.columns.2.z - R.columns.1.z * R.columns.2.y)
                - R.columns.0.y * (R.columns.1.x * R.columns.2.z - R.columns.1.z * R.columns.2.x)
                + R.columns.0.z * (R.columns.1.x * R.columns.2.y - R.columns.1.y * R.columns.2.x)
        if det < 0 {
            let Ucorr = simd_float3x3(columns: (U3.columns.0, U3.columns.1, -U3.columns.2))
            R = Ucorr * VT3
        }

        // 9. Normal constraint: QR surface normal must face the camera.
        //    R.columns.2 is the object Z axis (surface normal) in camera frame.
        //    t points from camera to QR. A QR facing the camera has its normal
        //    pointing opposite to t, so dot(normal, t) should be negative.
        //    Only correct when clearly wrong (dot > 0.5) to avoid false triggers
        //    at steep viewing angles where noise could oscillate the sign.
        let tDir = simd_normalize(t)
        if simd_dot(R.columns.2, tDir) > 0.5 {
            // R' = R * Rx(π) where Rx(π) = diag(1, -1, -1)
            R = simd_float3x3(columns: (R.columns.0, -R.columns.1, -R.columns.2))
        }

        return (R, t)
    }

    /// Apply 5-frame quaternion temporal smoothing to a rotation matrix.
    /// Returns a stabilized quaternion via iterative SLERP averaging.
    func stabilize(_ rotation: simd_float3x3) -> simd_quatf {
        var q = simd_quatf(rotation)

        // Ensure consistent hemisphere with the most recent frame
        if let prev = quatHistory.last {
            if simd_dot(q, prev) < 0 {
                q = simd_quatf(ix: -q.imag.x, iy: -q.imag.y, iz: -q.imag.z, r: -q.real)
            }
        }

        // Add to rolling window
        quatHistory.append(q)
        if quatHistory.count > 5 {
            quatHistory.removeFirst()
        }

        // Average quaternions by iterative SLERP (more weight to recent frames)
        var avg = quatHistory[0]
        for i in 1..<quatHistory.count {
            var qi = quatHistory[i]
            if simd_dot(qi, avg) < 0 {
                qi = simd_quatf(ix: -qi.imag.x, iy: -qi.imag.y, iz: -qi.imag.z, r: -qi.real)
            }
            avg = simd_slerp(avg, qi, 1.0 / Float(i + 1))
        }
        return simd_normalize(avg)
    }

    /// Compute mean reprojection error (pixels²) for a given pose.
    func reprojectionError(imagePoints: [simd_float2],
                           objectPoints: [simd_float2],
                           rotation: simd_float3x3,
                           translation: simd_float3,
                           intrinsics: simd_float3x3) -> Float {
        let fx = intrinsics[0][0]
        let fy = intrinsics[1][1]
        let cx = intrinsics[2][0]
        let cy = intrinsics[2][1]

        var total: Float = 0
        for i in 0..<objectPoints.count {
            let objPt = simd_float3(objectPoints[i].x, objectPoints[i].y, 0)
            let camPt = rotation * objPt + translation
            guard camPt.z > 1e-6 else { return Float.greatestFiniteMagnitude }
            let projX = fx * camPt.x / camPt.z + cx
            let projY = fy * camPt.y / camPt.z + cy
            let dx = projX - imagePoints[i].x
            let dy = projY - imagePoints[i].y
            total += dx * dx + dy * dy
        }
        return total / Float(objectPoints.count)
    }

    /// Clear smoothing history (call when QR tracking is lost).
    func reset() {
        quatHistory.removeAll()
    }
}
