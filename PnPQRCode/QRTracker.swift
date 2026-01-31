//
//  QRTracker.swift
//  PnPQRCode
//

import UIKit
import SceneKit
import ARKit
import Vision

protocol QRTrackerDelegate: AnyObject {
    func qrTracker(_ tracker: QRTracker, didUpdate position: SCNVector3, rotation: SCNVector3, for payload: String)
    func qrTrackerDidLoseTracking(_ tracker: QRTracker, for payload: String)
    func qrTracker(_ tracker: QRTracker, didUpdateDebugInfo text: String)
}

class QRTracker {

    weak var delegate: QRTrackerDelegate?
    var sceneView: ARSCNView!

    /// Half the physical side length of the printed QR code (in meters).
    /// Measure your printed QR and set this accordingly.
    /// Example: a 2.5 cm QR code → qrHalfSize = 0.0125
    static let qrHalfSize: Float = 0.0125

    // Visualization nodes keyed by payload
    private(set) var markerNodes: [String: SCNNode] = [:]

    // Detection state
    private var isDetecting = false
    private var frameCount = 0
    private let detectionInterval = 3

    // Temporal smoothing (5-frame rolling window)
    private var positionHistory: [String: [SCNVector3]] = [:]
    private var lastDetectedRotations: [String: SCNVector3] = [:]
    private let smoothingWindow = 5
    private let smoothingDuration: TimeInterval = 0.1

    // Frame counter for hiding when QR not detected
    private var framesWithoutDetection: [String: Int] = [:]
    private let maxFramesWithoutDetection = 30

    // Per-payload PnP solvers (each has its own quaternion smoothing history)
    private var pnpSolvers: [String: PnPSolver] = [:]

    // MARK: - Public

    func processFrame(_ frame: ARFrame) {
        frameCount += 1
        guard frameCount % detectionInterval == 0 else { return }
        guard !isDetecting else { return }

        let pixelBuffer = frame.capturedImage
        let imageWidth = CVPixelBufferGetWidth(pixelBuffer)
        let imageHeight = CVPixelBufferGetHeight(pixelBuffer)
        let cameraTransform = frame.camera.transform
        let cameraIntrinsics = frame.camera.intrinsics

        isDetecting = true

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }

            self.detectBarcodes(in: pixelBuffer,
                              imageWidth: imageWidth,
                              imageHeight: imageHeight,
                              cameraTransform: cameraTransform,
                              cameraIntrinsics: cameraIntrinsics)

            self.isDetecting = false
        }
    }

    // MARK: - Detection

    private func detectBarcodes(in pixelBuffer: CVPixelBuffer,
                                imageWidth: Int,
                                imageHeight: Int,
                                cameraTransform: simd_float4x4,
                                cameraIntrinsics: simd_float3x3) {
        let barcodeRequest = VNDetectBarcodesRequest { [weak self] request, error in
            guard let self = self else { return }

            if let error = error {
                print("Barcode detection error: \(error)")
                return
            }

            guard let results = request.results as? [VNBarcodeObservation] else { return }

            DispatchQueue.main.async {
                self.updateBarcodeBoxes(with: results,
                                       imageWidth: imageWidth,
                                       imageHeight: imageHeight,
                                       cameraTransform: cameraTransform,
                                       cameraIntrinsics: cameraIntrinsics)
            }
        }

        barcodeRequest.symbologies = [.qr]

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .leftMirrored, options: [:])

        do {
            try handler.perform([barcodeRequest])
        } catch {
            print("Failed to perform barcode detection: \(error)")
        }
    }

    private func updateBarcodeBoxes(with observations: [VNBarcodeObservation],
                                    imageWidth: Int,
                                    imageHeight: Int,
                                    cameraTransform: simd_float4x4,
                                    cameraIntrinsics: simd_float3x3) {
        var seenBarcodes = Set<String>()

        for observation in observations {
            let payload = observation.payloadStringValue ?? "unknown"
            seenBarcodes.insert(payload)

            let markerNode = getOrCreateMarkerNode(for: payload)

            guard let (worldPosition, rotation) = convertVisionToWorldSpace(
                observation: observation,
                payload: payload,
                imageWidth: imageWidth,
                imageHeight: imageHeight,
                cameraTransform: cameraTransform,
                cameraIntrinsics: cameraIntrinsics
            ) else { continue }

            // Rolling position history for temporal smoothing
            var history = positionHistory[payload] ?? []
            history.append(worldPosition)
            if history.count > smoothingWindow { history.removeFirst() }
            positionHistory[payload] = history

            let n = Float(history.count)
            var sx: Float = 0, sy: Float = 0, sz: Float = 0
            for p in history { sx += p.x; sy += p.y; sz += p.z }
            let smoothedPosition = SCNVector3(sx / n, sy / n, sz / n)

            lastDetectedRotations[payload] = rotation
            framesWithoutDetection[payload] = 0

            markerNode.removeAllActions()

            let moveAction = SCNAction.move(to: smoothedPosition, duration: smoothingDuration)
            let rotateAction = SCNAction.rotateTo(x: CGFloat(rotation.x),
                                                 y: CGFloat(rotation.y),
                                                 z: CGFloat(rotation.z),
                                                 duration: smoothingDuration)
            let groupAction = SCNAction.group([moveAction, rotateAction])

            markerNode.runAction(groupAction)
            markerNode.isHidden = false

            delegate?.qrTracker(self, didUpdate: smoothedPosition, rotation: rotation, for: payload)

            updateMarkerDebugLabel(payload: payload, worldPos: smoothedPosition, visionBounds: observation.boundingBox, rotation: rotation)
        }

        for (payload, node) in markerNodes {
            if !seenBarcodes.contains(payload) {
                let currentCount = framesWithoutDetection[payload] ?? 0
                framesWithoutDetection[payload] = currentCount + 1

                if currentCount >= maxFramesWithoutDetection {
                    node.isHidden = true
                    positionHistory[payload] = nil
                    pnpSolvers[payload]?.reset()
                    delegate?.qrTrackerDidLoseTracking(self, for: payload)
                }
            }
        }

        if seenBarcodes.isEmpty {
            delegate?.qrTracker(self, didUpdateDebugInfo: "No QR detected")
        }
    }

    // MARK: - Visualization

    /// Creates a 3D coordinate axes marker: R/G/B cylinders for X/Y/Z
    /// plus a semi-transparent plane at the QR surface.
    private func getOrCreateMarkerNode(for payload: String) -> SCNNode {
        if let existingNode = markerNodes[payload] {
            return existingNode
        }

        let axisLength: CGFloat = 0.03
        let axisRadius: CGFloat = 0.001

        let root = SCNNode()

        // Semi-transparent plane at QR surface
        let planeSize = CGFloat(QRTracker.qrHalfSize * 2)
        let plane = SCNPlane(width: planeSize, height: planeSize)
        plane.firstMaterial?.diffuse.contents = UIColor.white.withAlphaComponent(0.3)
        plane.firstMaterial?.isDoubleSided = true
        let planeNode = SCNNode(geometry: plane)
        root.addChildNode(planeNode)

        // X axis — Red
        let xCyl = SCNCylinder(radius: axisRadius, height: axisLength)
        xCyl.firstMaterial?.diffuse.contents = UIColor.red
        let xNode = SCNNode(geometry: xCyl)
        xNode.eulerAngles.z = -.pi / 2
        xNode.position.x = Float(axisLength / 2)
        root.addChildNode(xNode)

        // Y axis — Green
        let yCyl = SCNCylinder(radius: axisRadius, height: axisLength)
        yCyl.firstMaterial?.diffuse.contents = UIColor.green
        let yNode = SCNNode(geometry: yCyl)
        yNode.position.y = Float(axisLength / 2)
        root.addChildNode(yNode)

        // Z axis — Blue
        let zCyl = SCNCylinder(radius: axisRadius, height: axisLength)
        zCyl.firstMaterial?.diffuse.contents = UIColor.blue
        let zNode = SCNNode(geometry: zCyl)
        zNode.eulerAngles.x = .pi / 2
        zNode.position.z = Float(axisLength / 2)
        root.addChildNode(zNode)

        sceneView.scene.rootNode.addChildNode(root)

        markerNodes[payload] = root
        return root
    }

    // MARK: - Coordinate Conversion

    private func convertVisionToWorldSpace(observation: VNBarcodeObservation,
                                           payload: String,
                                           imageWidth: Int,
                                           imageHeight: Int,
                                           cameraTransform: simd_float4x4,
                                           cameraIntrinsics: simd_float3x3) -> (position: SCNVector3, rotation: SCNVector3)? {
        let solver = pnpSolvers[payload] ?? {
            let s = PnPSolver()
            pnpSolvers[payload] = s
            return s
        }()
        let halfSize = QRTracker.qrHalfSize
        let objPts: [simd_float2] = [
            simd_float2(-halfSize, -halfSize),
            simd_float2( halfSize, -halfSize),
            simd_float2( halfSize,  halfSize),
            simd_float2(-halfSize,  halfSize),
        ]

        // KNOWN BUG: VNBarcodeObservation swaps topRight <-> bottomLeft
        let correctedTL = observation.topLeft
        let correctedTR = observation.bottomLeft
        let correctedBR = observation.bottomRight
        let correctedBL = observation.topRight

        // .leftMirrored (EXIF 5): bufU = (1-visionY)*W, bufV = visionX*H
        func toBufPx(_ p: CGPoint) -> simd_float2 {
            simd_float2(Float(1.0 - p.y) * Float(imageWidth),
                        Float(p.x) * Float(imageHeight))
        }
        let imgPts: [simd_float2] = [
            toBufPx(correctedTL),
            toBufPx(correctedTR),
            toBufPx(correctedBR),
            toBufPx(correctedBL),
        ]

        guard let pnp = solver.solve(imagePoints: imgPts,
                                         objectPoints: objPts,
                                         intrinsics: cameraIntrinsics) else {
            return nil
        }

        // Convert intrinsic camera frame -> ARKit camera frame
        let F = simd_float3x3(diagonal: simd_float3(1, -1, -1))
        let R_ark = F * pnp.rotation
        let t_ark = F * pnp.translation

        // Transform to world space
        let camRot = simd_float3x3(
            simd_float3(cameraTransform.columns.0.x, cameraTransform.columns.0.y, cameraTransform.columns.0.z),
            simd_float3(cameraTransform.columns.1.x, cameraTransform.columns.1.y, cameraTransform.columns.1.z),
            simd_float3(cameraTransform.columns.2.x, cameraTransform.columns.2.y, cameraTransform.columns.2.z)
        )
        let camPos = simd_float3(cameraTransform.columns.3.x,
                                 cameraTransform.columns.3.y,
                                 cameraTransform.columns.3.z)

        let worldPos = camPos + camRot * t_ark
        let R_world = camRot * R_ark

        // Quaternion stabilization
        let q_stable = solver.stabilize(R_world)

        // Extract euler angles from stabilized quaternion
        let rm = simd_float3x3(q_stable)
        let sy = sqrt(rm.columns.0.x * rm.columns.0.x + rm.columns.0.y * rm.columns.0.y)
        let rotX: Float, rotY: Float, rotZ: Float
        if sy > 1e-6 {
            rotX = atan2(rm.columns.2.y, rm.columns.2.z)
            rotY = atan2(-rm.columns.2.x, sy)
            rotZ = atan2(rm.columns.0.y, rm.columns.0.x)
        } else {
            rotX = atan2(-rm.columns.1.z, rm.columns.1.y)
            rotY = atan2(-rm.columns.2.x, sy)
            rotZ = 0
        }

        return (SCNVector3(worldPos.x, worldPos.y, worldPos.z), SCNVector3(rotX, rotY, rotZ))
    }

    private func updateMarkerDebugLabel(payload: String, worldPos: SCNVector3, visionBounds: CGRect, rotation: SCNVector3) {
        let cameraPos = sceneView.pointOfView?.worldPosition ?? SCNVector3Zero
        let dx = worldPos.x - cameraPos.x
        let dy = worldPos.y - cameraPos.y
        let dz = worldPos.z - cameraPos.z
        let distance = sqrt(dx*dx + dy*dy + dz*dz)

        let rad2deg: Float = 180.0 / .pi
        let text = """
        QR: \(payload.prefix(20))
        Distance: \(String(format: "%.2f", distance))m

        Position (m):
        X: \(String(format: "%.3f", worldPos.x))
        Y: \(String(format: "%.3f", worldPos.y))
        Z: \(String(format: "%.3f", worldPos.z))

        Rotation:
        X: \(String(format: "%.1f", rotation.x * rad2deg))°
        Y: \(String(format: "%.1f", rotation.y * rad2deg))°
        Z: \(String(format: "%.1f", rotation.z * rad2deg))°
        """

        delegate?.qrTracker(self, didUpdateDebugInfo: text)
    }
}
