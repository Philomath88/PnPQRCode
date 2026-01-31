//
//  ARViewController.swift
//  PnPQRCode
//

import UIKit
import SceneKit
import ARKit

class ARViewController: UIViewController, ARSessionDelegate, QRTrackerDelegate {

    let sceneView = ARSCNView()
    private let qrTracker = QRTracker()

    var onDebugUpdate: ((String) -> Void)?
    var onTrackingStateChange: ((Bool) -> Void)?

    override func viewDidLoad() {
        super.viewDidLoad()

        sceneView.frame = view.bounds
        sceneView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        view.addSubview(sceneView)

        sceneView.session.delegate = self
        sceneView.automaticallyUpdatesLighting = true

        qrTracker.sceneView = sceneView
        qrTracker.delegate = self
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)

        guard ARFaceTrackingConfiguration.isSupported else {
            onDebugUpdate?("Face tracking not supported on this device")
            return
        }

        let configuration = ARFaceTrackingConfiguration()
        sceneView.session.run(configuration)
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        sceneView.session.pause()
    }

    // MARK: - ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        qrTracker.processFrame(frame)
    }

    // MARK: - QRTrackerDelegate

    nonisolated func qrTracker(_ tracker: QRTracker, didUpdate position: SCNVector3, rotation: SCNVector3, for payload: String) {
        MainActor.assumeIsolated {
            onTrackingStateChange?(true)
        }
    }

    nonisolated func qrTrackerDidLoseTracking(_ tracker: QRTracker, for payload: String) {
        MainActor.assumeIsolated {
            onTrackingStateChange?(false)
        }
    }

    nonisolated func qrTracker(_ tracker: QRTracker, didUpdateDebugInfo text: String) {
        MainActor.assumeIsolated {
            onDebugUpdate?(text)
        }
    }
}
