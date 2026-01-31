//
//  ARViewContainer.swift
//  PnPQRCode
//

import SwiftUI

struct ARViewContainer: UIViewControllerRepresentable {
    @Binding var debugText: String
    @Binding var isTracking: Bool

    func makeUIViewController(context: Context) -> ARViewController {
        let vc = ARViewController()
        vc.onDebugUpdate = { text in
            debugText = text
        }
        vc.onTrackingStateChange = { tracking in
            isTracking = tracking
        }
        return vc
    }

    func updateUIViewController(_ uiViewController: ARViewController, context: Context) {}
}
