# PnPQRCode

A minimal iOS sample project that demonstrates **real-time 3D pose estimation of QR codes** using a pure Swift PnP (Perspective-n-Point) solver — no OpenCV required.

Hold a printed QR code in front of your iPhone's front camera and watch 3D coordinate axes track the code's position and orientation in real time.

[![Demo](https://img.youtube.com/vi/u3uf9mGnBHQ/maxresdefault.jpg)](https://youtu.be/u3uf9mGnBHQ)

## What makes this interesting

The core of this project is a **pure Swift PnP solver** (`PnPSolver.swift`) that computes the full 6-DoF pose (position + orientation) of a planar target from its four corner points. The pipeline:

1. **QR detection** via Apple's Vision framework (`VNDetectBarcodesRequest`)
2. **Corner ordering** — tries both possible corner label orderings and picks the one with lower reprojection error, working around inconsistent `VNBarcodeObservation` corner labels
3. **DLT homography** — builds an 8×9 system from the 4 corner correspondences
4. **LAPACK SVD** (`sgesvd_`) — solves for the null space to get the homography matrix
5. **Polar decomposition** — orthogonalizes the rotation via a second SVD pass, ensuring a proper rotation matrix (det = +1)
6. **Normal constraint** — enforces that the QR surface normal faces the camera, eliminating the planar pose ambiguity
7. **Reprojection validation** — selects the pose with lowest reprojection error
8. **Quaternion smoothing** — 5-frame rolling SLERP average for temporal stability
9. **Position smoothing** — 5-frame rolling average for stable translation
10. **ARKit integration** — transforms the camera-frame pose into world space using ARKit's camera transform and intrinsics

All math runs on the CPU using `simd` types and Accelerate/LAPACK. No third-party dependencies.

## Requirements

- Physical iOS device with a **TrueDepth front camera** (iPhone X or later)
- iOS 18.0+
- Xcode 26+

The app uses `ARFaceTrackingConfiguration` to access the front camera with known intrinsics. A simulator won't work — you need a real device.

## Setup

1. Clone this repo and open `PnPQRCode.xcodeproj` in Xcode
2. Select your physical device as the build target
3. Build and run
4. Print a QR code (any content works) and measure its side length
5. Set `QRTracker.qrHalfSize` to **half** the measured side length in meters
   - Default: `0.0125` (for a 2.5 cm QR code)
   - Example: 5 cm QR → set to `0.025`
6. Hold the QR code in front of the front camera

You should see:
- **RGB coordinate axes** (red = X, green = Y with arrow, blue = Z) anchored to the QR code
- A **green arrow** indicating the QR code's "up" direction
- A **semi-transparent plane** at the QR surface
- A **debug overlay** (bottom-left) showing distance and Euler angles

## File structure

```
PnPQRCode/
├── PnPQRCodeApp.swift      # App entry point
├── ContentView.swift        # SwiftUI root view with debug overlay
├── ARViewContainer.swift    # UIViewControllerRepresentable bridge
├── ARViewController.swift   # ARSCNView + ARSession management
├── QRTracker.swift          # QR detection, PnP integration, visualization
└── PnPSolver.swift          # Pure Swift PnP solver (the interesting part)
```

## Known limitations

- **Front camera only** — uses `ARFaceTrackingConfiguration` for camera intrinsics. Adapting to the rear camera would require a different AR configuration.
- **Planar targets only** — the homography-based approach assumes all object points lie on a plane (z = 0), which is true for QR codes but not for general 3D objects.
- **QR size must be known** — the solver needs the physical size of the QR code. There's no way to infer it from a single camera view without additional constraints.
- **Single QR at a time** — while the code tracks multiple QR codes by payload, performance is best with one.
- **LAPACK deprecation warnings** — the CLAPACK interface used by `sgesvd_` is deprecated. Functionally correct but produces compiler warnings.

## Support

If you find this useful, consider buying me a coffee:

[![Buy Me A Coffee](https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png)](https://www.buymeacoffee.com/philomath88)

## License

MIT — see [LICENSE](LICENSE).
