//
//  ContentView.swift
//  PnPQRCode
//

import SwiftUI

struct ContentView: View {
    @State private var debugText = "No QR detected"
    @State private var isTracking = false

    var body: some View {
        ZStack {
            ARViewContainer(debugText: $debugText, isTracking: $isTracking)
                .ignoresSafeArea()

            VStack {
                Spacer()

                HStack {
                    VStack(alignment: .leading, spacing: 4) {
                        if !isTracking {
                            Text("Point front camera at a QR code")
                                .font(.callout)
                                .foregroundStyle(.yellow)
                        }

                        Text(debugText)
                            .font(.system(.caption2, design: .monospaced))
                            .multilineTextAlignment(.leading)
                    }
                    .padding(10)
                    .background(.black.opacity(0.5))
                    .foregroundStyle(.white)
                    .clipShape(RoundedRectangle(cornerRadius: 8))
                    .padding(.leading, 8)
                    .padding(.bottom, 8)

                    Spacer()
                }
            }
        }
    }
}
