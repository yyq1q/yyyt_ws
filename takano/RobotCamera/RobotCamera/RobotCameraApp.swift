//
//  RobotCameraApp.swift
//  RobotCamera
//
//  Created by Takano on 2024/07/17.
//

import SwiftUI

@main
struct RobotCameraApp: App {
    //インスタンスの紐付け
    @StateObject private var multipeerSession = MultipeerSession()
    var body: some Scene {
        WindowGroup {
            ContentView()
            //インスタンスの紐付け
                .environmentObject(multipeerSession)
        }
    }
}
