//
//  ContentView.swift
//  RobotCamera
//
//  Created by Takano on 2024/07/17.
//

import SwiftUI

struct ContentView: View {
    @StateObject private var cameraManager = CameraManager()
    @State var tcpserver = TCPServer(port: 9866)
    //MultipeerSessionオブジェクトを監視対象として設定
    //@EnvironmentObject var multipeerSession: MultipeerSession
    //メッセージ用テキストフィールド
    @State var messageText = "デバイスが接続されていません"
    var body: some View {
        VStack {
            Text("IP:\(tcpserver.host)")
            Text("\(tcpserver.connectionState)")
            if let currentFrame = cameraManager.currentFrame {
                Image(uiImage: currentFrame)
                    .resizable()
                    .scaledToFit()
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else {
                Text("Loading...")
                    .font(.largeTitle)
            }

        }
        .padding()
        .frame(maxWidth: .infinity, maxHeight: .infinity) // VStack全体を画面の中央に寄せる
        //ビューが表示された時にピアとの接続を開始
        .onAppear{
            cameraManager.setupCaptureSession()
               // multipeerSession.startSession()
        }
        //tcpの接続状況の変化を監視
//        .onChange(of:tcpserver.connections){
//            if(tcpserver.connections=="[]"){
//                messageText = "デバイスが接続されていません"
//            }else{
//                messageText = tcpserver.connections+"に接続中"
//            }
//        }
        //connectedPeersの変化を監視
//        .onChange(of: multipeerSession.connectedPeers){ oldpeers, peers in
//            if peers.isEmpty{
//                messageText = "デバイスが接続されていません"
//            }else{
//                messageText = "\(peers.map { $0.displayName }.joined(separator: ", "))に接続されました"
//            }
//        }
        .onChange(of: cameraManager.currentFrame){
            if(cameraManager.currentFrame != nil){
                tcpserver.send(image: cameraManager.currentFrame!)
                //multipeerSession.sendImage(cameraManager.currentFrame!)
            }
        }
    }
}
