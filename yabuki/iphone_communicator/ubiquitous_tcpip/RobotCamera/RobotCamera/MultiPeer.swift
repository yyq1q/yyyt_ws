//
//  MultiPeer.swift
//  iPhonoidContoroller
//
//  Created by Hamada_laboPC on 2024/06/15.
//

import Foundation
import SwiftUI
import MultipeerConnectivity
import AVFoundation

// P2P通信を扱うためのクラスMultipeerSessionを作成し、ObservableObjectプロトコルを準拠
class MultipeerSession: NSObject, ObservableObject {
    // P2P通信で使用するサービスの識別子
    private let serviceType = "ubi-connect"
    
    // デバイスの名前を使用してMCPeerIDを作成
    //パブリックに
    let myPeerId = MCPeerID(displayName: UIDevice.current.name)
    
    // サービスの広告主を作成（自分のデバイスを発見可能にする）
    private let serviceAdvertiser: MCNearbyServiceAdvertiser
    
    // サービスのブラウザを作成（他のデバイスを発見する）
    private let serviceBrowser: MCNearbyServiceBrowser
    
    // 受信したメッセージを保持するための配列、@PublishedでUIに反映
    @Published var receivedMessages: [String] = []
    
    //接続されているピアを保持するための配列
    @Published var connectedPeers: [MCPeerID] = []
    
    // セッションオブジェクトを作成
    var session: MCSession
    
    // 初期化メソッド
    override init() {
        // セッションを初期化
        self.session = MCSession(peer: myPeerId, securityIdentity: nil, encryptionPreference: .required)
        // サービスの広告主を初期化
        self.serviceAdvertiser = MCNearbyServiceAdvertiser(peer: myPeerId, discoveryInfo: nil, serviceType: serviceType)
        // サービスのブラウザを初期化
        self.serviceBrowser = MCNearbyServiceBrowser(peer: myPeerId, serviceType: serviceType)
        
        super.init()
        
        // セッションのデリゲートを設定
        self.session.delegate = self
        // サービス広告主のデリゲートを設定
        self.serviceAdvertiser.delegate = self
        // サービスブラウザのデリゲートを設定
        self.serviceBrowser.delegate = self
        
//        // 広告主を開始
//        self.serviceAdvertiser.startAdvertisingPeer()
//        // ブラウザを開始
//        self.serviceBrowser.startBrowsingForPeers()
    }
    
    func startSession(){
        self.serviceAdvertiser.startAdvertisingPeer()
//        self.serviceBrowser.startBrowsingForPeers()
    }
    
    // メッセージを送信するメソッド
    func send(message: String) {
        // 接続されているピアが存在する場合にメッセージを送信
        if session.connectedPeers.count > 0 {
            do {
                // メッセージをデータに変換して送信
                try session.send(message.data(using: .utf8)!, toPeers: session.connectedPeers, with: .reliable)
            } catch let error {
                // 送信エラー時の処理
                print("Error sending message: \(error.localizedDescription)")
            }
        }
    }
    
    func sendImage(_ image: UIImage) {
        //guard let imageData = image.pngData() else { return }
        guard let imageData = image.jpegData(compressionQuality: 0.0) else { return } // 圧縮してデータサイズを減らす
        
        
        do {
            try session.send(imageData, toPeers: session.connectedPeers, with: .reliable)
        } catch {
            print("Error sending image: \(error.localizedDescription)")
        }
    }
    
    //接続されているピアのリストを返す
    func getConnectedPeers() -> [MCPeerID] {
        return session.connectedPeers
    }
}

// MCSessionDelegateプロトコルに準拠させる
extension MultipeerSession: MCSessionDelegate {
    // ピアの接続状態が変わった時の処理
    func session(_ session: MCSession, peer peerID: MCPeerID, didChange state: MCSessionState) {
        print("Peer \(peerID.displayName) changed state to \(state.rawValue)")
        DispatchQueue.main.async{
            //接続されているピアのリストを更新
            self.connectedPeers = session.connectedPeers
        }
    }
    
    // データを受信した時の処理
    func session(_ session: MCSession, didReceive data: Data, fromPeer peerID: MCPeerID) {
        if let message = String(data: data, encoding: .utf8) {
            // メインスレッドで受信メッセージを配列に追加
            DispatchQueue.main.async {
                self.receivedMessages.append(message)
            }
        }
    }
    
    
    // 以下のメソッドはストリームやリソースの送受信に使用、今回は使用しないので空実装
    func session(_ session: MCSession, didReceive stream: InputStream, withName streamName: String, fromPeer peerID: MCPeerID) { }
    func session(_ session: MCSession, didStartReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, with progress: Progress) { }
    func session(_ session: MCSession, didFinishReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, at localURL: URL?, withError error: Error?) { }
}

// MCNearbyServiceAdvertiserDelegateプロトコルに準拠させる
extension MultipeerSession: MCNearbyServiceAdvertiserDelegate {
    // 広告主の開始に失敗した時の処理
    func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didNotStartAdvertisingPeer error: Error) {
        print("Failed to start advertising: \(error.localizedDescription)")
    }
    
    // 他のピアから招待を受け取った時の処理
    func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didReceiveInvitationFromPeer peerID: MCPeerID, withContext context: Data?, invitationHandler: @escaping (Bool, MCSession?) -> Void) {
        // 招待を受諾し、セッションを使用
        invitationHandler(true, self.session)
    }
}

// MCNearbyServiceBrowserDelegateプロトコルに準拠させる
extension MultipeerSession: MCNearbyServiceBrowserDelegate {
    // ブラウザの開始に失敗した時の処理
    func browser(_ browser: MCNearbyServiceBrowser, didNotStartBrowsingForPeers error: Error) {
        print("Failed to start browsing for peers: \(error.localizedDescription)")
    }
    
    // ピアを発見した時の処理
    func browser(_ browser: MCNearbyServiceBrowser, foundPeer peerID: MCPeerID, withDiscoveryInfo info: [String : String]?) {
        // ピアを招待
        browser.invitePeer(peerID, to: self.session, withContext: nil, timeout: 10)
    }
    
    // ピアが見つからなくなった時の処理
    func browser(_ browser: MCNearbyServiceBrowser, lostPeer peerID: MCPeerID) {
        print("Lost peer: \(peerID.displayName)")
    }
}


