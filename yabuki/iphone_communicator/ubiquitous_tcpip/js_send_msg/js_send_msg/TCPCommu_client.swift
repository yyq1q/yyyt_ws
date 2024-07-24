import Foundation
import Network
import UIKit
import Combine

class TCPClient: ObservableObject {
    var host: NWEndpoint.Host
    var port: NWEndpoint.Port
    var connection: NWConnection?
    var connectionState: NWConnection.State = .setup
    private var reconnectionTimer: Timer?
    private var receiveTimer: Timer?
    @Published var receivedImage: UIImage?
    private var dataBuffer = Data()
    private let startMarker = Data([0xFF, 0xD8]) // JPEG開始マーカー
    private let endMarker = Data([0xFF, 0xD9])   // JPEG終了マーカー

    init(host: String, port: UInt16) {
        self.host = NWEndpoint.Host(host)
        self.port = NWEndpoint.Port(rawValue: port)!
        startConnection()
    }

    private func startConnection() {
        connection = NWConnection(host: host, port: port, using: .tcp)
        connection?.stateUpdateHandler = { [weak self] state in
            guard let self = self else { return }
            self.connectionState = state
            switch state {
            case .ready:
                print("Client connected")
                //self.receive() // Start receiving data immediately
                self.startReceiveTimer()
                self.stopReconnectionTimer()
            case .failed(let error):
                print("Client connection failed: \(error.localizedDescription)")
                self.connection = nil
                self.stopReceiveTimer()
                self.startReconnectionTimer()
            case .waiting(let error):
                print("Client connection waiting: \(error.localizedDescription)")
            case .cancelled:
                print("Client connection cancelled")
                self.connection = nil
                self.stopReceiveTimer()
                self.startReconnectionTimer()
            default:
                break
            }
        }
        connection?.start(queue: .main)
    }

    func updateConnection(host: String, port: UInt16) {
        // Cancel the current connection if it exists
        connection?.cancel()
        
        // Update host and port
        self.host = NWEndpoint.Host(host)
        self.port = NWEndpoint.Port(rawValue: port)!
        
        // Start a new connection
        startConnection()
    }
    
    func reconnection(){
        // Cancel the current connection if it exists
        connection?.cancel()
        
        // Start a new connection
        startConnection()
    }
    
    func send(image: UIImage) {
        guard let imageData = image.pngData() else {
            print("Failed to convert image to data")
            return
        }

        connection?.send(content: imageData, completion: .contentProcessed { error in
            if let error = error {
                print("Send error: \(error.localizedDescription)")
            } else {
                print("Image sent successfully")
            }
        })
    }

//    func receive() {
//        connection?.receive(minimumIncompleteLength: 1, maximumLength: 10000000) { data, context, isComplete, error in
//                if let data = data, !data.isEmpty {
//                    // Handle received data
//                    self.handleReceivedData(data)
//                }
//                if isComplete {
//                    self.connection?.cancel()
//                } else if let error = error {
//                    print("Receive error: \(error.localizedDescription)")
//                    self.connection?.cancel()
//                } else {
//                    self.receive()
//                }
//            }
//        }
//    
//    
//        private func handleReceivedData(_ data: Data) {
//        // Process received image data
//        if UIImage(data: data) != nil {
//            print("Received image")
//            guard let image = UIImage(data: data) else { return }
//            DispatchQueue.main.async {
//                self.receivedImage = image
//            }
//            // Handle the received image
//        }
//    }
//
    private func handleReceivedData(_ data: Data) {
            dataBuffer.append(data)
            
            // 完全な画像データを探す
            while let startIndex = dataBuffer.range(of: startMarker)?.lowerBound,
                  let endIndex = dataBuffer.range(of: endMarker, in: startIndex..<dataBuffer.endIndex)?.upperBound {
                let imageData = dataBuffer.subdata(in: startIndex..<endIndex)
                
                if let image = UIImage(data: imageData) {
                    DispatchQueue.main.async {
                        self.receivedImage = image
                        print("Complete image received and set")
                    }
                } else {
                    print("Failed to create image from received data")
                }
                
                // 処理済みのデータをバッファから削除
                dataBuffer.removeSubrange(0..<endIndex)
            }
        }

        func receive() {
            connection?.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, context, isComplete, error in
                guard let self = self else { return }
                
                if let data = data, !data.isEmpty {
                    self.handleReceivedData(data)
                }
                
                if isComplete {
                    self.connection?.cancel()
                } else if let error = error {
                    print("Receive error: \(error.localizedDescription)")
                    self.connection?.cancel()
                } else {
                    self.receive()
                }
            }
        }

    private func startReceiveTimer() {
        //receiveTimer = Timer.scheduledTimer(withTimeInterval: 5.0, repeats: true) { [weak self] _ in
            //self?.receive()
        self.receive()
        //}
    }
    
    private func startReconnectionTimer(){
        reconnectionTimer = Timer.scheduledTimer(withTimeInterval: 5.0, repeats: true) { [weak self] _ in
            self?.reconnection()
        }
    }
    
    private func stopReconnectionTimer(){
        reconnectionTimer?.invalidate()
        reconnectionTimer = nil
    }

    private func stopReceiveTimer() {
        receiveTimer?.invalidate()
        receiveTimer = nil
    }
}
