import Foundation
import Network
import UIKit

class TCPServer {
    let port: NWEndpoint.Port
    var host: NWEndpoint.Host = "000.000.00.00"
    var listener: NWListener?
    var connections: [NWConnection] = []
    var connectionState: NWConnection.State = .setup
    
    init(port: UInt16) {
        self.port = NWEndpoint.Port(rawValue: port)!
        self.host = NWEndpoint.Host(self.getIPAddress()!)
        startListening()
    }
    
    private func startListening() {
        do {
            listener = try NWListener(using: .tcp, on: port)
        } catch {
            print("Failed to create listener: \(error.localizedDescription)")
            return
        }
        
        listener?.newConnectionHandler = { [weak self] newConnection in
            self?.handleNewConnection(newConnection)
        }
        
        listener?.start(queue: .main)
        print("Server is listening on port \(port)")
    }
    
    private func handleNewConnection(_ connection: NWConnection) {
        connections.append(connection)
        connection.stateUpdateHandler = { state in
            self.connectionState = state
            switch state {
            case .ready:
                print("Connection ready")
                self.receive(on: connection)
            case .failed(let error):
                print("Connection failed: \(error.localizedDescription)")
            default:
                break
            }
        }
        connection.start(queue: .main)
    }
    
    private func receive(on connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 65536) { data, context, isComplete, error in
            if let data = data, !data.isEmpty {
                // Handle received data
                self.handleReceivedData(data)
            }
            if isComplete {
                connection.cancel()
            } else if let error = error {
                print("Receive error: \(error.localizedDescription)")
                connection.cancel()
            } else {
                self.receive(on: connection)
            }
        }
    }
    
    private func handleReceivedData(_ data: Data) {
        // Process received image data
        if UIImage(data: data) != nil {
            print("Received image")
            // Handle the received image
        }
    }
    
    func send(image: UIImage) {
        guard let imageData = image.jpegData(compressionQuality: 0.0) else { return } // 圧縮してデータサイズを減らす
        
        for connection in connections {
            connection.send(content: imageData, completion: .contentProcessed { error in
                if let error = error {
                    print("Send error: \(error.localizedDescription)")
                } else {
                    print("Image sent successfully")
                }
            })
        }
    }
    
    private func getIPAddress() -> String? {
        var address: String?
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        
        guard getifaddrs(&ifaddr) == 0 else {
            return nil
        }
        defer { freeifaddrs(ifaddr) }
        
        var ptr = ifaddr
        while ptr != nil {
            defer { ptr = ptr?.pointee.ifa_next }
            
            let interface = ptr!.pointee
            let addrFamily = interface.ifa_addr.pointee.sa_family
            if addrFamily == UInt8(AF_INET) {
                let name = String(cString: interface.ifa_name)
                if name == "en0" { // Wi-Fi interface
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(interface.ifa_addr, socklen_t(interface.ifa_addr.pointee.sa_len),
                                &hostname, socklen_t(hostname.count),
                                nil, socklen_t(0), NI_NUMERICHOST)
                    address = String(cString: hostname)
                    break
                }
            }
        }
        
        return address
    }
}
