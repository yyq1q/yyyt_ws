import Foundation

func sendMessage(ipAddress: String, msg: String) {
    guard let url = URL(string: "http://" + ipAddress + ":5555/message") else { return }
    
    var request = URLRequest(url: url)
    request.httpMethod = "POST"
    request.setValue("application/json", forHTTPHeaderField: "Content-Type")
    
    let jsonData: [String: Any] = ["message": msg]
    
    do {
        request.httpBody = try JSONSerialization.data(withJSONObject: jsonData, options: [])
    } catch {
        print("Error serializing JSON: \(error)")
        return
    }
    
    let task = URLSession.shared.dataTask(with: request) { data, response, error in
        if let error = error {
            print("Error sending message: \(error)")
            return
        }
        if let data = data, let responseString = String(data: data, encoding: .utf8) {
            print("Response from server: \(responseString)")
        }
    }
    task.resume()
}

func sendMessage(ipAddress: String, port: String, msg: [String: Any]) {
    guard let url = URL(string: "http://" + ipAddress + ":" + port + "/data") else { return }
    
    var request = URLRequest(url: url)
    request.httpMethod = "POST"
    request.setValue("application/json", forHTTPHeaderField: "Content-Type")
    
    do {
        request.httpBody = try JSONSerialization.data(withJSONObject: msg, options: [])
    } catch {
        print("Error serializing JSON: \(error)")
        return
    }
    
    let task = URLSession.shared.dataTask(with: request) { data, response, error in
        if let error = error {
            print("Error sending message: \(error)")
            return
        }
        if let data = data, let responseString = String(data: data, encoding: .utf8) {
            print("Response from server: \(responseString)")
        }
    }
    task.resume()
}
