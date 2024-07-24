import SwiftUI

struct ContentView: View {
    //MultipeerSessionオブジェクトを監視対象として設定
    //@EnvironmentObject var multipeerSession: MultipeerSession
    @State private var isSettingsAlertPresented = false
    @State private var isCameraSettingsAlertPresented = false
    @StateObject private var tcpclient = TCPClient(host: "192.168.5.211", port: 9866)
    //メッセージ用テキストフィールド
    @State var messageText = "デバイス未接続"
    @State private var inputText = "192.168.5.4"
    @State var userInput = "192.168.5.4"
    @State private var IPofCam = "192.168.5.211"
    @State var IPofCam_in = "192.168.5.211"
    @State var PortofCam:UInt16 = 9866
    @State var isConnectToCam = false
    @StateObject private var viewModel = JoystickViewModel()
    @State var image:UIImage!
    
    @State private var reset: Bool = true
    
    @State var state: Joystick.State = .ended
    @State var position: Joystick.Position = .origin
    
    @State var js_x_L: Float = 0.0  // 左側のジョイスティックのX座標
    @State var js_y_L: Float = 0.0  // 左側のジョイスティックのY座標
    @State var js_x_R: Float = 0.0  // 右側のジョイスティックのX座標
    @State var js_y_R: Float = 0.0  // 右側のジョイスティックのY座標
    
    //スライドバー
    @State private var speed = 1.0
    @State private var isEditing = false

    var body: some View {
        NavigationStack{
            ZStack{
                VStack {
                    HStack{
                        Button(action: {
                            isSettingsAlertPresented = true
                        }) {
                            Image(systemName: "gear")
                                .foregroundColor(.white)
                        }
                        .alert("接続先のIPを入力", isPresented: $isSettingsAlertPresented, actions: {
                            TextField(inputText, text: $userInput)
                                .keyboardType(.URL)
                            
                            Button("OK", action: {
                                inputText=userInput
                            })
                            Button("やめる", role: .cancel, action: {
                            })
                        })
                        Text(inputText)
                        Spacer()
                        
                        VStack{
                            Button("Call Back") {
                                viewModel.reset()
                                position = .origin
                                sendMessage(ipAddress: inputText, msg: "Go")
                            }
                            Button("Stop") {
                                viewModel.reset()
                                position = .origin
                                sendMessage(ipAddress: inputText, msg: "No")
                            }
                        }
                    }
                    Spacer()
                    HStack{
                        Button(action: {
                            isCameraSettingsAlertPresented = true
                        }) {
                            Image(systemName: "gear")
                                .foregroundColor(.white)
                        }
                        .alert("カメラのIPを入力", isPresented: $isCameraSettingsAlertPresented, actions: {
                            TextField(IPofCam, text: $IPofCam_in)
                                .keyboardType(.URL)
                            
                            Button("OK", action: {
                                IPofCam=IPofCam_in
                                tcpclient.updateConnection(host: IPofCam, port: PortofCam)
                            })
                            Button("やめる", role: .cancel, action: {
                            })
                        })
                        Text("camera:"+IPofCam)
                        Text("\(tcpclient.connectionState)")
                        Spacer()
                        NavigationLink(destination: SecondView().navigationTitle("RPiMouse Controller")) {
                            Text("RPiMouse->")
                        }
                    }
                    
//                    Slider(
//                        value: $speed,
//                        in: 0...5,
//                        onEditingChanged: { editing in
//                            isEditing = editing
//                        }
//                    )
//                    Text("\(speed)")
//                        .foregroundColor(isEditing ? .red : .blue)
                    
                    Slider(
                            value: $speed,
                            in: 0...5,
                            step: 0.01
                        ) {
                            Text("Speed")
                        } minimumValueLabel: {
                            Text("0.0")
                        } maximumValueLabel: {
                            Text("5.0")
                        } onEditingChanged: { editing in
                            isEditing = editing
                        }
                        Text("\(speed)")
                            .foregroundColor(isEditing ? .red : .blue)
                    
                    // ジョイスティックと座標表示
                    HStack {
                        // 左側のジョイスティック
                        Joystick()
                            .onChange { state, value in
                                self.state = state
                                self.position = value
                                self.js_x_L = Float(value.x) * Float(speed)  // 左側のジョイスティックのX座標を設定
                                self.js_y_L = Float(value.y) * Float(speed)  // 左側のジョイスティックのY座標を設定
                                self.sendData()
                            }
                            .frame(width:180, height: 180)
                        Spacer()
                        if(tcpclient.receivedImage != nil){
                            Image(uiImage: tcpclient.receivedImage!)
                                .resizable()
                                .scaledToFill()
                                .frame(maxWidth: 250, maxHeight: 250)
                        }
                        Spacer()
                        //右側のジョイスティック
                        Joystick()
                            .onChange { state, value in
                                self.state = state
                                self.position = value
                                self.js_x_R = Float(value.x) * Float(speed)  // 右側のジョイスティックのX座標を設定
                                self.js_y_R = Float(value.y) * Float(speed)  // 右側のジョイスティックのY座標を設定
                                self.sendData()
                            }
                            .frame(width:180, height: 180)
                    }
                    .padding()
                    
                    //Spacer()
                }
                .padding()
            }
        }
        .navigationTitle("メカナム")
    }

    private func hideKeyboard() {
        UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
    }
    
    private func sendData() {
        guard let url = URL(string: "http://" + inputText + ":5000/data") else { return }
        
        // 送信するJSONデータの設定
        let json: [String: Any] = [
            "X_L": js_x_L,
            "Y_L": js_y_L,
            "X_R": js_x_R,
            "Y_R": js_y_R
        ]
        
        // JSONデータをData型に変換
        let jsonData = try! JSONSerialization.data(withJSONObject: json, options: [])
        
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.httpBody = jsonData
        
        // JSONデータをData型に変換
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: json)
            
            var request = URLRequest(url: url)
            request.httpMethod = "POST"
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
            request.httpBody = jsonData
            
            let task = URLSession.shared.uploadTask(with: request, from: jsonData) { data, response, error in
                if let error = error {
                    print("Error: \(error)")
                    return
                }
                
                guard let httpResponse = response as? HTTPURLResponse, httpResponse.statusCode == 200 else {
                    print("Invalid response")
                    return
                }
                
                print("Data sent successfully")
            }
            
            task.resume()
        } catch {
            print("Error creating JSON data: \(error)")
        }
    }
}
