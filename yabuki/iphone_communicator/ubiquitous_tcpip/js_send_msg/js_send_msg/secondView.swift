import SwiftUI

struct SecondView: View {
    @State private var isSettingsAlertPresented = false
    @State private var isCameraSettingsAlertPresented = false
    @StateObject private var tcpclient_sub = TCPClient(host: "192.168.5.212", port: 9866)

    //メッセージ用テキストフィールド
    @State var messageText = "デバイス未接続"
    @State private var inputText = "192.168.5.4"
    @State var userInput = "192.168.5.4"
    @State private var IPofCam = "192.168.5.212"
    @State var IPofCam_in = "192.168.5.212"
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
                }
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
                            tcpclient_sub.updateConnection(host: IPofCam, port: PortofCam)
                        })
                        Button("やめる", role: .cancel, action: {
                        })
                    })
                    Text("camera:"+IPofCam)
                    Text("\(tcpclient_sub.connectionState)")
                    Spacer()
                }
                
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
                            js_x_L = Float(speed) * Float(value.x)
                            js_y_L = Float(speed) * Float(value.y)
                            let json: [String: Any] = [
                                "X_L": js_x_L,
                                "Y_L": js_y_L,
                                "X_R": js_x_R,
                                "Y_R": js_y_R
                            ]
                            print(inputText)
                            sendMessage(ipAddress: inputText, port: "1515", msg: json)
                        }
                        .frame(width:180, height: 180)
                    
                    Spacer()
                    
                    if(tcpclient_sub.receivedImage != nil){
                        Image(uiImage: tcpclient_sub.receivedImage!)
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
                            js_x_R = Float(speed) * Float(value.x)
                            js_y_R = Float(speed) * Float(value.y)
                            let json: [String: Any] = [
                                "X_L": js_x_L,
                                "Y_L": js_y_L,
                                "X_R": js_x_R,
                                "Y_R": js_y_R
                            ]
                            sendMessage(ipAddress: inputText, port: "1515", msg: json)
                        }
                        .frame(width:180, height: 180)
                }
                .padding()
            }
            .padding()
        }
    }
}
