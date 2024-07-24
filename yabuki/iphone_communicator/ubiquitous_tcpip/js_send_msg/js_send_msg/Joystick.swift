import SwiftUI

struct Joystick: UIViewRepresentable {

    enum State {
        case began, moved, ended, repeating
    }

    typealias Position = CGPoint

    private var isHold: Bool? = nil
    private var isRepeatable: Bool? = nil
    private var repeatDelay: TimeInterval? = nil
    private var repeatInterval: TimeInterval? = nil

    typealias Callback = (State, Position) -> Void
    private var onChange: Callback?

    let setViewHandler: ((NKJoystick) -> Void)?
    init(_ setViewHandler: ((NKJoystick) -> Void)? = nil) {
        self.setViewHandler = setViewHandler
      //isHold = true
    }
    func makeUIView(context: Context) -> NKJoystick {
        let view = NKJoystick(frame: CGRect.zero)
        setViewHandler?(view)
        view.delegate = context.coordinator
        return view
    }
        
    func updateUIView(_ joystick: NKJoystick, context: Context) {
        if let isHold = isHold { joystick.isHold = isHold }
        if let isRepeatable = isRepeatable { joystick.isRepeatable = isRepeatable }
        if let repeatDelay = repeatDelay { joystick.repeatDelay = repeatDelay }
        if let repeatInterval = repeatInterval { joystick.repeatInterval = repeatInterval }

        context.coordinator.onChangeCallback = onChange
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }
    
    func onChange(callback: @escaping Callback) -> Self {
        var view = self
        view.onChange = callback
        return view
    }

    class Coordinator: NSObject, joystickDelegate {
        let parent: Joystick
        var onChangeCallback: Callback?
        
        init(_ nkJoystick: Joystick) {
            parent = nkJoystick
        }
        
        func joystickBeginTracking(_ joystick: NKJoystick) {
            onChangeCallback?(.began, joystick.position)
        }
        func joystickEndTracking(_ joystick: NKJoystick) {
            onChangeCallback?(.ended, joystick.position)
        }
        func joystickWillPositionChanged(_ joystick: NKJoystick) {
        }
        func joystickDidPositionChanged(_ joystick: NKJoystick) {
            onChangeCallback?(.moved, joystick.position)
        }
        func joystickRepeatTracking(_ joystick: NKJoystick) {
            onChangeCallback?(.repeating, joystick.position)
        }
    }
}

extension Joystick {
    func hold(_ hold: Bool) -> Self {
        var view = self
        view.isHold = hold
        return view
    }
    func repeatable(_ repeatable: Bool, interval: TimeInterval = 0.2, delay: TimeInterval = 1.0) -> Self {
        var view = self
        view.isRepeatable = repeatable
        view.repeatDelay = delay
        view.repeatInterval = interval
        return view
    }
}


class JoystickViewModel: ObservableObject {
    private weak var joystick: NKJoystick?
    
    func setJoystick(_ joystick: NKJoystick) {
        self.joystick = joystick
    }
    
    func reset() {
        joystick?.reset()
    }
}
