//
//  NKJoystick.swift
//  Controller_test01
//
//  Created by 中嶋 義弘 on 2021/10/13.
//

import UIKit

@objc public protocol joystickDelegate {
    @objc optional func joystickBeginTracking(_ joystick: NKJoystick)
    @objc optional func joystickEndTracking(_ joystick: NKJoystick)
    @objc optional func joystickWillPositionChanged(_ joystick: NKJoystick)
    @objc optional func joystickDidPositionChanged(_ joystick: NKJoystick)
    @objc optional func joystickRepeatTracking(_ joystick: NKJoystick)
}

typealias NKJoystickPosition = CGPoint

@IBDesignable
public class NKJoystick: UIControl {
    
    
    @IBInspectable public var isHold: Bool = false
    @IBInspectable public var isRepeatable: Bool = false
    @IBInspectable public var repeatDelay: TimeInterval = .zero
    @IBInspectable public var repeatInterval: TimeInterval = .zero
    @IBOutlet public weak var delegate: joystickDelegate?
    
    fileprivate var radius: CGFloat = .zero
    fileprivate var sideLen: CGFloat = .zero
    fileprivate var boundsCenter: CGPoint = .zero
    fileprivate var currentLocation: CGPoint = .zero    //touch location
    var previousPosition: NKJoystickPosition = .zero
    var position: NKJoystickPosition = .zero {
        didSet {
            setNeedsDisplay()
        }
    }

    fileprivate var isRepeating: Bool = false
    fileprivate var repeatingTimer: Timer?

    public override func layoutSubviews() {
        super.layoutSubviews()
        let side = min(bounds.size.width, bounds.size.height) / 2
        radius = side
        sideLen = side / sqrt(2)
        boundsCenter = CGPoint(x: bounds.midY, y: bounds.midX)
    }

    override init(frame: CGRect) {
        super.init(frame: frame)
        position = .zero
    }
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    func reset() {
        currentLocation = .zero
        position = .zero
        isSelected = false
        tick()
    }
    override public func beginTracking(_ touch: UITouch, with event: UIEvent?) -> Bool {
        let location = touch.location(in: self)
        setValue(location)
        delegate?.joystickBeginTracking?(self)
        return true
    }
    
    override public func continueTracking(_ touch: UITouch, with event: UIEvent?) -> Bool {
        delegate?.joystickWillPositionChanged?(self)
        let location = touch.location(in: self)
        setValue(location)
        delegate?.joystickDidPositionChanged?(self)
        return true
    }
    
    override public func endTracking(_ touch: UITouch?, with event: UIEvent?) {
        if isHold { isSelected = true }
        if !isSelected {
            position = .zero
        }
        isRepeating = false
        delegate?.joystickEndTracking?(self)
    }
    
    override public func draw(_ rect: CGRect) {
        super.draw(rect)
        
        let radius = self.radius - 8
        
        UIBezierPath(rect: bounds).apply {
            UIColor.systemBackground.setFill()
            $0.fill()
        }

/*      draw rectage
        UIBezierPath(rect: bounds.insetBy(dx: 8, dy: 8)).apply {
            UIColor.label.setStroke()
            $0.lineWidth = 2
            $0.stroke()
        }
*/
        UIBezierPath(circleCenter: boundsCenter, radius: radius).apply {
            UIColor.label.setStroke()
            $0.lineWidth = 2
            $0.stroke()
        }
        
        let x = bounds.midX
        let y0 = bounds.midY - radius
        let y1 = bounds.midY + radius
        UIBezierPath(linePoint: CGPoint(x: x, y: y0), to: CGPoint(x: x, y: y1)).apply {
            UIColor.systemGray.setStroke()
            $0.lineWidth = 2
            $0.stroke()
        }
        
        let y = bounds.midY
        let x0 = bounds.midX - radius
        let x1 = bounds.midX + radius
        UIBezierPath(linePoint: CGPoint(x: x0, y: y), to: CGPoint(x: x1, y: y)).apply {
            UIColor.systemGray.setStroke()
            $0.lineWidth = 2
            $0.stroke()
        }
        
        //draw stick
        var stickPosition = CGPoint(x: (position.x * radius) + boundsCenter.x, y: -(position.y * radius) + boundsCenter.y)

        var radian = atan2(position.x, position.y) //* (180.0 / .pi)
        if radian < 0 { radian += .pi * 2 }
        
        let circumferencesLocation = CGPoint(x: sin(radian), y: -cos(radian))
        let circumferencesPosition = circumferencesLocation * radius + boundsCenter
        
        if position.distance > circumferencesLocation.distance {
            stickPosition = circumferencesPosition
        }
            
        UIBezierPath(linePoint: boundsCenter, to: stickPosition).apply {
            UIColor.red.setStroke()
            $0.lineWidth = 2
            $0.stroke()
        }
        UIBezierPath(circleCenter: stickPosition, radius: 6).apply {
            UIColor.red.setFill()
            $0.fill()
        }

    }
}

private extension NKJoystick {
    func setValue(_ location: CGPoint) {
        previousPosition = position
        
        currentLocation = location
        var x = currentLocation.x - boundsCenter.x
        var y = currentLocation.y - boundsCenter.y

        if x < -radius { x = -radius }
        if radius < x { x = radius }
        if y < -radius { y = -radius }
        if radius < y { y = radius }
        
        x = floor(1000 * x / radius) / 1000
        y = floor(1000 * y / radius) / 1000
        position = NKJoystickPosition(x: x, y: -y)
        
        if position == previousPosition && !isRepeatable { return }
        
        tick()
        
        if isRepeatable {
            if !isRepeating {
                isRepeating = true
                repeatingTimer = Timer.scheduledTimer(withTimeInterval: repeatDelay, repeats: false, block: { self.timeout($0) })
            }
        }

    }
    
    func timeout(_ timer: Timer?) {
        if !self.isRepeatable || !self.isRepeating {
            timer?.invalidate()
            repeatingTimer = nil
            return
        }
        
        tick()

        self.delegate?.joystickRepeatTracking?(self)
        if let timer = repeatingTimer { timer.invalidate() }
        repeatingTimer = Timer.scheduledTimer(withTimeInterval: repeatInterval, repeats: false, block: { self.timeout($0) })
    }

}

