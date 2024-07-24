//
//  Misc.swift
//

import UIKit
import AudioToolbox

var isPad: Bool { UIDevice.current.userInterfaceIdiom == .pad }

public protocol Applicable {}
public extension Applicable {
    @discardableResult
    func apply(block: (Self) -> Void) -> Self {
        block(self)
        return self
    }
}

extension NSObject: Applicable {}

public extension UIBezierPath {
    convenience init(circleCenter center: CGPoint, radius: CGFloat) {
        self.init(arcCenter: center, radius: radius, startAngle: 0, endAngle: .pi * 2, clockwise: true)
    }
    convenience init(linePoint point: CGPoint, to: CGPoint) {
        self.init()
        self.move(to: point)
        self.addLine(to: to)
    }
}

func tick() {
    if isPad {
        let soundID = SystemSoundID(1104)
        AudioServicesPlaySystemSound(soundID)
    } else if #available(iOS 10.0, *) {
        Feedbacker.impact(style: .light)
    } else {
        let soundID = SystemSoundID(1519)
        AudioServicesPlaySystemSound(soundID)
    }
}

struct Feedbacker {
    static func notice(type: UINotificationFeedbackGenerator.FeedbackType) {
        if #available(iOS 10.0, *) {
            let generator = UINotificationFeedbackGenerator()
            generator.prepare()
            generator.notificationOccurred(type)
        }
    }
    static func impact(style: UIImpactFeedbackGenerator.FeedbackStyle) {
        if #available(iOS 10.0, *) {
            let generator = UIImpactFeedbackGenerator(style: style)
            generator.prepare()
            generator.impactOccurred()
        }
    }
    static func selection() {
        if #available(iOS 10.0, *) {
            let generator = UISelectionFeedbackGenerator()
            generator.prepare()
            generator.selectionChanged()
        }
    }
    
}

extension CGFloat {
    var int: Int { Int(self) }
}
extension Int {
    var uint8: UInt8 { UInt8(self & 0xFF) }
    var float: CGFloat { CGFloat(self) }
}
extension Character {
    var asciiCode: UInt8 { self.asciiValue ?? 0 }
    var string: String { String(self) }
}
extension String {
    var asciiCode: UInt8 { self.first?.asciiCode ?? 0 }
    var asciiCodes: [UInt8] { self.map { $0.asciiCode } }
}

extension CGPoint {
    static let origin: CGPoint = .zero
    var isOrigin: Bool { self == .zero }
    var distance: CGFloat { (self.x * self.x) + (self.y * self.y) }
}


extension String.StringInterpolation {
    mutating func appendInterpolation(_ value: CGFloat, specifier: String) {
        appendLiteral(String(format: specifier, value))
    }
}

func *(_ lhs: (x: CGFloat, y: CGFloat), _ rhs: CGFloat) -> CGPoint {
    CGPoint(x: lhs.x * rhs, y: lhs.y * rhs)
}
func *(_ lhs: CGPoint, _ rhs: Int) -> CGPoint {
    CGPoint(x: lhs.x * rhs.float, y: lhs.y * rhs.float)
}
func *(_ lhs: CGPoint, _ rhs: CGFloat) -> CGPoint {
    CGPoint(x: lhs.x * rhs, y: lhs.y * rhs)
}
func +(_ lhs: CGPoint, _ rhs: CGPoint) -> CGPoint {
    CGPoint(x: lhs.x + rhs.x, y: lhs.y + rhs.y)
}

