import AVFoundation
import UIKit

class CameraManager: NSObject, ObservableObject {
    private var captureSession: AVCaptureSession?
    private var videoOutput: AVCaptureVideoDataOutput?
    private var videoPreviewLayer: AVCaptureVideoPreviewLayer?
    @Published var currentFrame: UIImage?
    
    override init() {
        super.init()
        setupCaptureSession()
    }
    
    func setupCaptureSession() {
        captureSession = AVCaptureSession()
        captureSession?.sessionPreset = .vga640x480  // 解像度を設定
        
        guard let captureDevice = AVCaptureDevice.default(for: .video) else {
            print("Failed to get the camera device")
            return
        }
        
        do {
            let input = try AVCaptureDeviceInput(device: captureDevice)
            if let captureSession = captureSession {
                captureSession.addInput(input)
                
                try captureDevice.lockForConfiguration()
                captureDevice.activeVideoMinFrameDuration = CMTimeMake(value: 1, timescale: 30)
                captureDevice.activeVideoMaxFrameDuration = CMTimeMake(value: 1, timescale: 30)
                captureDevice.unlockForConfiguration()
                
                videoOutput = AVCaptureVideoDataOutput()
                videoOutput?.setSampleBufferDelegate(self, queue: DispatchQueue(label: "videoQueue"))
                videoOutput?.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
                
                if captureSession.canAddOutput(videoOutput!) {
                    captureSession.addOutput(videoOutput!)
                }
                
                captureSession.startRunning()
            }
        } catch {
            print("Error setting up camera input: \(error)")
        }
    }

    private func imageFromSampleBuffer(_ sampleBuffer: CMSampleBuffer) -> UIImage? {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return nil
        }
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        if let cgImage = context.createCGImage(ciImage, from: ciImage.extent) {
            return UIImage(cgImage: cgImage)
        }
        return nil
    }
}

extension CameraManager: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        // カメラの向きを設定
        connection.videoRotationAngle = 180 // 必要に応じて .landscapeLeft などに変更
        guard let image = imageFromSampleBuffer(sampleBuffer) else { return }
        DispatchQueue.main.async {
            self.currentFrame = image
        }
    }
}
