#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# ===============================
# Utility
# ===============================
def rotation_to_euler_xyz(R):
    rx = np.arctan2(R[2,1], R[2,2])
    ry = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    rz = np.arctan2(R[1,0], R[0,0])
    return np.degrees([rx, ry, rz])

def draw_rotation_text(img, R, euler, origin=(10, 80)):
    x, y = origin
    for i in range(3):
        text = f"R{i}: [{R[i,0]:+.2f} {R[i,1]:+.2f} {R[i,2]:+.2f}]"
        cv2.putText(img, text, (x, y + i*22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    rx, ry, rz = euler
    cv2.putText(img, f"RX={rx:+.1f}  RY={ry:+.1f}  RZ={rz:+.1f}",
                (x, y + 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

def draw_axis(img, center_px, R, scale=80):
    cx, cy = center_px
    colors = [(0,0,255), (0,255,0), (255,0,0)]  # X,Y,Z
    for i in range(3):
        end = (
            int(cx + R[0,i] * scale),
            int(cy - R[1,i] * scale)
        )
        cv2.arrowedLine(img, (cx,cy), end, colors[i], 3)

# ===============================
# YOLO Detector
# ===============================
class YOLODetector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        print(f"✓ YOLO model loaded: {model_path}")

    def detect(self, image, conf=0.5):
        results = self.model(image, conf=conf, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                detections.append({
                    "bbox": (x1,y1,x2,y2),
                    "center": (cx,cy),
                    "conf": float(box.conf[0]),
                    "name": r.names[int(box.cls[0])]
                })
        return detections

    def draw(self, img, dets):
        out = img.copy()
        for d in dets:
            x1,y1,x2,y2 = d["bbox"]
            cv2.rectangle(out,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.circle(out,d["center"],4,(0,0,255),-1)
            cv2.putText(out,f"{d['name']} {d['conf']:.2f}",
                        (x1,y1-8),cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,(0,255,0),2)
        return out

# ===============================
# RealSense Camera
# ===============================
class RealSenseCamera:
    def __init__(self, w=1280, h=720, fps=30):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        print("✓ RealSense started")

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        c = frames.get_color_frame()
        d = frames.get_depth_frame()
        if not c or not d:
            return None, None, None
        return np.asanyarray(c.get_data()), np.asanyarray(d.get_data()), d

    def stop(self):
        self.pipeline.stop()

# ===============================
# Pose Estimator
# ===============================
class PoseEstimator3D:
    def roi_to_pointcloud(self, depth_frame, bbox, stride=3):
        x1,y1,x2,y2 = bbox
        intr = depth_frame.profile.as_video_stream_profile().intrinsics
        pts = []
        for v in range(y1,y2,stride):
            for u in range(x1,x2,stride):
                d = depth_frame.get_distance(u,v)
                if d <= 0:
                    continue
                pts.append(rs.rs2_deproject_pixel_to_point(intr,[u,v],d))
        return np.array(pts) if len(pts) > 100 else None

    def estimate_pose(self, pts):
        mean = pts.mean(axis=0)
        X = pts - mean

        cov = np.cov(X.T)
        w,v = np.linalg.eigh(cov)
        x_axis = v[:,np.argmax(w)]
        x_axis /= np.linalg.norm(x_axis)

        _,_,vh = np.linalg.svd(X)
        z_axis = vh[-1]
        z_axis /= np.linalg.norm(z_axis)

        y_axis = np.cross(z_axis,x_axis)
        y_axis /= np.linalg.norm(y_axis)

        x_axis = np.cross(y_axis,z_axis)
        x_axis /= np.linalg.norm(x_axis)

        R = np.column_stack((x_axis,y_axis,z_axis))
        return R, mean

# ===============================
# Main
# ===============================
def main():
    MODEL_PATH = "weights/best.pt"  # <<< 改這裡
    cam = RealSenseCamera()
    yolo = YOLODetector(MODEL_PATH)
    pose = PoseEstimator3D()

    last_R = None
    last_euler = None
    last_center = None

    print("\n[s] 計算並顯示 Rotation   [q] 離開\n")

    try:
        while True:
            color, depth, depth_frame = cam.get_frames()
            if color is None:
                continue

            dets = yolo.detect(color)
            vis = yolo.draw(color, dets)

            if last_R is not None:
                draw_axis(vis, last_center, last_R)
                draw_rotation_text(vis, last_R, last_euler)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            if key == ord('s') and len(dets) > 0:
                target = max(dets, key=lambda d: d["conf"])
                pts = pose.roi_to_pointcloud(depth_frame, target["bbox"])
                if pts is None:
                    print("⚠ 點雲不足")
                    continue

                R, c = pose.estimate_pose(pts)
                euler = rotation_to_euler_xyz(R)

                print("\n=== Rotation Matrix ===")
                print(R)
                print("Euler XYZ (deg):", euler)

                last_R = R
                last_euler = euler
                last_center = target["center"]

            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.03),
                cv2.COLORMAP_JET
            )
            cv2.imshow("YOLO + 3D Pose", np.hstack((vis, depth_vis)))

    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("System closed")

if __name__ == "__main__":
    main()
