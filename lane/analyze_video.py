import argparse
import sys
import os
import torch
import cv2
import numpy as np
import time
import torchvision.transforms as transforms

# Add YOLOP to path
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
YOLOP_DIR = os.path.join(CURRENT_DIR, "yolop_repo")
sys.path.append(YOLOP_DIR)

# Import YOLOP components
from lib.models import get_net
from lib.config import cfg
from lib.core.general import non_max_suppression, scale_coords

# Check for hardware acceleration
def get_device():
    if torch.cuda.is_available():
        return torch.device("cuda")
    elif torch.backends.mps.is_available():
        return torch.device("mps")
    else:
        return torch.device("cpu")

normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
)
transform = transforms.Compose([
    transforms.ToTensor(),
    normalize,
])


def download_youtube(url, output_dir, duration=120):
    """Download a YouTube video, then trim to `duration` seconds with ffmpeg. Returns path to trimmed file."""
    import yt_dlp
    import subprocess
    os.makedirs(output_dir, exist_ok=True)
    output_template = os.path.join(output_dir, "%(title)s.%(ext)s")
    ydl_opts = {
        'format': 'bestvideo[vcodec^=avc1][height<=1080]+bestaudio[ext=m4a]/best[vcodec^=avc1]/best[ext=mp4]/best',
        'outtmpl': output_template,
        'merge_output_format': 'mp4',
        'quiet': False,
    }
    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        info = ydl.extract_info(url, download=True)
        filename = ydl.prepare_filename(info)
        base, _ = os.path.splitext(filename)
        for ext in ['.mp4', '.mkv', '.webm']:
            candidate = base + ext
            if os.path.exists(candidate):
                filename = candidate
                break

    # Trim with ffmpeg (stream copy, no re-encode)
    trimmed = os.path.join(output_dir, "trimmed.mp4")
    subprocess.run([
        "ffmpeg", "-y", "-i", filename,
        "-t", str(duration), "-c", "copy", trimmed
    ], check=True)
    return trimmed


def frame_iterator_bag(bag_path):
    """Yield BGR frames from an mcap bag file."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    decoder = DecoderFactory()
    with open(bag_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[decoder])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            if hasattr(ros_msg, "encoding") and (ros_msg.encoding in ["rgb8", "bgr8", "mono8"] or "8" in ros_msg.encoding):
                if not hasattr(ros_msg, "data"):
                    continue
                img_data = np.frombuffer(ros_msg.data, dtype=np.uint8)
                if ros_msg.encoding == "rgb8":
                    yield cv2.cvtColor(img_data.reshape((ros_msg.height, ros_msg.width, 3)), cv2.COLOR_RGB2BGR)
                elif ros_msg.encoding == "bgr8":
                    yield img_data.reshape((ros_msg.height, ros_msg.width, 3))


def frame_iterator_video(video_path):
    """Yield BGR frames from a video file."""
    cap = cv2.VideoCapture(video_path)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        yield frame
    cap.release()


def run_yolop(frame_iter, weights_path, source_name):
    device = get_device()
    print(f"Using device: {device}")

    print(f"Loading YOLOP model from {weights_path}")
    model = get_net(cfg)
    checkpoint = torch.load(weights_path, map_location=device)
    model.load_state_dict(checkpoint['state_dict'])
    model.to(device)
    model.eval()
    names = model.module.names if hasattr(model, 'module') else model.names

    print(f"Processing: {source_name}")
    print("Press 'q' to quit")

    window_name = "YOLOP - Drivable Area (Green) | Lanes (Red) | Detections (Yellow)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    count = 0
    for img_bgr in frame_iter:
        img_h, img_w = img_bgr.shape[:2]
        img_size = 640

        r = min(img_size / img_h, img_size / img_w)
        new_unpad_h, new_unpad_w = int(round(img_h * r)), int(round(img_w * r))
        pad_h = (img_size - new_unpad_h) / 2
        pad_w = (img_size - new_unpad_w) / 2

        img_resized = cv2.resize(img_bgr, (new_unpad_w, new_unpad_h), interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        img_padded = cv2.copyMakeBorder(img_resized, top, bottom, left, right,
                                        cv2.BORDER_CONSTANT, value=(114, 114, 114))

        img_rgb_padded = cv2.cvtColor(img_padded, cv2.COLOR_BGR2RGB)
        inputs = transform(img_rgb_padded).to(device).unsqueeze(0)

        t0 = time.time()
        with torch.no_grad():
            outputs = model(inputs)
            det_out = outputs[0]
            da_seg_out = outputs[1]
            ll_seg_out = outputs[2]

        inf_out, _ = det_out
        det_pred = non_max_suppression(inf_out, conf_thres=0.2, iou_thres=0.45, classes=None, agnostic=False)
        det = det_pred[0]

        _, _, height, width = inputs.shape
        pad_h_int = int(round(pad_h))
        pad_w_int = int(round(pad_w))

        da_predict = da_seg_out[:, :, pad_h_int:(height - pad_h_int), pad_w_int:(width - pad_w_int)]
        da_seg_mask = torch.nn.functional.interpolate(da_predict, size=(img_h, img_w), mode='bilinear')
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()

        ll_predict = ll_seg_out[:, :, pad_h_int:(height - pad_h_int), pad_w_int:(width - pad_w_int)]
        ll_seg_mask = torch.nn.functional.interpolate(ll_predict, size=(img_h, img_w), mode='bilinear')
        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()

        if device.type == 'cuda':
            torch.cuda.synchronize()
        elif device.type == 'mps':
            try:
                torch.mps.synchronize()
            except AttributeError:
                pass

        t1 = time.time()
        dt_ms = (t1 - t0) * 1000.0

        vis_img = img_bgr.copy()
        color_area = np.zeros_like(vis_img)
        color_area[da_seg_mask == 1] = [0, 255, 0]
        color_area[ll_seg_mask == 1] = [0, 0, 255]
        vis_img = cv2.addWeighted(vis_img, 1, color_area, 0.5, 0)

        if det is not None and len(det):
            det[:, :4] = scale_coords(inputs.shape[2:], det[:, :4], vis_img.shape).round()
            for *xyxy, conf, cls in reversed(det):
                label = f'{names[int(cls)]} {conf:.2f}'
                c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                color = (0, 255, 255)
                cv2.rectangle(vis_img, c1, c2, color, 2)
                cv2.putText(vis_img, label, (c1[0], c1[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        cv2.putText(vis_img, f'{dt_ms:.0f}ms ({1000/dt_ms:.1f} FPS)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(window_name, vis_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        count += 1
        if count % 10 == 0:
            print(f"Processed {count} frames. {dt_ms:.0f}ms/frame", end='\r')

    print(f"\nDone. Processed {count} frames.")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lane detection with YOLOP")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=str, help="Path to .mcap bag file")
    source.add_argument("--video", type=str, help="Path to a video file (mp4, etc.)")
    source.add_argument("--youtube", type=str, help="YouTube URL to download and process")
    parser.add_argument("--weights", type=str, default="lane/yolop_repo/weights/End-to-end.pth")
    parser.add_argument("--duration", type=int, default=120, help="Seconds to download from YouTube (default: 120)")

    args = parser.parse_args()

    # Resolve weights
    if not os.path.exists(args.weights):
        alt_weights = os.path.join(YOLOP_DIR, "weights", "End-to-end.pth")
        if os.path.exists(alt_weights):
            args.weights = alt_weights
        else:
            print(f"Error: Weights not found at {args.weights}")
            sys.exit(1)

    # Build frame iterator based on source
    if args.bag:
        frames = frame_iterator_bag(args.bag)
        name = args.bag
    elif args.video:
        if not os.path.exists(args.video):
            print(f"Error: Video not found at {args.video}")
            sys.exit(1)
        frames = frame_iterator_video(args.video)
        name = args.video
    elif args.youtube:
        download_dir = os.path.join(CURRENT_DIR, "downloads")
        print(f"Downloading first {args.duration}s from: {args.youtube}")
        video_path = download_youtube(args.youtube, download_dir, args.duration)
        print(f"Downloaded to: {video_path}")
        frames = frame_iterator_video(video_path)
        name = video_path

    run_yolop(frames, args.weights, name)
