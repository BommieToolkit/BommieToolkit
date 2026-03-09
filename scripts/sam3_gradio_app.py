import os
import cv2
import torch
import numpy as np
from PIL import Image, ImageDraw
import gradio as gr
from tempfile import NamedTemporaryFile
from sam3.model_builder import build_sam3_video_model

# ---------- Device ----------
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# ---------- SAM-3 ----------
sam3_model = build_sam3_video_model()
predictor = sam3_model.tracker
predictor.backbone = sam3_model.detector.backbone

# ---------- Global State ----------
video_frames = []        # PIL images
video_frames_np = []     # np arrays
inference_state = None
clicks = []              # list of {"x":..., "y":..., "label":1/0}
current_obj_id = 1
frame0_size = None
temp_video_path = None
current_click_type = 1  # 1=positive, 0=negative

# ---------- Utility: convert folder to temporary video ----------
def folder_to_temp_video(folder_path, fps=5):
    global temp_video_path
    allowed_exts = ('.jpg', '.jpeg', '.png')
    img_files = sorted([os.path.join(folder_path, f) for f in os.listdir(folder_path) 
                        if f.lower().endswith(allowed_exts)])
    if len(img_files) == 0:
        raise RuntimeError(f"No images found in {folder_path}")

    first_frame = cv2.imread(img_files[0])
    height, width, _ = first_frame.shape

    temp_file = NamedTemporaryFile(suffix=".mp4", delete=False)
    temp_video_path = temp_file.name
    temp_file.close()

    out = cv2.VideoWriter(temp_video_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    for f in img_files:
        img = cv2.imread(f)
        out.write(img)
    out.release()
    return temp_video_path, img_files

# ---------- Load Folder ----------
def load_folder(folder_path):
    global video_frames, video_frames_np, inference_state, frame0_size, clicks, temp_video_path, video_img_paths
    video_frames = []
    video_frames_np = []
    clicks = []

    video_file, img_paths = folder_to_temp_video(folder_path)
    video_img_paths = img_paths

    for p in img_paths:
        pil_img = Image.open(p).convert("RGB")
        video_frames.append(pil_img)
        video_frames_np.append(np.array(pil_img))

    frame0_size = video_frames_np[0].shape[1], video_frames_np[0].shape[0]

    inference_state = predictor.init_state(video_path=video_file)
    return video_frames_np[0]

# ---------- Set Click Type ----------
def set_positive_mode():
    global current_click_type
    current_click_type = 1
    return "Positive Mode Active"

def set_negative_mode():
    global current_click_type
    current_click_type = 0
    return "Negative Mode Active"

# ---------- Update live preview point ----------
def update_preview(x, y):
    """Draw the current sliders point on top of frame0 and existing clicks"""
    if not video_frames_np:
        return None
    pil = Image.fromarray(video_frames_np[0])
    draw = ImageDraw.Draw(pil)
    # Draw existing clicks
    for c in clicks:
        color = "green" if c["label"]==1 else "red"
        draw.ellipse((c["x"]-5, c["y"]-5, c["x"]+5, c["y"]+5), fill=color)
    # Draw the current point from sliders in blue
    if x is not None and y is not None:
        draw.ellipse((x-5, y-5, x+5, y+5), outline="blue", width=2)
    return pil

def add_click_via_select(evt: gr.SelectData):
    global clicks, frame0_size
    if frame0_size is None:
        return None
    width, height = frame0_size
    x, y = evt.index[0], evt.index[1]
    x = int(np.clip(x, 0, width-1))
    y = int(np.clip(y, 0, height-1))
    clicks.append({"x": x, "y": y, "label": current_click_type})
    return update_preview(x, y)

# ---------- Run Model ----------
def run_model(masks_parent_path):
    global clicks, current_obj_id, frame0_size
    if inference_state is None:
        return "Please load a folder first.", []

    if len(clicks) == 0:
        return "Please add at least one click.", []

    width, height = frame0_size
    points = np.array([[c["x"]/width, c["y"]/height] for c in clicks], dtype=np.float32)
    labels = np.array([c["label"] for c in clicks], dtype=np.int32)

    points_tensor = torch.tensor(points, dtype=torch.float32)
    labels_tensor = torch.tensor(labels, dtype=torch.int32)

    _, out_obj_ids, low_res_masks, video_res_masks = predictor.add_new_points(
        inference_state=inference_state,
        frame_idx=0,
        obj_id=current_obj_id,
        points=points_tensor,
        labels=labels_tensor,
        clear_old_points=False,
    )

    video_segments = {}
    for frame_idx, obj_ids, low_res_masks, video_res_masks, obj_scores in predictor.propagate_in_video(
        inference_state, start_frame_idx=0, max_frame_num_to_track=len(video_frames_np), reverse=False, propagate_preflight=True
    ):
        video_segments[frame_idx] = {
            out_obj_id: (video_res_masks[i] > 0.0).cpu().numpy()
            for i, out_obj_id in enumerate(out_obj_ids)
        }

    masks_output_folder = os.path.join(masks_parent_path, "masks")
    os.makedirs(masks_output_folder, exist_ok=True)
    gallery_imgs = []

    for idx, frame in enumerate(video_frames_np):
      if idx in video_segments:
        pil_frame = Image.fromarray(frame)
        original_filename = os.path.basename(video_img_paths[idx])
        base_name, ext = os.path.splitext(original_filename) 

        for obj_id, mask in video_segments[idx].items():
            mask = mask.squeeze()  # remove extra dims
            # Ensure mask is 2D
            if mask.ndim != 2:
                mask = mask.reshape(frame.shape[0], frame.shape[1])
            mask_img = Image.fromarray((mask*255).astype(np.uint8))
            mask_filename = os.path.join(masks_output_folder, f"{base_name}.png") 
            mask_img.save(mask_filename)
            pil_frame = Image.blend(pil_frame, mask_img.convert("RGB").resize(pil_frame.size), alpha=0.5)
        gallery_imgs.append(pil_frame)

    return "Model run complete!", gallery_imgs

# ---------- Gradio UI ----------
with gr.Blocks() as demo:
    gr.Markdown("## SAM-3 Interactive Folder Segmentation (Gradio 6.3)")

    with gr.Row():
        folder_input = gr.Textbox(label="Folder with images")
        load_btn = gr.Button("Load Folder")

    frame_display = gr.Image(interactive=True, label="Frame 0")  # display only

    mode_text = gr.Textbox(label="Click Mode", interactive=False, value="Positive Mode")

    with gr.Row():
        pos_btn = gr.Button("Positive Mode")
        neg_btn = gr.Button("Negative Mode")

    output_folder_input = gr.Textbox(label="Folder to store masks", value=".")
    run_btn = gr.Button("Run Model")
    gallery_out = gr.Gallery(label="Segmented Frames", show_label=True, scale=4)

    # ---------- Events ----------
    load_btn.click(load_folder, inputs=folder_input, outputs=frame_display)
    pos_btn.click(set_positive_mode, inputs=None, outputs=mode_text)
    neg_btn.click(set_negative_mode, inputs=None, outputs=mode_text)

    # Add click directly by selecting on the image
    frame_display.select(add_click_via_select, inputs=None, outputs=frame_display)
    
    run_btn.click(run_model, inputs=output_folder_input, outputs=[gr.Textbox(), gallery_out])

demo.launch(show_error=True, share=False, server_name="0.0.0.0", server_port=7997, inbrowser=True) 
