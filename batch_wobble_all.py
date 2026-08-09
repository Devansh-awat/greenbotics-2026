import os, glob, json
import cv2
import numpy as np
from multiprocessing import Pool, cpu_count

def process_single_video(f):
    try:
        cap = cv2.VideoCapture(f)
        if not cap.isOpened():
            return f, None, 'Failed to open'

        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps is None or fps <= 0 or np.isnan(fps):
            fps = 10.0

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if total_frames <= 0:
            cap.release()
            return f, None, 'No frames'

        duration_s = total_frames / fps
        if duration_s < 5.0:
            cap.release()
            return f, None, f'Total duration < 5s ({duration_s:.1f}s)'

        # Trimming logic:
        # If duration >= 10s: trim 5s start, 5s end
        # If 5s <= duration < 10s: trim 1.5s start, 1.5s end (preserving core steady state)
        if duration_s >= 10.0:
            trim_start_s = 5.0
            trim_end_s = 5.0
        else:
            trim_start_s = 1.5
            trim_end_s = 1.5

        start_frame = int(trim_start_s * fps)
        end_frame = int(total_frames - trim_end_s * fps)

        if end_frame - start_frame < 10:
            cap.release()
            return f, None, f'Trimmed length too short ({duration_s:.1f}s total)'

        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        ret, prev_frame = cap.read()
        if not ret or prev_frame is None:
            cap.release()
            return f, None, 'Read error at start_frame'

        h, w = prev_frame.shape[:2]
        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

        dx_list, dy_list, da_list = [], [], []
        curr_f = start_frame + 1

        while curr_f < end_frame:
            ret, curr_frame = cap.read()
            if not ret or curr_frame is None:
                break

            curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
            pts_prev = cv2.goodFeaturesToTrack(prev_gray, maxCorners=200, qualityLevel=0.01, minDistance=20)
            
            if pts_prev is not None and len(pts_prev) > 0:
                pts_curr, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, pts_prev, None)
                if status is not None:
                    idx = np.where(status == 1)[0]
                    if len(idx) >= 6:
                        p0 = pts_prev[idx]
                        p1 = pts_curr[idx]
                        m, _ = cv2.estimateAffinePartial2D(p0, p1)
                        if m is not None:
                            dx = m[0, 2]
                            dy = m[1, 2]
                            da = np.arctan2(m[1, 0], m[0, 0])
                            dx_list.append(dx)
                            dy_list.append(dy)
                            da_list.append(da)
                        else:
                            dx_list.append(0); dy_list.append(0); da_list.append(0)
                    else:
                        dx_list.append(0); dy_list.append(0); da_list.append(0)
                else:
                    dx_list.append(0); dy_list.append(0); da_list.append(0)
            else:
                dx_list.append(0); dy_list.append(0); da_list.append(0)

            prev_gray = curr_gray
            curr_f += 1

        cap.release()

        if len(dx_list) < 10:
            return f, None, 'Too few valid frames'

        dx_arr = np.array(dx_list)
        dy_arr = np.array(dy_list)
        da_arr = np.array(da_list)

        cum_y = np.cumsum(dy_arr)
        cum_a = np.cumsum(da_arr)

        win = 7
        kernel = np.ones(win) / win
        padded_y = np.pad(cum_y, (win//2, win//2), mode='edge')
        padded_a = np.pad(cum_a, (win//2, win//2), mode='edge')

        smooth_y = np.convolve(padded_y, kernel, mode='valid')[:len(cum_y)]
        smooth_a = np.convolve(padded_a, kernel, mode='valid')[:len(cum_a)]

        wobble_y_px = cum_y - smooth_y
        wobble_a_deg = np.rad2deg(cum_a - smooth_a)
        wobble_y_percent = (wobble_y_px / h) * 100.0

        da_deg = np.rad2deg(da_arr)
        rot_jerk = np.diff(da_deg) * fps

        res = {
            'path': f,
            'duration_s': float(duration_s),
            'analyzed_frames': len(dx_list),
            'roll_wobble_rms_deg': float(np.sqrt(np.mean(wobble_a_deg**2))),
            'roll_wobble_max_deg': float(np.max(np.abs(wobble_a_deg))),
            'vert_bounce_rms_percent': float(np.sqrt(np.mean(wobble_y_percent**2))),
            'vert_bounce_rms_px': float(np.sqrt(np.mean(wobble_y_px**2))),
            'rotational_jerk_rms': float(np.sqrt(np.mean(rot_jerk**2)))
        }
        return f, res, 'OK'
    except Exception as e:
        return f, None, str(e)

if __name__ == '__main__':
    all_videos = glob.glob('**/*.mp4', recursive=True) + glob.glob('**/*.avi', recursive=True) + glob.glob('**/*.mkv', recursive=True)
    pivot = '2026-04-18_15-06-49'
    ignored_folder = 'obstacle/2026-08-06_23-57-14'

    filtered_videos = [v for v in all_videos if os.path.dirname(v) != ignored_folder]
    print(f'Starting processing of {len(filtered_videos)} total video files...')

    with Pool(processes=cpu_count()) as pool:
        outputs = pool.map(process_single_video, filtered_videos)

    results = {
        'With Wheel Stabilizer (Recent Runs)': [],
        'Without Stabilizer (Current Robot)': [],
        'Older Robot (Before 2026-04-18, Incl 2025)': []
    }
    skipped_counts = {}

    for f, res, status in outputs:
        parts = f.split(os.sep)
        is_old = False
        is_stab = False

        for p in parts:
            if p >= '2026-08-06_23-20-25':
                is_stab = True
            elif p.startswith('2025-') or (p.startswith('2026-') and p < pivot):
                is_old = True

        if is_stab:
            category = 'With Wheel Stabilizer (Recent Runs)'
        elif is_old:
            category = 'Older Robot (Before 2026-04-18, Incl 2025)'
        else:
            category = 'Without Stabilizer (Current Robot)'

        if res is not None:
            results[category].append(res)
        else:
            reason = status.split('(')[0].strip()
            skipped_counts[reason] = skipped_counts.get(reason, 0) + 1

    print('\n=== SKIPPED REASONS ===')
    for k, v in skipped_counts.items():
        print(f'  {k}: {v}')

    print('\n=== SUMMARY RESULTS ===')
    for name, data in results.items():
        print(f'\n--- {name} (Valid Analyzed Videos: {len(data)}) ---')
        if not data:
            continue
        rolls = [d['roll_wobble_rms_deg'] for d in data]
        roll_maxs = [d['roll_wobble_max_deg'] for d in data]
        verts = [d['vert_bounce_rms_percent'] for d in data]
        jerks = [d['rotational_jerk_rms'] for d in data]

        print(f'Roll Wobble RMS (deg): {np.mean(rolls):.4f} +/- {np.std(rolls):.4f}')
        print(f'Roll Wobble Peak Max (deg): {np.mean(roll_maxs):.4f} +/- {np.std(roll_maxs):.4f}')
        print(f'Vertical Bounce RMS (% height): {np.mean(verts):.4f}% +/- {np.std(verts):.4f}%')
        print(f'Rotational Jerk RMS (deg/s^2): {np.mean(jerks):.4f} +/- {np.std(jerks):.4f}')
