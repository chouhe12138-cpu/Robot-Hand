"""Offline preliminary analysis; writes no robot TF and sends no robot commands."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


from object_hand.calibration import transform, mean_pose, errors, solve


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--square-mm', type=float, default=30)
    parser.add_argument('--marker-mm', type=float, default=22)
    args = parser.parse_args()
    analyze_session(args.session, args.square_mm, args.marker_mm)


def analyze_session(root, square_mm=30., marker_mm=22.):
    root = Path(root)
    session = json.loads((root/'session.json').read_text())
    k = np.array(session['camera_matrix'], dtype=float)
    dist = np.array(session['distortion_coefficients'], dtype=float)
    # This printed board is 7 columns by 5 rows. Initial session metadata was unordered 5x7.
    board = cv2.aruco.CharucoBoard((7, 5), square_mm / 1000, marker_mm / 1000,
                                  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    detector = cv2.aruco.CharucoDetector(board)
    output = root/'preliminary_analysis'
    output.mkdir(exist_ok=True)
    rows, robots, boards = [], [], []
    for path in sorted(root.glob('*.png')):
        image = cv2.imread(str(path))
        corners, ids, mc, mi = detector.detectBoard(image)
        row = {'image': path.name, 'corners': 0 if ids is None else len(ids)}
        overlay = image.copy()
        if ids is None or len(ids) < 8 or board.checkCharucoCornersCollinear(ids):
            row['accepted'] = False
            row['reason'] = 'too few or collinear corners'
            rows.append(row)
            continue
        cv2.aruco.drawDetectedCornersCharuco(overlay, corners, ids)
        obj, img = board.matchImagePoints(corners, ids)
        ok, rv, tv = cv2.solvePnP(obj, img, k, dist)
        if not ok:
            row.update(accepted=False, reason='PnP failure'); rows.append(row); continue
        projected, _ = cv2.projectPoints(obj, rv, tv, k, dist)
        residual = projected.reshape(-1, 2) - img.reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
        data = json.loads(path.with_suffix('.json').read_text())
        before, after = data['robot_before_image'], data['robot_after_image']
        movement = float(np.max(np.abs(np.array(after['actual_q_rad']) - before['actual_q_rad'])))
        row.update(reprojection_rms_px=rms, accepted=bool(rms <= 1.5 and movement <= .001 and tv[2,0]>0),
                   robot_bracket_joint_change_rad=movement,
                   T_camera_board=transform(rv, tv).tolist())
        cv2.drawFrameAxes(overlay, k, dist, rv, tv, .05)
        cv2.imwrite(str(output / path.name), overlay)
        rows.append(row)
        if row['accepted']:
            pose = after['actual_tcp_pose_m_rotvec_rad']
            robots.append(transform(pose[3:], pose[:3]))
            boards.append(transform(rv, tv))
    result = {
        'status': 'PRELIMINARY_UNVERIFIED_DO_NOT_USE_FOR_MOTION',
        'assumptions': ['Board rigidly fixed during acquisition', 'Printed square size correct',
                        'Active TCP constant; TCP offset and ROS frame mapping unresolved'],
        'board_model': {'columns': 7, 'rows': 5, 'square_mm': square_mm,
                        'marker_mm': marker_mm,
                        'dictionary': 'DICT_4X4_50 matches observed IDs 0..16; larger 4x4 dictionaries share these IDs'},
        'images': rows, 'accepted_samples': len(robots), 'solvers': {},
    }
    if len(robots) >= 8:
        held = [i for i in range(len(robots)) if i % 5 == 4]
        train = [i for i in range(len(robots)) if i not in held]
        for name in ['TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS']:
            method = getattr(cv2, 'CALIB_HAND_EYE_' + name)
            try:
                x = solve(robots, boards, method)
                fixed = [a @ x @ b for a, b in zip(robots, boards)]
                err = errors(fixed, mean_pose(fixed))
                xt = solve([robots[i] for i in train], [boards[i] for i in train], method)
                ref = mean_pose([robots[i] @ xt @ boards[i] for i in train])
                test = errors([robots[i] @ xt @ boards[i] for i in held], ref)
                loo = []
                for leave in range(len(robots)):
                    subset = [i for i in range(len(robots)) if i != leave]
                    xl = solve([robots[i] for i in subset], [boards[i] for i in subset], method)
                    loo.append(xl)
                result['solvers'][name] = {
                    'T_active_tcp_camera': x.tolist(),
                    'fixed_board_consistency': err,
                    'heldout_image_names': [r['image'] for i,r in enumerate([r for r in rows if r['accepted']]) if i in held],
                    'heldout_errors': test,
                    'leave_one_out_transform_changes': errors(loo, x),
                }
                print(name, 'board RMS mm', round(float(np.sqrt(np.mean(np.array(err['translation_mm'])**2))), 3),
                      'rotation RMS deg', round(float(np.sqrt(np.mean(np.array(err['rotation_deg'])**2))), 3),
                      'heldout mm', np.round(test['translation_mm'], 3).tolist(), flush=True)
            except (ValueError, cv2.error) as e:
                result['solvers'][name] = {'error': str(e)}
    (output/'report.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('Accepted:', len(robots), '/', len(rows), 'Report:', output/'report.json', flush=True)
    print('Image RMS px:', [round(r.get('reprojection_rms_px', -1), 3) for r in rows], flush=True)
    return result


if __name__ == '__main__':
    main()
