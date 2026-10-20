"""Validate frozen ZED hand-eye transform on a separate capture session; no refitting."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from object_hand.calibration import transform, mean_pose, errors


def summarize(error):
    return {kind: {'rms': float(np.sqrt(np.mean(np.square(values)))),
                   'max': float(np.max(values)), 'values': values} for kind,values in error.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration', type=Path)
    parser.add_argument('validation_session', type=Path)
    args = parser.parse_args()
    calibration = json.loads(args.calibration.read_text())
    original = Path(calibration['source_session'])
    original_report = json.loads((original/'preliminary_analysis/report.json').read_text())
    tool_tcp = np.array(calibration['T_tool0_active_tcp'])
    tool_camera = np.array(calibration['T_tool0_camera'])
    original_fixed = []
    for row in original_report['images']:
        if row['accepted']:
            data = json.loads((original/row['image']).with_suffix('.json').read_text())
            p = data['robot_after_image']['actual_tcp_pose_m_rotvec_rad']
            original_fixed.append(transform(p[3:],p[:3]) @ np.linalg.inv(tool_tcp) @ tool_camera @ np.array(row['T_camera_board']))
    reference = mean_pose(original_fixed)
    root = args.validation_session
    session = json.loads((root/'session.json').read_text())
    if session['camera_serial'] != json.loads((original/'session.json').read_text())['camera_serial']:
        raise ValueError('Camera serial mismatch')
    k = np.array(session['camera_matrix'],float)
    distortion = np.array(session['distortion_coefficients'],float)
    model = calibration['board_model']
    board = cv2.aruco.CharucoBoard((model['columns'],model['rows']),model['square_mm']/1000,
                                  model['marker_mm']/1000, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    detector = cv2.aruco.CharucoDetector(board)
    output = root/'validation_frozen_calibration'
    output.mkdir(exist_ok=True)
    rows, fixed, joints, tcp_rotations = [], [], [], []
    for path in sorted(root.glob('*.png')):
        image = cv2.imread(str(path))
        corners, ids, _, _ = detector.detectBoard(image)
        row = {'image':path.name, 'charuco_corners':0 if ids is None else len(ids)}
        if ids is None or len(ids)<8 or board.checkCharucoCornersCollinear(ids):
            row.update(accepted=False, reason='insufficient/non-collinear board points'); rows.append(row); continue
        obj,img = board.matchImagePoints(corners,ids)
        ok, rv, tv = cv2.solvePnP(obj,img,k,distortion)
        if not ok:
            row.update(accepted=False,reason='PnP failed'); rows.append(row); continue
        projected,_=cv2.projectPoints(obj,rv,tv,k,distortion)
        rms=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-img.reshape(-1,2))**2,axis=1))))
        raw=json.loads(path.with_suffix('.json').read_text())
        before,after=raw['robot_before_image'],raw['robot_after_image']
        dq=float(np.max(np.abs(np.array(after['actual_q_rad'])-before['actual_q_rad'])))
        row.update(accepted=bool(rms<1.5 and dq<=.001 and tv[2,0]>0),reprojection_rms_px=rms,
                   bracket_joint_change_rad=dq)
        cv2.aruco.drawDetectedCornersCharuco(image,corners,ids)
        cv2.drawFrameAxes(image,k,distortion,rv,tv,.05)
        cv2.imwrite(str(output/path.name),image)
        if row['accepted']:
            p=after['actual_tcp_pose_m_rotvec_rad']
            pose=transform(p[3:],p[:3]) @ np.linalg.inv(tool_tcp) @ tool_camera @ transform(rv,tv)
            row['T_ur_base_board']=pose.tolist()
            fixed.append(pose); joints.append(after['actual_q_rad']);tcp_rotations.append(p[3:])
        rows.append(row)
    if len(fixed)<3:
        raise ValueError('Fewer than 3 valid validation images')
    centre=mean_pose(fixed)
    max_angle=0.
    rotations=Rotation.from_rotvec(np.array(tcp_rotations))
    for i in range(len(rotations)):
        for j in range(i):
            max_angle=max(max_angle,float(np.rad2deg((rotations[i].inv()*rotations[j]).magnitude())))
    result={
        'kind':'FROZEN_EXTRINSIC_NEW_SESSION_VALIDATION_NOT_ABSOLUTE_ACCURACY',
        'calibration_file':str(args.calibration.resolve()),
        'calibration_sha256':hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
        'refitted':False, 'accepted_samples':len(fixed), 'total_samples':len(rows),
        'compared_to_original_fixed_board':summarize(errors(fixed,reference)),
        'within_new_session_consistency':summarize(errors(fixed,centre)),
        'new_session_mean_shift_from_original_mm':((centre[:3,3]-reference[:3,3])*1000).tolist(),
        'joint_ranges_deg':np.rad2deg(np.ptp(joints,axis=0)).tolist(),
        'maximum_pairwise_tcp_orientation_change_deg':max_angle,
        'T_ur_base_board_reference':reference.tolist(), 'images':rows,
        'assumptions':['Board remained at original location; TCP unchanged at (0,0,0.135m,0,0,0rad)',
                       'Image pose computed using intrinsics from this new session'],
        'limitation':'This tests board-pose consistency; absolute positioning, ROS FK and actual gripper TCP accuracy still require checking.',
    }
    (output/'report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({key:value for key,value in result.items() if key not in ['images','T_ur_base_board_reference']},indent=2))
    print('Saved:',output/'report.json')


if __name__=='__main__':
    main()
